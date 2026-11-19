#!/usr/bin/env python3
"""
vcd_system_view.py

Visualises the testbench VCD output.

  tt    tb_tt_smoke.vcd   ASIC handshake with the external SoC, sensors and rails
  fsm   tb_power_fsm.vcd  same view, using internal signal names
  dvfs  tb_dvfs_ctrl.vcd  DVFS ports; checks the order of voltage and clock steps

  python vcd_system_view.py tb_tt_smoke.vcd               # timeline + event log
  python vcd_system_view.py tb_tt_smoke.vcd --at 60       # block diagram at cycle 60
  python vcd_system_view.py tb_tt_smoke.vcd --gif soc.gif # animated block diagram
  python vcd_system_view.py tb_power_fsm.vcd --preset fsm
  python vcd_system_view.py tb_dvfs_ctrl.vcd --preset dvfs

Needs matplotlib (pillow too, if you want --gif).

Things to check against your RTL:
  * tt: VID1/VID0 are assumed to be uo_out[4]/uo_out[3].
  * tt: sensor pins are active low (warn_n, crit_n, oc_n), so they are inverted here.
  * dvfs: the DUT's internals are not in the VCD, so ordering is judged from ports only.
"""

import argparse
import re
import bisect
import os
import sys
from collections import defaultdict
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch, Rectangle

SAMPLE_OFFSET = 1  # sample 1 time unit after each rising CLK, like the tb's "#1"


# ----------------------------------------------------------------------------
# VCD reading
# ----------------------------------------------------------------------------
class Vcd:
    def __init__(self, path):
        self.names = {}                     # full signal name -> id code
        self.width = {}                     # id code -> bit width
        self.times = defaultdict(list)
        self.vals = defaultdict(list)
        self.top = None
        self.time_unit_ns = None
        self._read(path)

    @staticmethod
    def _timescale_ns(tokens):
        scale = "".join(token for token in tokens if token != "$end")
        match = re.fullmatch(r"(\d+)(s|ms|us|ns|ps|fs)", scale)
        if match is None:
            raise ValueError(f"unsupported VCD timescale: {scale!r}")
        unit_ns = {"s": 1e9, "ms": 1e6, "us": 1e3,
                   "ns": 1, "ps": 1e-3, "fs": 1e-6}
        return int(match.group(1)) * unit_ns[match.group(2)]

    def _read(self, path):
        scope, t, in_defs, timescale_tokens = [], 0, True, None
        with open(path) as f:
            for line in f:
                tok = line.split()
                if not tok:
                    continue
                if in_defs:
                    if timescale_tokens is not None:
                        timescale_tokens.extend(tok)
                        if "$end" in tok:
                            self.time_unit_ns = self._timescale_ns(timescale_tokens)
                            timescale_tokens = None
                    elif tok[0] == "$timescale":
                        timescale_tokens = tok[1:]
                        if "$end" in timescale_tokens:
                            self.time_unit_ns = self._timescale_ns(timescale_tokens)
                            timescale_tokens = None
                    elif tok[0] == "$scope":
                        scope.append(tok[2])
                        self.top = self.top or tok[2]
                    elif tok[0] == "$upscope":
                        scope.pop()
                    elif tok[0] == "$var":
                        width, code, name = int(tok[2]), tok[3], tok[4]
                        self.names.setdefault(".".join(scope + [name]), code)
                        self.width[code] = width
                    elif tok[0] == "$enddefinitions":
                        in_defs = False
                    continue
                head = tok[0]
                if head.startswith("#"):
                    t = int(head[1:])
                elif head[0] in "bB":
                    self._record(tok[1], t, head[1:])
                elif head[0] in "01xXzZ" and len(head) > 1:
                    self._record(head[1:], t, head[0])

    def _record(self, code, t, val):
        if self.times[code] and self.times[code][-1] == t:
            self.vals[code][-1] = val       # last change in a timestep wins
        else:
            self.times[code].append(t)
            self.vals[code].append(val)

    def bit(self, full, idx, t):
        """Value of bit idx of a signal at time t, as 0 or 1 (x/z read as 0)."""
        code = self.names[full]
        i = bisect.bisect_right(self.times[code], t) - 1
        if i < 0:
            return 0
        v, w = self.vals[code][i], self.width[code]
        v = v.rjust(w, "0")
        return 1 if v[w - 1 - idx] == "1" else 0

    def rising_edges(self, full):
        code = self.names[full]
        ts, vs = self.times[code], self.vals[code]
        return [t for t, v, p in zip(ts[1:], vs[1:], vs[:-1]) if v == "1" and p != "1"]


# ----------------------------------------------------------------------------
# Pins and lanes for each testbench
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class Pin:
    name: str           # label used in the plots
    var: str            # VCD variable under the top scope
    bit: int = None     # bit index for buses, None for scalars
    active_low: bool = False


# Pin level, from tb_tt_smoke.v
TT_PINS = [
    Pin("PWR_REQ",   "uio_in", 4),
    Pin("PERF_REQ1", "uio_in", 6),
    Pin("PERF_REQ0", "uio_in", 5),
    Pin("FAULT_CLR", "ui_in", 7),
    Pin("WARN",      "ui_in", 3, active_low=True),
    Pin("CRIT",      "ui_in", 4, active_low=True),
    Pin("OC",        "ui_in", 5, active_low=True),
    Pin("VREG_ACK",  "ui_in", 6),
    Pin("PG_A",      "ui_in", 0),
    Pin("PG_B",      "ui_in", 1),
    Pin("PG_C",      "ui_in", 2),
    Pin("EN_A",      "uo_out", 0),
    Pin("EN_B",      "uo_out", 1),
    Pin("EN_C",      "uo_out", 2),
    Pin("VID1",      "uo_out", 4),
    Pin("VID0",      "uo_out", 3),
    Pin("CLK_OUT",   "uo_out", 5),
    Pin("FAULT",     "uo_out", 6),
    Pin("DVFS_EN",   "uo_out", 7),
    Pin("THROTTLED", "uio_out", 7),
]

# Internal names from tb_power_fsm.v
FSM_PINS = [
    Pin("PWR_REQ",   "pwr_req"),
    Pin("PERF_REQ1", "perf_req", 1),
    Pin("PERF_REQ0", "perf_req", 0),
    Pin("FAULT_CLR", "fault_clr"),
    Pin("WARN",      "therm_warn"),
    Pin("CRIT",      "therm_crit"),
    Pin("OC",        "oc"),
    Pin("PG_A",      "pg_a"),
    Pin("PG_B",      "pg_b"),
    Pin("PG_C",      "pg_c"),
    Pin("EN_A",      "en_a"),
    Pin("EN_B",      "en_b"),
    Pin("EN_C",      "en_c"),
    Pin("DVFS_EN",   "dvfs_enable"),
    Pin("FAULT",     "in_fault"),
    Pin("THROTTLED", "in_throttled"),
]

# Port level from tb_dvfs_ctrl.v. The DUT's internal state is not in this VCD,
# so the ordering check uses the voltage code and clock divide code outputs.
DVFS_PINS = [
    Pin("RST_N",       "rst_n"),
    Pin("DVFS_REQ1",   "dvfs_req", 1),
    Pin("DVFS_REQ0",   "dvfs_req", 0),
    Pin("THROTTLED",   "throttled"),
    Pin("FSM_FAULT",   "fsm_fault"),
    Pin("VREG_ACK",    "vreg_ack"),
    Pin("CLK_HI_ACK1", "clk_hi_ack", 1),
    Pin("CLK_HI_ACK0", "clk_hi_ack", 0),
    Pin("V1",          "v_code", 1),
    Pin("V0",          "v_code", 0),
    Pin("CLKDIV1",     "clk_div_code", 1),
    Pin("CLKDIV0",     "clk_div_code", 0),
    Pin("BUSY",        "dvfs_busy"),
    Pin("DVFS_FAULT",  "dvfs_fault"),
]

PRESETS = {"tt": TT_PINS, "fsm": FSM_PINS, "dvfs": DVFS_PINS}

# Timeline lanes: (group, label, pins). Two pins make a 2-bit bus lane.
# A lane is dropped if any of its pins is missing from the VCD.
LANES = [
    ("SoC",    "PWR_REQ",       ("PWR_REQ",)),
    ("SoC",    "PERF_REQ[1:0]", ("PERF_REQ1", "PERF_REQ0")),
    ("SoC",    "FAULT_CLR",     ("FAULT_CLR",)),
    ("Sensor", "WARN",          ("WARN",)),
    ("Sensor", "CRIT",          ("CRIT",)),
    ("Sensor", "OC",            ("OC",)),
    ("ASIC",   "DVFS_EN",       ("DVFS_EN",)),
    ("ASIC",   "THROTTLED",     ("THROTTLED",)),
    ("ASIC",   "FAULT",         ("FAULT",)),
    ("ASIC",   "EN_A",          ("EN_A",)),
    ("ASIC",   "EN_B",          ("EN_B",)),
    ("ASIC",   "EN_C",          ("EN_C",)),
    ("ASIC",   "VID[1:0]",      ("VID1", "VID0")),
    ("Rails",  "PG_A",          ("PG_A",)),
    ("Rails",  "PG_B",          ("PG_B",)),
    ("Rails",  "PG_C",          ("PG_C",)),
    ("Rails",  "VREG_ACK",      ("VREG_ACK",)),
]

DVFS_LANES = [
    ("Request", "RST_N",           ("RST_N",)),
    ("Request", "DVFS_REQ[1:0]",   ("DVFS_REQ1", "DVFS_REQ0")),
    ("Request", "THROTTLED",       ("THROTTLED",)),
    ("Request", "FSM_FAULT",       ("FSM_FAULT",)),
    ("Ack",     "VREG_ACK",        ("VREG_ACK",)),
    ("Ack",     "CLK_HI_ACK[1:0]", ("CLK_HI_ACK1", "CLK_HI_ACK0")),
    ("Output",  "V_CODE[1:0]",     ("V1", "V0")),
    ("Output",  "CLK_DIV[1:0]",    ("CLKDIV1", "CLKDIV0")),
    ("Output",  "BUSY",            ("BUSY",)),
    ("Output",  "DVFS_FAULT",      ("DVFS_FAULT",)),
]

GROUP_COL = {"SoC": "#2b6cb0", "Sensor": "#c53030", "ASIC": "#2f855a", "Rails": "#6b46c1",
             "Request": "#2b6cb0", "Ack": "#6b46c1", "Output": "#2f855a"}
STATE_COL = {"OFF": "#e2e8f0", "STARTUP": "#fefcbf", "RUN": "#c6f6d5",
             "THROTTLED": "#feebc8", "FAULT": "#fed7d7", "SHUTDOWN": "#e9d8fd",
             "STEADY": "#c6f6d5", "RAMP": "#fefcbf"}


# ----------------------------------------------------------------------------
# Turn the waveform into one row per clock cycle
# ----------------------------------------------------------------------------
def lane_value(row, names):
    v = 0
    for n in names:
        v = (v << 1) | row[n]
    return v


def infer_state(row):
    """ASIC state from its outputs, so it works without internal signals."""
    if row.get("FAULT"):
        return "FAULT"
    if row.get("DVFS_EN"):
        return "THROTTLED" if row.get("THROTTLED") else "RUN"
    if any(row.get(n) for n in ("EN_A", "EN_B", "EN_C")):
        return "STARTUP" if row.get("PWR_REQ") else "SHUTDOWN"
    return "OFF"


def dvfs_state(row):
    if row.get("DVFS_FAULT"):
        return "FAULT"
    if row.get("BUSY"):
        return "RAMP"
    return "STEADY"


def sample_rows(vcd, top, pins, clk, state_fn):
    rows = []
    for t in vcd.rising_edges(f"{top}.{clk}"):
        row = {"_t": t}
        for p in pins:
            b = vcd.bit(f"{top}.{p.var}", p.bit or 0, t + SAMPLE_OFFSET)
            row[p.name] = b ^ 1 if p.active_low else b
        row["STATE"] = state_fn(row)
        rows.append(row)
    return rows


def edges_of(rows, name, direction):
    return [c for c in range(1, len(rows))
            if rows[c][name] == direction and rows[c - 1][name] != direction]


def rail_handshakes(rows):
    """Pair each EN_x change with the PG_x change that answers it, before the next EN_x change."""
    out = []
    for rail in "ABC":
        en, pg = f"EN_{rail}", f"PG_{rail}"
        if en not in rows[0] or pg not in rows[0]:
            continue
        for d in (1, 0):
            causes = edges_of(rows, en, d)
            effects = edges_of(rows, pg, d)
            for i, cc in enumerate(causes):
                limit = causes[i + 1] if i + 1 < len(causes) else len(rows)
                nxt = [e for e in effects if cc <= e < limit]
                out.append((rail, d, cc, nxt[0] if nxt else None))
    return sorted(out, key=lambda x: x[2])


def event_log(rows, names):
    ev = []
    for n in names:
        if n == "CLK_OUT":
            continue
        for c in range(1, len(rows)):
            if rows[c][n] != rows[c - 1][n]:
                ev.append((c, f"{n} {'rises' if rows[c][n] else 'falls'}"))
    for c in range(1, len(rows)):
        if rows[c]["STATE"] != rows[c - 1]["STATE"]:
            ev.append((c, f"state -> {rows[c]['STATE']}"))
    return sorted(ev)


def print_report(rows, events, pairs):
    print("\n== event log ==")
    for c, text in events:
        print(f"{rows[c]['_t']:>8} ns  cyc {c:>5}  {text}")
    print("\n== rail handshakes (EN -> PG) ==")
    for rail, d, cc, ce in pairs:
        word = "rise" if d else "fall"
        if ce is None:
            print(f"rail {rail} {word}: EN_{rail} at cyc {cc}, NO PG response")
        else:
            dt = rows[ce]["_t"] - rows[cc]["_t"]
            print(f"rail {rail} {word}: EN_{rail} cyc {cc} -> PG_{rail} cyc {ce}"
                  f"  (+{ce - cc} cycles, {dt} ns)")


# ----------------------------------------------------------------------------
# DVFS: find each transition (dvfs_busy high) and check the order of steps
# ----------------------------------------------------------------------------
V_NAMES = ("V1", "V0")
K_NAMES = ("CLKDIV1", "CLKDIV0")
REQ_NAMES = ("DVFS_REQ1", "DVFS_REQ0")
ACK_NAMES = ("CLK_HI_ACK1", "CLK_HI_ACK0")


def dvfs_windows(rows):
    """(start, end) cycles where dvfs_busy is high."""
    wins, c, n = [], 0, len(rows)
    while c < n:
        if rows[c]["BUSY"]:
            s = c
            while c < n and rows[c]["BUSY"]:
                c += 1
            wins.append((s, c - 1))
        else:
            c += 1
    return wins


def rising_in(rows, name, lo, hi):
    return [c for c in range(max(lo, 1), hi + 1)
            if rows[c][name] and not rows[c - 1][name]]


def changes_in(vals, lo, hi):
    return [c for c in range(max(lo, 1), hi + 1) if vals[c] != vals[c - 1]]


def check_dvfs(rows, wins):
    """Returns (per-transition results, code changes made outside any busy window)."""
    n = len(rows)
    v = [lane_value(r, V_NAMES) for r in rows]
    k = [lane_value(r, K_NAMES) for r in rows]
    a = [lane_value(r, ACK_NAMES) for r in rows]
    covered, results = set(), []

    for s, e in wins:
        lo, hi = max(s - 1, 0), min(e + 1, n - 1)
        covered.update(range(lo, hi + 1))
        v0, v1, k0, k1 = v[lo], v[hi], k[lo], k[hi]
        v_steps = changes_in(v, lo, hi)
        k_steps = changes_in(k, lo, hi)

        if v0 == v1 and k0 == k1:
            direction = "SAME"
        else:
            direction = "UP" if v1 + k1 > v0 + k0 else "DOWN"

        # the requested level at the end of the window (throttle forces the lowest)
        target = 0 if rows[hi]["THROTTLED"] else lane_value(rows[hi], REQ_NAMES)

        problems, note = [], ""
        if v1 != target or k1 != target:
            problems.append(f"ends at V={v1:02b} CLK={k1:02b}, expected {target:02b}")

        if v_steps and k_steps:
            if v_steps[0] == k_steps[0]:
                problems.append("V and CLK change in the same cycle, so order is not enforced")
            elif direction == "UP" and k_steps[0] < v_steps[0]:
                problems.append("clock sped up before voltage")
            elif direction == "DOWN" and v_steps[0] < k_steps[0]:
                problems.append("voltage dropped before clock")
            else:
                note = ("voltage first, then clock" if direction == "UP"
                        else "clock first, then voltage")
        elif v_steps or k_steps:
            note = "only one domain changed"

        results.append({
            "s": s, "e": e, "dir": direction,
            "v0": v0, "v1": v1, "k0": k0, "k1": k1,
            "v_steps": v_steps, "k_steps": k_steps,
            "vack": rising_in(rows, "VREG_ACK", lo, hi),
            "kack": changes_in(a, lo, hi),
            "problems": problems, "note": note,
        })

    stray = [c for c in range(1, n)
             if (v[c] != v[c - 1] or k[c] != k[c - 1])
             and c not in covered and rows[c].get("RST_N", 1)]
    return results, stray


def print_dvfs_report(rows, events, results, stray):
    print("\n== event log ==")
    for c, text in events:
        print(f"{rows[c]['_t']:>8} ns  cyc {c:>5}  {text}")

    print("\n== transitions ==")
    if not results:
        print("no dvfs_busy windows found")
    for i, r in enumerate(results, 1):
        print(f"#{i}  cyc {r['s']}-{r['e']}  {r['dir']:<4}  "
              f"V {r['v0']:02b}->{r['v1']:02b}  CLK {r['k0']:02b}->{r['k1']:02b}")
        print(f"     V steps at {r['v_steps']}, CLK steps at {r['k_steps']}")
        print(f"     vreg_ack rises at {r['vack']}, clk_hi_ack changes at {r['kack']}")
        if r["problems"]:
            for p in r["problems"]:
                print(f"     FAIL: {p}")
        else:
            print(f"     OK: {r['note'] or 'codes reached target'}")
    for c in stray:
        print(f"FAIL: code changed outside a busy window at cyc {c} ({rows[c]['_t']} ns)")

    nbad = sum(1 for r in results if r["problems"]) + len(stray)
    print(f"\n{'PASS' if nbad == 0 else 'FAIL'}: {len(results)} transition(s), {nbad} problem(s)")


def draw_windows(ax, top, results):
    for i, r in enumerate(results, 1):
        ok = not r["problems"]
        col = "#2f855a" if ok else "#c53030"
        ax.axvspan(r["s"] - 0.5, r["e"] + 1.5, color=col, alpha=0.10, zorder=0)
        ax.text((r["s"] + r["e"]) / 2, top + 1.0,
                f"#{i} {r['dir']} {'OK' if ok else 'FAIL'}",
                ha="center", fontsize=8, color=col, fontweight="bold")


# ----------------------------------------------------------------------------
# Timeline with handshake arrows
# ----------------------------------------------------------------------------
def draw_bus(ax, rows, names, yb, col):
    vals = [lane_value(r, names) for r in rows]
    w, s = len(names), 0
    for c in range(1, len(vals) + 1):
        if c == len(vals) or vals[c] != vals[s]:
            ax.add_patch(Rectangle((s, yb), c - s, 0.8, fc="white", ec=col, lw=1.2))
            if c - s >= 4:
                ax.text((s + c) / 2, yb + 0.4, format(vals[s], f"0{w}b"),
                        ha="center", va="center", fontsize=7, color=col)
            s = c


def plot_timeline(rows, lanes, pairs, title, path,
                  state_label="[ASIC] state", overlay=None):
    n = len(rows)
    L = len(lanes)
    base = {}
    for i, (_, label, _) in enumerate(lanes):
        base[label] = (L - 1 - i) * 1.3
    top = L * 1.3

    fig, ax = plt.subplots(figsize=(14, 0.5 * (L + 3)))
    xs = list(range(n))
    for grp, label, names in lanes:
        yb, col = base[label], GROUP_COL[grp]
        if len(names) == 1:
            ys = [yb + 0.8 * rows[c][names[0]] for c in xs]
            ax.step(xs, ys, where="post", color=col, lw=1.6)
        else:
            draw_bus(ax, rows, names, yb, col)
        ax.axhline(yb - 0.25, color="#edf2f7", lw=0.6, zorder=0)

    # state band along the top
    states = [r["STATE"] for r in rows]
    s = 0
    for c in range(1, n + 1):
        if c == n or states[c] != states[s]:
            ax.add_patch(Rectangle((s, top), c - s, 0.9,
                                   fc=STATE_COL.get(states[s], "#e2e8f0"),
                                   ec="#4a5568", lw=0.4))
            if c - s >= 6:
                ax.text((s + c) / 2, top + 0.45, states[s], ha="center", va="center",
                        fontsize=8, fontweight="bold")
            s = c
    base["STATE"] = top

    # EN_x -> PG_x handshake arrows
    for rail, d, cc, ce in pairs:
        if f"EN_{rail}" not in base:
            continue
        lvl = 0.8 if d else 0.0
        yc = base[f"EN_{rail}"] + lvl
        if ce is None:
            ax.plot([cc], [yc], "x", color="#c53030", ms=9)
            ax.text(cc, yc + 0.15, "no PG response", color="#c53030", fontsize=8)
        else:
            ye = base[f"PG_{rail}"] + lvl
            ax.annotate("", xy=(ce, ye), xytext=(cc, yc),
                        arrowprops=dict(arrowstyle="->", color="#c53030", lw=1.2,
                                        connectionstyle="arc3,rad=0.2"))
            ax.text((cc + ce) / 2, (yc + ye) / 2 + 0.3, f"+{ce - cc} cyc",
                    color="#c53030", fontsize=8, ha="center")

    yticks = [base[l] + 0.4 for _, l, _ in lanes] + [top + 0.45]
    ylabels = [f"[{g}] {l}" for g, l, _ in lanes] + [state_label]
    ax.set_yticks(yticks)
    ax.set_yticklabels(ylabels, fontsize=9)
    ax.set_xlim(-1, n)
    ax.set_ylim(-0.5, top + 1.3)
    ax.set_xlabel("clock cycle (rising CLK edges)")
    ax.set_title(title, fontsize=11)
    ax.grid(axis="x", color="#edf2f7", lw=0.6)
    if overlay:
        overlay(ax, top)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    return fig


# ----------------------------------------------------------------------------
# Block diagram snapshot (also used for the GIF)
# ----------------------------------------------------------------------------
COL_REQ, COL_SENS, COL_ON, COL_OFF = "#2b6cb0", "#c53030", "#2f855a", "#a0aec0"


def box(ax, x, y, w, h, title, fc):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec="#4a5568", lw=1.2))
    ax.text(x + w / 2, y + h - 0.2, title, ha="center", va="top", fontsize=9, fontweight="bold")


def wire(ax, x0, x1, y, label, on, color, label_dy=0.12, va="bottom"):
    c = color if on else COL_OFF
    ax.annotate("", xy=(x1, y), xytext=(x0, y),
                arrowprops=dict(arrowstyle="-|>", color=c, lw=2.2 if on else 0.9))
    ax.text((x0 + x1) / 2, y + label_dy, label, ha="center", va=va, fontsize=7.5, color=c)


SOC_DIM, SOC_BRIGHT = "#ebf8ff", "#63b3ed"
DVFS_NAMES = {0: "LO", 1: "MID", 2: "HI", 3: "V_UP", 4: "F_UP",
              5: "F_DN", 6: "V_DN", 7: "FAULT"}
DVFS_COL = {"LO": "#c6f6d5", "MID": "#c6f6d5", "HI": "#c6f6d5",
            "V_UP": "#feebc8", "F_UP": "#feebc8", "F_DN": "#feebc8",
            "V_DN": "#feebc8", "FAULT": "#fed7d7"}
DV_ST_NAMES = ("DV_ST3", "DV_ST2", "DV_ST1", "DV_ST0")
DV_V_NAMES = ("DV_V1", "DV_V0")
DV_K_NAMES = ("DV_K1", "DV_K0")
DV_CUR_NAMES = ("DV_CUR1", "DV_CUR0")   # actual divider, div_cur from clk_div
DIV_OF = {3: 2, 1: 4, 0: 8}              # divide code -> clk/N (clk_div.v)


def mix(c1, c2, t):
    """Blend two #rrggbb colours; t=0 gives c1, t=1 gives c2."""
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * t):02x}" for x, y in zip(a, b))


def find_dvfs_scope(vcd, top):
    """Relative path of the dvfs_ctrl instance: the scope that holds v_code,
    clk_div_code and state. Returns None if it isn't in the dump."""
    for full in vcd.names:
        if full.endswith(".v_code"):
            prefix = full[:-len(".v_code")]
            if f"{prefix}.clk_div_code" in vcd.names and f"{prefix}.state" in vcd.names:
                if prefix == top:
                    return ""
                if prefix.startswith(top + "."):
                    return prefix[len(top) + 1:]
    return None


def dvfs_pins(rel):
    """Pins for the dvfs_ctrl instance at scope `rel` (relative to the top)."""
    def p(name):
        return f"{rel}.{name}" if rel else name
    return [
        Pin("DV_ST3", p("state"), 3), Pin("DV_ST2", p("state"), 2),
        Pin("DV_ST1", p("state"), 1), Pin("DV_ST0", p("state"), 0),
        Pin("DV_V1", p("v_code"), 1), Pin("DV_V0", p("v_code"), 0),
        Pin("DV_K1", p("clk_div_code"), 1), Pin("DV_K0", p("clk_div_code"), 0),
        Pin("DV_BUSY", p("dvfs_busy")),
    ]


def find_clkdiv_cur(vcd, top, rel):
    """Relative path of div_cur for the clock divider beside dvfs_ctrl."""
    parent = rel.rsplit(".", 1)[0] if "." in rel else ""
    for full in vcd.names:
        if full.endswith(".div_cur") and full.startswith(top + "."):
            rel2 = full[len(top) + 1:]
            if parent == "" or rel2.startswith(parent + "."):
                return rel2
    return None


def clock_speed(rows):
    """Clock speed from the divider's actual code (div_cur): clk/2, clk/4 or clk/8.
    Brightness is 1.0 at clk/2 (the fastest) and scales down from there."""
    if "DV_CUR1" in rows[0]:
        names, src_name = DV_CUR_NAMES, "actual divider (div_cur)"
    elif "DV_K1" in rows[0]:
        names, src_name = DV_K_NAMES, "requested divider (clk_div_code)"
    else:
        return None, "no dvfs_ctrl in dump"
    speed = [2.0 / DIV_OF.get(lane_value(r, names), 8) for r in rows]
    return speed, src_name


def draw_block(ax, row, cycle, last_ev, speed=None):
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8.2)
    ax.axis("off")
    st = row["STATE"]
    has_dvfs = "DV_ST3" in row
    s = 0.0 if speed is None else min(max(speed, 0.0), 1.0)
    ax.text(0.2, 8.0, "blue: SoC requests   red: sensors asserted   green: rail/PG high   grey: idle",
            fontsize=8, color="#4a5568")

    # External SoC: brightness follows the DVFS clock speed (a halo plus a lighter fill)
    if speed is not None:
        ax.add_patch(FancyBboxPatch((0.08, 4.48), 2.84, 3.44,
                                    boxstyle="round,pad=0.02,rounding_size=0.2",
                                    fc=SOC_BRIGHT, ec="none", alpha=0.6 * s, zorder=0))
    box(ax, 0.2, 4.6, 2.6, 3.2, "External SoC", mix(SOC_DIM, SOC_BRIGHT, s))
    if speed is not None:
        names = DV_CUR_NAMES if "DV_CUR1" in row else DV_K_NAMES
        div = DIV_OF.get(lane_value(row, names), 8)
        ax.text(0.4, 5.2, f"clock clk/{div}  ({s * 100:.0f}% of fastest)", fontsize=8)
        ax.add_patch(Rectangle((0.4, 4.8), 2.2, 0.22, fc="white", ec="#4a5568", lw=0.6))
        ax.add_patch(Rectangle((0.4, 4.8), 2.2 * s, 0.22, fc=SOC_BRIGHT, ec="none"))
    box(ax, 0.2, 0.7, 2.6, 2.9, "Sensors", "#fefcbf")
    box(ax, 4.2, 0.7, 1.8, 6.9, "SoC PMC ASIC", "#f0fff4")

    box(ax, 7.6, 0.7, 2.3, 6.9, "Regulators", "#faf5ff")

    if "PWR_REQ" in row:
        v = row["PWR_REQ"]
        wire(ax, 2.8, 4.2, 7.0, f"PWR_REQ={v}", v, COL_REQ)
    if "PERF_REQ1" in row:
        v = lane_value(row, ("PERF_REQ1", "PERF_REQ0"))
        wire(ax, 2.8, 4.2, 6.3, f"PERF_REQ={v:02b}", v, COL_REQ)
    if "FAULT_CLR" in row:
        v = row["FAULT_CLR"]
        wire(ax, 2.8, 4.2, 5.6, f"FAULT_CLR={v}", v, COL_REQ)

    for label, y in (("WARN", 2.9), ("CRIT", 2.2), ("OC", 1.5)):
        if label in row:
            v = row[label]
            wire(ax, 2.8, 4.2, y, f"{label}={v}", v, COL_SENS)

    ax.text(5.1, 6.6, st, ha="center", fontsize=12, fontweight="bold",
            bbox=dict(boxstyle="round", fc=STATE_COL.get(st, "white"), ec="none"))
    y = 5.8
    for name in ("DVFS_EN", "THROTTLED", "FAULT"):
        if name in row:
            ax.text(4.35, y, f"{name} = {row[name]}", fontsize=8)
            y -= 0.45
    if "VID1" in row:
        ax.text(4.35, y, f"VID = {lane_value(row, ('VID1', 'VID0')):02b}", fontsize=8)

    # dvfs_ctrl panel, inside the ASIC box
    dvfs_state = None
    if has_dvfs:
        dvfs_state = DVFS_NAMES.get(lane_value(row, DV_ST_NAMES), "?")
        ax.add_patch(FancyBboxPatch((4.3, 0.95), 1.6, 2.5,
                                    boxstyle="round,pad=0.02,rounding_size=0.08",
                                    fc="#faf5ff", ec="#6b46c1", lw=1.0, zorder=2))
        ax.text(5.1, 3.3, "dvfs_ctrl", ha="center", va="top", fontsize=8,
                fontweight="bold", color="#6b46c1", zorder=3)
        ax.text(5.1, 2.8, dvfs_state, ha="center", va="center", fontsize=11,
                fontweight="bold", zorder=3,
                bbox=dict(boxstyle="round", fc=DVFS_COL.get(dvfs_state, "white"), ec="none"))
        v = lane_value(row, DV_V_NAMES)
        k = lane_value(row, DV_K_NAMES)
        b = row.get("DV_BUSY", 0)
        lines = [(f"V_CODE   {v:02b}", 2.3), (f"CLK_REQ  {k:02b}", 1.9)]
        if "DV_CUR1" in row:
            lines.append((f"CLK_CUR  {lane_value(row, DV_CUR_NAMES):02b}", 1.5))
        lines.append((f"busy     {b}", 1.1))
        for text, yy in lines:
            ax.text(4.4, yy, text, fontsize=8, family="monospace", zorder=3)

    # VID out to the regulators, VREG_ACK back
    for rail, c in (("A", 6.1), ("B", 4.6), ("C", 3.1)):
        if f"EN_{rail}" not in row:
            continue
        en, pg = row[f"EN_{rail}"], row.get(f"PG_{rail}", 0)
        wire(ax, 6.0, 8.0, c + 0.2, f"EN_{rail}={en}", en, COL_ON)
        wire(ax, 8.0, 6.0, c - 0.2, f"PG_{rail}={pg}", pg, COL_ON, label_dy=-0.1, va="top")
        ax.add_patch(FancyBboxPatch((8.0, c - 0.45), 1.8, 0.9,
                                    boxstyle="round,pad=0.02,rounding_size=0.08",
                                    fc="#c6f6d5" if pg else "#edf2f7", ec="#4a5568", lw=0.8))
        ax.text(8.9, c, f"Rail {rail}: {'ON' if pg else 'off'}", ha="center", va="center", fontsize=8)

    if "VID1" in row:
        v = lane_value(row, ("VID1", "VID0"))
        wire(ax, 6.0, 7.6, 2.0, f"VID={v:02b}", v != 0, COL_REQ)
    if "VREG_ACK" in row:
        v = row["VREG_ACK"]
        wire(ax, 7.6, 6.0, 1.3, f"VREG_ACK={v}", v, COL_ON)

    foot = f"cycle {cycle}   t = {row['_t']} ns   state = {st}"
    if dvfs_state:
        foot += f"   dvfs = {dvfs_state}"
    ax.text(0.2, 0.2, foot + f"   last event: {last_ev}", fontsize=9, family="monospace")


# ----------------------------------------------------------------------------
# Per-case GIFs
# ----------------------------------------------------------------------------
def load_cases(path):
    """Read the case marker file: one 'time_ns name' line per case."""
    cases = []
    with open(path, errors="replace") as f:
        for n, line in enumerate(f, 1):
            raw = line.rstrip("\r\n")
            parts = raw.strip().split(" ", 1)
            if len(parts) == 2 and parts[0].isdigit():
                cases.append((int(parts[0]), parts[1].strip().strip("\x00").strip()))
            else:
                print(f"  case file line {n} not understood: {raw!r}")
    print(f"  case file has {len(cases)} usable line(s):")
    for t, name in cases:
        print(f"    {t:>10} ns  {name}")
    return cases


def case_slices(rows, cases, time_unit_ns):
    """(name, first_row, end_row) per case. A case runs until the next one starts."""
    if time_unit_ns is None or time_unit_ns <= 0:
        raise ValueError("VCD must declare a supported $timescale to match case times in ns")
    times = [r["_t"] for r in rows]
    case_ticks = [(round(t / time_unit_ns), name) for t, name in cases]
    out = []
    for i, (t, name) in enumerate(case_ticks):
        s = bisect.bisect_left(times, t)
        e = (bisect.bisect_left(times, case_ticks[i + 1][0])
             if i + 1 < len(case_ticks) else len(rows))
        if e > s:
            out.append((name, s, e))
    return out


def activity_frames(rows, s, e, idle_stride=25):
    """Rows to draw: every cycle where a signal changes, plus one row per
    idle_stride cycles while nothing moves. Ramps stay frame-by-frame."""
    keys = [k for k in rows[s] if not k.startswith("_") and k not in ("STATE", "DV")]
    frames, prev, last = [], None, None
    for c in range(s, e):
        sig = tuple(rows[c].get(k, 0) for k in keys)
        changed = prev is None or sig != prev
        prev = sig
        if changed or last is None or c - last >= idle_stride:
            frames.append(c)
            last = c
    if frames[-1] != e - 1:
        frames.append(e - 1)
    return frames


def write_case_gif(rows, speed, name, s, e, path, last_event, interval_ms=500):
    frames = activity_frames(rows, s, e)
    fig, ax = plt.subplots(figsize=(11, 8.4))
    fig.suptitle(name, fontsize=12, fontweight="bold")

    def update(i):
        c = frames[i]
        sp = speed[c] if speed else None
        draw_block(ax, rows[c], c, last_event(c), sp)
        gap = frames[i + 1] - c if i + 1 < len(frames) else 0
        ax.text(9.9, 0.2, f"frame {i + 1}/{len(frames)}   next: +{gap} cycles",
                ha="right", fontsize=8, color="#4a5568")

    anim = FuncAnimation(fig, update, frames=len(frames), interval=interval_ms)
    anim.save(path, writer="pillow", dpi=100)
    plt.close(fig)
    return len(frames)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Visualise a testbench VCD")
    ap.add_argument("vcd", help="VCD file dumped by the testbench")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="tt",
                    help="tt = tb_tt_smoke pins, fsm = tb_power_fsm names, "
                         "dvfs = tb_dvfs_ctrl ports")
    ap.add_argument("--clk", default="clk", help="clock signal under the top scope")
    ap.add_argument("--at", type=int, help="cycle for the block-diagram snapshot "
                                           "(default: first RUN cycle)")
    ap.add_argument("--gif", help="also write an animated block diagram to this file")
    ap.add_argument("--out", default="soc_view", help="folder for the PNG output")
    ap.add_argument("--cases", help="case marker file from the testbench "
                                    "(default: <vcd name>_cases.txt next to the VCD)")
    args = ap.parse_args()

    vcd = Vcd(args.vcd)
    top = vcd.top
    pins = [p for p in PRESETS[args.preset] if f"{top}.{p.var}" in vcd.names]
    missing = [p.name for p in PRESETS[args.preset] if p not in pins]
    if missing:
        print("note: not in this VCD, skipped:", ", ".join(missing))

    # the dvfs_ctrl inside the SoC, for the top-level block view
    dv_pins = []
    if args.preset == "tt":
        rel = find_dvfs_scope(vcd, top)
        if rel is None:
            print("dvfs_ctrl not found in this VCD; DVFS panel left out")
        else:
            dv_pins = list(dvfs_pins(rel))
            cur_rel = find_clkdiv_cur(vcd, top, rel)
            if cur_rel:
                dv_pins += [Pin("DV_CUR1", cur_rel, 1), Pin("DV_CUR0", cur_rel, 0)]
            else:
                print("clock divider div_cur not found; using requested code")
            dv_pins = [p for p in dv_pins if f"{top}.{p.var}" in vcd.names]
            print(f"dvfs_ctrl found at {top}.{rel or '(top)'}")
            pins = pins + dv_pins

    state_fn = dvfs_state if args.preset == "dvfs" else infer_state
    rows = sample_rows(vcd, top, pins, args.clk, state_fn)
    if not rows:
        sys.exit(f"no rising edges on {top}.{args.clk}; check --clk")
    if dv_pins:
        for r in rows:
            r["DV"] = DVFS_NAMES.get(lane_value(r, DV_ST_NAMES), "?")

    present = {p.name for p in pins}
    lane_set = DVFS_LANES if args.preset == "dvfs" else LANES
    lanes = [ln for ln in lane_set if all(n in present for n in ln[2])]
    events = event_log(rows, sorted(n for n in present if not n.startswith("DV_")))
    name = os.path.basename(args.vcd)
    os.makedirs(args.out, exist_ok=True)

    if args.preset == "dvfs":
        if "BUSY" not in present:
            sys.exit("dvfs_busy not found in this VCD")
        results, stray = check_dvfs(rows, dvfs_windows(rows))
        print_dvfs_report(rows, events, results, stray)
        path = os.path.join(args.out, "dvfs_timeline.png")
        plot_timeline(rows, lanes, [], f"{name}: DVFS ports, transitions checked", path,
                      state_label="[DVFS] state",
                      overlay=lambda ax, top_y: draw_windows(ax, top_y, results))
        print(f"\nwrote {path}")
        plt.show()
        return

    pairs = rail_handshakes(rows)
    ev_cycles = [c for c, _ in events]

    def last_event(c):
        i = bisect.bisect_right(ev_cycles, c) - 1
        return events[i][1] if i >= 0 else "reset"

    print_report(rows, events, pairs)

    plot_timeline(rows, lanes, pairs, f"{name}: ASIC handshake with SoC, sensors and rails",
                  os.path.join(args.out, "timeline.png"))

    speed, src = clock_speed(rows)
    if speed is not None:
        print(f"clock speed for the block view: {src}")

    cyc = args.at
    if cyc is None:
        cyc = next((c for c, r in enumerate(rows) if r["STATE"] == "RUN"), 0)
    cyc = min(max(cyc, 0), len(rows) - 1)
    sp = speed[cyc] if speed else None
    fig, ax = plt.subplots(figsize=(11, 8.2))
    draw_block(ax, rows[cyc], cyc, last_event(cyc), sp)
    fig.savefig(os.path.join(args.out, f"block_cycle_{cyc}.png"), dpi=150)

    cases_path = args.cases or os.path.splitext(args.vcd)[0] + "_cases.txt"
    if os.path.exists(cases_path):
        slices = case_slices(rows, load_cases(cases_path), vcd.time_unit_ns)
        print(f"{len(slices)} cases from {cases_path}")
        for i, (cname, s, e) in enumerate(slices, 1):
            slug = re.sub(r"[^a-z0-9]+", "_", cname.lower()).strip("_")
            gif = os.path.join(args.out, f"case_{i:02d}_{slug}.gif")
            nf = write_case_gif(rows, speed, cname, s, e, gif, last_event)
            print(f"wrote {gif}  ({e - s} cycles, {nf} frames)")
            plot_timeline(rows[s:e], lanes, [], f"{name}: {cname}",
                          os.path.join(args.out, f"case_{i:02d}_{slug}_timeline.png"))
    else:
        print(f"WARNING: no case file at {cases_path}; the per-case GIFs were not made. "
              "Re-run the testbench and check that tb_soc_full_cases.txt sits next to the VCD.")
    if args.gif and not os.path.exists(cases_path):
        frames = list(range(0, len(rows), max(1, len(rows) // 300)))
        anim = FuncAnimation(
            fig,
            lambda i: draw_block(ax, rows[frames[i]], frames[i], last_event(frames[i]),
                                 speed[frames[i]] if speed else None),
            frames=len(frames), interval=120)
        anim.save(args.gif, writer="pillow")
        print(f"wrote {args.gif}")

    print(f"wrote PNGs to {args.out}/")
    plt.show()


if __name__ == "__main__":
    main()