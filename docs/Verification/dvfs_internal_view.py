#!/usr/bin/env python3
"""
dvfs_internal_view.py

Internal-state view of dvfs_ctrl, from a tb_dvfs_ctrl VCD. The VCD must include
the dut.* internals, which $dumpvars(0, tb_dvfs_ctrl) does.

  python dvfs_internal_view.py tb_dvfs_ctrl.vcd
  python dvfs_internal_view.py tb_dvfs_ctrl.vcd --gif dvfs.gif   # animated FSM

Works with the updated dvfs_ctrl.v (ramp targets and voltage timeouts).

Writes to soc_view/ (or --out):
  dvfs_timeline.png      ports + FSM state band + internal signals
  dvfs_fsm_coverage.png  state diagram: edges taken in this run, with counts

Printed report:
  state visits with dwell time and what ended each visit (clock ack, timer, vreg_ack...)
  safety/ordering checks each cycle
  coverage of every FSM edge in dvfs_ctrl.v

Needs vcd_system_view.py in the same folder.
"""

import argparse
import os
import sys
from collections import Counter

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch

from vcd_system_view import GROUP_COL, STATE_COL, Pin, Vcd, lane_value, plot_timeline, sample_rows

# ----------------------------------------------------------------------------
# Pins: ports from tb_dvfs_ctrl plus internals of dvfs_ctrl (dut.*)
# ----------------------------------------------------------------------------
DVFS_INT_PINS = [
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
    # dut.state is a 4-bit reg; each bit is its own pin
    Pin("ST3", "dut.state", 3), Pin("ST2", "dut.state", 2),
    Pin("ST1", "dut.state", 1), Pin("ST0", "dut.state", 0),
    Pin("SHUT",     "dut.shutting_down"),
    Pin("FFLAG",    "dut.fault_flag"),
    Pin("TMR_LOAD", "dut.tmr_load"),
    Pin("TMR_EXP",  "dut.tmr_expired"),
    Pin("UP1", "dut.up_tgt", 1), Pin("UP0", "dut.up_tgt", 0),
    Pin("DN1", "dut.dn_tgt", 1), Pin("DN0", "dut.dn_tgt", 0),
]

# Encoding from the localparams in dvfs_ctrl.v
FSM_NAMES = {0: "LO", 1: "MID", 2: "HI", 3: "V_UP", 4: "F_UP",
             5: "F_DN", 6: "V_DN", 7: "FAULT"}
ST_NAMES = ("ST3", "ST2", "ST1", "ST0")
STEADY, RAMP = {"LO", "MID", "HI"}, {"V_UP", "F_UP", "F_DN", "V_DN"}
V_NAMES, K_NAMES = ("V1", "V0"), ("CLKDIV1", "CLKDIV0")
ACK_NAMES = ("CLK_HI_ACK1", "CLK_HI_ACK0")
UP_NAMES = ("UP1", "UP0")      # level an up ramp is aiming for
DN_NAMES = ("DN1", "DN0")      # level a down ramp is aiming for
REQ_NAMES = ("DVFS_REQ1", "DVFS_REQ0")

# Every transition in the case statement of dvfs_ctrl.v.
# reachable=False marks ones the RTL can never take.
EDGES = [
    ("LO",    "V_UP",  "req!=00 & !throttled",            True),
    ("LO",    "FAULT", "fsm_fault | fault_flag",          True),
    ("MID",   "V_UP",  "req==11",                         True),
    ("MID",   "F_DN",  "shutting_down",                   True),
    ("MID",   "FAULT", "fsm_fault | fault_flag",          True),
    ("HI",    "F_DN",  "shutting_down | req!=11",         True),
    ("HI",    "FAULT", "fsm_fault | fault_flag",          True),
    ("V_UP",  "F_UP",  "vreg_ack",                        True),
    ("V_UP",  "F_DN",  "shutting_down (or vreg timeout)", True),
    ("F_UP",  "HI",    "clk_ack==up_tgt, up_tgt==11",     True),
    ("F_UP",  "MID",   "clk_ack==up_tgt, up_tgt==01",     True),
    ("F_UP",  "F_DN",  "shutting_down",                   True),
    ("F_DN",  "V_DN",  "clk_ack==dn_tgt | timer",         True),
    ("V_DN",  "LO",    "vreg_ack, dn_tgt==00, !fault",    True),
    ("V_DN",  "MID",   "vreg_ack, dn_tgt==01, !fault",    True),
    ("V_DN",  "FAULT", "vreg_ack&fault | vreg timeout",   True),
    ("FAULT", "LO",    "!fsm_fault",                      True),
]
EDGE_KEYS = {(s, d) for s, d, _, _ in EDGES}

# Layout for the state diagram (x, y), roughly following the ramp paths
NODE_POS = {
    "LO": (1.0, 5.6), "MID": (5.0, 5.6), "HI": (9.0, 5.6),
    "V_UP": (2.4, 3.6), "F_UP": (5.9, 3.6),
    "V_DN": (1.0, 1.4), "F_DN": (9.0, 1.4),
    "FAULT": (5.0, 0.0),
}

INT_LANES = [
    ("Request",  "RST_N",           ("RST_N",)),
    ("Request",  "DVFS_REQ[1:0]",   ("DVFS_REQ1", "DVFS_REQ0")),
    ("Request",  "THROTTLED",       ("THROTTLED",)),
    ("Request",  "FSM_FAULT",       ("FSM_FAULT",)),
    ("Ack",      "VREG_ACK",        ("VREG_ACK",)),
    ("Ack",      "CLK_HI_ACK[1:0]", ("CLK_HI_ACK1", "CLK_HI_ACK0")),
    ("Output",   "V_CODE[1:0]",     ("V1", "V0")),
    ("Output",   "CLK_DIV[1:0]",    ("CLKDIV1", "CLKDIV0")),
    ("Output",   "BUSY",            ("BUSY",)),
    ("Output",   "DVFS_FAULT",      ("DVFS_FAULT",)),
    ("Internal", "shutting_down",   ("SHUT",)),
    ("Internal", "fault_flag",      ("FFLAG",)),
    ("Internal", "tmr_load",        ("TMR_LOAD",)),
    ("Internal", "tmr_expired",     ("TMR_EXP",)),
    ("Internal", "up_tgt[1:0]",     ("UP1", "UP0")),
    ("Internal", "dn_tgt[1:0]",     ("DN1", "DN0")),
]

# Colours for the state band, and the extra group used by the timeline
STATE_COL.update({s: "#c6f6d5" for s in STEADY})
STATE_COL.update({s: "#feebc8" for s in RAMP})
STATE_COL["FAULT"] = "#fed7d7"
GROUP_COL.update({"Request": "#2b6cb0", "Ack": "#6b46c1",
                 "Output": "#2f855a", "Internal": "#718096"})


# ----------------------------------------------------------------------------
# Derived internal state
# ----------------------------------------------------------------------------
def fsm_state(row):
    return FSM_NAMES.get(lane_value(row, ST_NAMES), "?")


def req_code(row):
    v = lane_value(row, REQ_NAMES)
    return 0 if v == 2 else v          # 2'b10 is mapped to 00 in the RTL


def visits_of(rows):
    """(state, first_row, last_row) for each run of the same state."""
    out, s = [], 0
    for c in range(1, len(rows) + 1):
        if c == len(rows) or rows[c]["STATE"] != rows[s]["STATE"]:
            out.append((rows[s]["STATE"], s, c - 1))
            s = c
    return out


def exit_cause(rows, name, end):
    """What took the FSM out of `name`, judged from the last row of the visit.
    Tests follow the priority order used in dvfs_ctrl.v."""
    if end + 1 >= len(rows):
        return "end of run"
    r = rows[end]
    ack = lane_value(r, ACK_NAMES)
    up_tgt = lane_value(r, UP_NAMES)
    dn_tgt = lane_value(r, DN_NAMES)
    if name == "F_DN":                  # clk_ack==dn_tgt is tested before the timer
        if ack == dn_tgt:
            return "clk ack (ramp down)"
        if r["TMR_EXP"]:
            return "timer (T_SETTLE)"
        return "?"
    if name == "F_UP":                  # shutting_down is tested first
        if r["SHUT"]:
            return "shutting_down"
        if ack == up_tgt:
            return "clk ack (ramp up)"
        if r["TMR_EXP"]:
            return "clk timeout -> fault"
        return "?"
    if name == "V_UP":
        if r["VREG_ACK"]:
            return "vreg_ack"
        if r["SHUT"]:
            # no ack and the timer ran out: this is the voltage timeout
            return "voltage timeout -> shutting" if r["TMR_EXP"] else "shutting_down"
        return "?"
    if name == "V_DN":
        if r["VREG_ACK"]:
            return "vreg_ack"
        if r["TMR_EXP"]:
            return "voltage timeout -> FAULT"
        return "?"
    if name == "FAULT":
        return "fsm_fault cleared"
    return f"req/fault -> {rows[end + 1]['STATE']}"


def check_invariants(rows):
    """Per-cycle checks on the outputs, taken from the RTL's intent."""
    viol = []
    for c in range(1, len(rows)):
        r, p = rows[c], rows[c - 1]
        if not r["RST_N"]:
            continue
        v, k = lane_value(r, V_NAMES), lane_value(r, K_NAMES)
        pv, pk = lane_value(p, V_NAMES), lane_value(p, K_NAMES)
        st = r["STATE"]
        if k > v:
            viol.append((c, f"clk_div {k:02b} is above v_code {v:02b} in {st}"))
        if st in ("LO", "FAULT") and (v or k):
            viol.append((c, f"outputs not 00 in {st}"))
        if k > pk and st not in ("F_UP", "MID", "HI"):
            viol.append((c, f"clk_div rose in {st} (voltage must come up first)"))
        if v < pv and st not in ("V_UP", "V_DN", "LO", "FAULT"):
            viol.append((c, f"v_code fell in {st} (clock must come down first)"))
    return viol


def coverage(visits):
    seen = Counter((a, b) for (a, _, _), (b, _, _) in zip(visits, visits[1:]))
    table = [(s, d, cond, reach, seen.get((s, d), 0)) for s, d, cond, reach in EDGES]
    unexpected = [(e, n) for e, n in seen.items() if e not in EDGE_KEYS]
    return table, unexpected


# ----------------------------------------------------------------------------
# Report
# ----------------------------------------------------------------------------
def print_report(rows, visits, viol, table, unexpected):
    print("\n== state visits ==")
    print(f"{'cycles':<12}{'ns':>8}  {'state':<7}{'dwell':>6}  exit")
    for name, a, b in visits:
        cause = exit_cause(rows, name, b)
        print(f"{a:>4}-{b:<6}{rows[a]['_t']:>8}  {name:<7}{b - a + 1:>6}  {cause}")

    print("\n== ordering / safety checks (every cycle) ==")
    if not viol:
        print("none")
    for c, msg in viol:
        print(f"cyc {c:>4} ({rows[c]['_t']} ns): {msg}")

    print("\n== FSM edge coverage (from dvfs_ctrl.v) ==")
    for s, d, cond, reach, n in table:
        if not reach:
            flag = "unreachable in RTL"
        elif n == 0:
            flag = "NOT EXERCISED"
        else:
            flag = ""
        print(f"{s:>5} -> {d:<6} x{n:<4} {cond:<30} {flag}")
    for (s, d), n in unexpected:
        print(f"UNEXPECTED transition {s} -> {d} x{n} (not in dvfs_ctrl.v)")

    print("\n== mechanism summary ==")
    by = Counter((name, exit_cause(rows, name, b)) for name, a, b in visits
                 if name in RAMP or name == "FAULT")
    for (name, cause), n in sorted(by.items()):
        print(f"{name:<7} x{n:<3} exit: {cause}")

    nbad = len(viol) + len(unexpected)
    print(f"\n{'PASS' if nbad == 0 else 'FAIL'}: {len(viol)} ordering/safety violation(s), "
          f"{len(unexpected)} unexpected transition(s)")


# ----------------------------------------------------------------------------
# State diagram
# ----------------------------------------------------------------------------
HALF = (0.85, 0.36)   # half-size of a state box in data units


def clip(c, other):
    """Point on the edge of box c, on the line from c towards other."""
    dx, dy = other[0] - c[0], other[1] - c[1]
    t = min(HALF[0] / abs(dx) if dx else float("inf"),
            HALF[1] / abs(dy) if dy else float("inf"))
    return (c[0] + dx * t, c[1] + dy * t)


def draw_fsm(ax, counts, active=None, last_edge=None, info=""):
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(-0.9, 6.8)
    ax.axis("off")

    for src, dst, cond, reach in EDGES:
        n = counts.get((src, dst), 0)
        a, b = NODE_POS[src], NODE_POS[dst]
        if (src, dst) == last_edge:
            col, lw, ls = "#dd6b20", 3.0, "-"
        elif n > 0:
            col, lw, ls = "#2b6cb0", 1.6, "-"
        elif not reach:
            col, lw, ls = "#e53e3e", 1.2, ":"
        else:
            col, lw, ls = "#cbd5e0", 1.0, "--"
        p0, p1 = clip(a, b), clip(b, a)
        ax.annotate("", xy=p1, xytext=p0,
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=lw, ls=ls,
                                    connectionstyle="arc3,rad=0.12"))
        if n > 0:
            mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
            ax.text(mx, my, f"x{n}", fontsize=8, color=col, ha="center", va="center",
                    bbox=dict(fc="white", ec="none", pad=0.6), zorder=5)

    for name, (x, y) in NODE_POS.items():
        if name in STEADY:
            fc, dark = "#c6f6d5", "#38a169"
        elif name in RAMP:
            fc, dark = "#feebc8", "#d69e2e"
        else:
            fc, dark = "#fed7d7", "#e53e3e"
        lw, weight = 1.0, "normal"
        if name == active:
            fc, lw, weight = dark, 2.6, "bold"
        ax.add_patch(FancyBboxPatch((x - HALF[0], y - HALF[1]), 2 * HALF[0], 2 * HALF[1],
                                    boxstyle="round,pad=0.02,rounding_size=0.1",
                                    fc=fc, ec="#2d3748", lw=lw, zorder=3))
        ax.text(x, y, name, ha="center", va="center", fontsize=10, fontweight=weight, zorder=4)

    ax.text(0.1, -0.75,
            "solid blue = taken (x = count)   dashed grey = never taken   "
            "dotted red = unreachable in RTL" if not info else info,
            fontsize=8.5, family="monospace")


def frame_states(rows):
    """Per row: cumulative edge counts and the most recent edge."""
    counts, last, out = Counter(), None, []
    for r in range(len(rows)):
        if r > 0 and rows[r]["STATE"] != rows[r - 1]["STATE"]:
            e = (rows[r - 1]["STATE"], rows[r]["STATE"])
            counts[e] += 1
            last = e
        out.append((dict(counts), last))
    return out


def info_line(row, c):
    req = req_code(row)
    ack = lane_value(row, ACK_NAMES)
    v, k = lane_value(row, V_NAMES), lane_value(row, K_NAMES)
    return (f"cyc {c:4d} {row['_t']:>6}ns  req={req:02b} thr={row['THROTTLED']} "
            f"flt={row['FSM_FAULT']}  vreg_ack={row['VREG_ACK']} clk_ack={ack:02b} "
            f"tmr_exp={row['TMR_EXP']} shut={row['SHUT']}  V={v:02b} CLK={k:02b}")


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Internal-state view of dvfs_ctrl")
    ap.add_argument("vcd", help="VCD from tb_dvfs_ctrl with dut.* internals")
    ap.add_argument("--clk", default="clk")
    ap.add_argument("--scope", help="path of the dvfs_ctrl instance, e.g. dut.u_dvfs "
                                    "(for tb_soc_full dumps)")
    ap.add_argument("--out", default="soc_view")
    ap.add_argument("--gif", help="also write an animated FSM to this file")
    args = ap.parse_args()

    vcd = Vcd(args.vcd)
    top = vcd.top
    base = list(DVFS_INT_PINS)
    if args.scope:
        # full-system dumps: the dvfs instance sits at args.scope, e.g. dut.u_dvfs
        base = [Pin(p.name, args.scope + "." + (p.var[4:] if p.var.startswith("dut.") else p.var),
                    p.bit, p.active_low) for p in base]
    pins = [p for p in base if f"{top}.{p.var}" in vcd.names]
    present = {p.name for p in pins}
    if "ST3" not in present:
        sys.exit("dut.state not found: dump with $dumpvars(0, tb_dvfs_ctrl)")
    missing = [p.name for p in DVFS_INT_PINS if p.name not in present]
    if missing:
        print("note: not in this VCD, skipped:", ", ".join(missing))

    rows = sample_rows(vcd, top, pins, args.clk, fsm_state)
    if not rows:
        sys.exit(f"no rising edges on {top}.{args.clk}")

    visits = visits_of(rows)
    viol = check_invariants(rows)
    table, unexpected = coverage(visits)
    print_report(rows, visits, viol, table, unexpected)

    os.makedirs(args.out, exist_ok=True)
    name = os.path.basename(args.vcd)
    lanes = [ln for ln in INT_LANES if all(n in present for n in ln[2])]
    tl = os.path.join(args.out, "dvfs_timeline.png")
    plot_timeline(rows, lanes, [], f"{name}: dvfs_ctrl internal state and ports", tl,
                  state_label="[FSM] state")
    print(f"\nwrote {tl}")

    final_counts = frame_states(rows)[-1][0]
    fig, ax = plt.subplots(figsize=(11, 7.6))
    draw_fsm(ax, final_counts)
    ax.set_title(f"{name}: dvfs_ctrl state diagram, edges taken in this run", fontsize=11)
    cov = os.path.join(args.out, "dvfs_fsm_coverage.png")
    fig.savefig(cov, dpi=150)
    print(f"wrote {cov}")

    if args.gif:
        states = frame_states(rows)
        step = max(1, len(rows) // 300)
        idx = list(range(0, len(rows), step))
        fig2, ax2 = plt.subplots(figsize=(11, 7.6))

        def update(i):
            counts, last = states[idx[i]]
            r = idx[i]
            draw_fsm(ax2, counts, active=rows[r]["STATE"], last_edge=last,
                     info=info_line(rows[r], r))

        anim = FuncAnimation(fig2, update, frames=len(idx), interval=110)
        anim.save(args.gif, writer="pillow")
        print(f"wrote {args.gif}")

    plt.show()


if __name__ == "__main__":
    main()