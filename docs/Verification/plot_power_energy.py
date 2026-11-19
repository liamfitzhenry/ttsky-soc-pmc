#!/usr/bin/env python3
"""
plot_power_energy.py

Plots power and energy over time for tb_power_long.vcd. Standalone: it does not
import vcd_system_view.py.

  python plot_power_energy.py tb_power_long.vcd
  python plot_power_energy.py tb_power_long.vcd --out power_energy.png

Background colour is the operating level, read from the VID pins in the waveform:
  red    high      (VID 11)
  green  normal    (VID 01)
  blue   low       (VID 00)
  grey   not in RUN (power is 0)

Time is device time from the VCD timestamps. The testbench stretches one clock
to 50 us, so the whole run is about 10 s.
"""

import argparse
import bisect
import os
import re
import statistics
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# key -> name suffix under the top scope (shortest matching name wins)
SIGNALS = {
    "power":  "u_pwr.power_uw",
    "vid":    "u_pwr.vid",
    "ready":  "u_pwr.ready",
    "energy": "u_energy.energy",
    "tick":   "tick",
}
LEVEL_OF_VID = {0b11: "high", 0b01: "normal", 0b00: "low"}
LEVEL_LABEL = {"high": "high (VID 11)", "normal": "normal (VID 01)",
               "low": "low (VID 00)", "off": "not in RUN", "other": "undefined VID"}
COLOR = {"high": "#f56565", "normal": "#48bb78", "low": "#4299e1",
         "off": "#cbd5e0", "other": "#a0aec0"}
UNIT_S = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9, "ps": 1e-12, "fs": 1e-15}


# ----------------------------------------------------------------------------
# VCD reading (only the signals in SIGNALS are kept)
# ----------------------------------------------------------------------------
def parse_timescale(tokens):
    text = "".join(t for t in tokens if t != "$end")
    m = re.fullmatch(r"(\d+)(s|ms|us|ns|ps|fs)", text)
    if m is None:
        sys.exit(f"unsupported VCD timescale: {text!r}")
    return int(m.group(1)) * UNIT_S[m.group(2)]


def find_signal(names, top, suffix):
    hits = [f for f in names if f.startswith(top + ".") and f.endswith("." + suffix)]
    return min(hits, key=lambda f: f.count("."), default=None)


def bits_value(bits):
    if any(c in "xXzZ" for c in bits):
        return 0
    return int(bits, 2)


def record(series, t, value):
    times, vals = series
    if times and times[-1] == t:
        vals[-1] = value
    else:
        times.append(t)
        vals.append(value)


def read_vcd(path):
    scope, names, top = [], {}, None
    unit_s, ts_tokens, in_defs = None, None, True
    code_key = {}
    series = {key: ([], []) for key in SIGNALS}
    t = 0
    with open(path) as f:
        for line in f:
            tok = line.split()
            if not tok:
                continue
            if in_defs:
                if ts_tokens is not None:
                    ts_tokens += tok
                    if "$end" in tok:
                        unit_s = parse_timescale(ts_tokens)
                        ts_tokens = None
                elif tok[0] == "$timescale":
                    ts_tokens = tok[1:]
                    if "$end" in ts_tokens:
                        unit_s = parse_timescale(ts_tokens)
                        ts_tokens = None
                elif tok[0] == "$scope":
                    scope.append(tok[2])
                    top = top or tok[2]
                elif tok[0] == "$upscope":
                    scope.pop()
                elif tok[0] == "$var":
                    names.setdefault(".".join(scope + [tok[4]]), tok[3])
                elif tok[0] == "$enddefinitions":
                    in_defs = False
                    for key, suffix in SIGNALS.items():
                        full = find_signal(names, top, suffix)
                        if full is None:
                            sys.exit(f"signal not found in VCD: *.{suffix}")
                        code_key[names[full]] = key
                continue
            head = tok[0]
            if head.startswith("#"):
                t = int(head[1:])
            elif head[0] in "bB" and len(tok) >= 2:
                key = code_key.get(tok[1])
                if key:
                    record(series[key], t, bits_value(head[1:]))
            elif head[0] in "01xXzZ" and len(head) > 1:
                key = code_key.get(head[1:])
                if key:
                    record(series[key], t, bits_value(head[0]))
    if unit_s is None:
        sys.exit("VCD has no $timescale")
    return series, unit_s


def value_at(series, t):
    times, vals = series
    i = bisect.bisect_right(times, t) - 1
    return vals[i] if i >= 0 else 0


# ----------------------------------------------------------------------------
# Samples: one row per time that power, VID, ready or energy changes
# ----------------------------------------------------------------------------
def build_rows(series, unit_s):
    change_times = set()
    for key in ("power", "vid", "ready", "energy"):
        change_times.update(series[key][0])
    rows = []
    for u in sorted(change_times):
        ready = value_at(series["ready"], u)
        vid = value_at(series["vid"], u)
        cls = "off" if not ready else LEVEL_OF_VID.get(vid, "other")
        rows.append({"t": u * unit_s, "cls": cls,
                     "power": value_at(series["power"], u),
                     "energy": value_at(series["energy"], u)})
    return rows


def tick_period_s(series, unit_s):
    times, vals = series["tick"]
    rises = [t for t, v in zip(times, vals) if v == 1]
    gaps = sorted(b - a for a, b in zip(rises, rises[1:]))
    if not gaps:
        return None
    return statistics.median(gaps) * unit_s


# ----------------------------------------------------------------------------
# Time and energy per level
# ----------------------------------------------------------------------------
def summarise(rows, tick_s):
    time_at = {k: 0.0 for k in COLOR}
    energy = {k: 0.0 for k in COLOR}
    for a, b in zip(rows, rows[1:]):
        time_at[a["cls"]] += b["t"] - a["t"]
        if tick_s is not None:
            energy[a["cls"]] += (b["energy"] - a["energy"]) * tick_s   # counts x s = uJ
    return time_at, energy


def print_summary(rows, time_at, energy, tick_s):
    run = sum(time_at[k] for k in ("high", "normal", "low"))
    print(f"\nrun time in RUN: {run:.3f} s   total window: {rows[-1]['t'] - rows[0]['t']:.3f} s")
    if tick_s is not None:
        print(f"energy tick period: {tick_s * 1e3:.3f} ms")
    print("\n== time at each level ==")
    for k in ("normal", "high", "low"):
        share = 100 * time_at[k] / run if run else 0.0
        print(f"{k:<7} {time_at[k]:8.3f} s   {share:5.1f} % of RUN   "
              f"energy {energy[k]:12.3f} uJ")
    for k in ("other", "off"):
        if time_at[k] > 0:
            print(f"{k:<7} {time_at[k]:8.3f} s   (not counted above)")

    normal = 100 * time_at["normal"] / run if run else 0.0
    high = 100 * time_at["high"] / run if run else 0.0
    print("\n== requirements ==")
    print(f"{'PASS' if normal >= 50 else 'FAIL'}: normal at least 50 % ({normal:.1f} %)")
    print(f"{'PASS' if high >= 100 / 3 else 'FAIL'}: high at least 1/3 ({high:.1f} %)")
    print(f"low takes the rest ({100 * time_at['low'] / run if run else 0:.1f} %)")

    total_uj = sum(energy.values())
    avg_uw = total_uj / run if run else 0.0
    print(f"\ntotal energy: {total_uj:.3f} uJ   average power in RUN: {avg_uw:.1f} uW")


# ----------------------------------------------------------------------------
# Plot
# ----------------------------------------------------------------------------
def plot(rows, tick_s, path, title):
    t = [r["t"] for r in rows]
    power = [r["power"] for r in rows]
    scale = tick_s if tick_s is not None else 1.0
    energy_y = [r["energy"] * scale for r in rows]
    energy_label = "cumulative energy (uJ)" if tick_s is not None else "cumulative energy (counts)"

    # merge neighbouring samples with the same level into one shaded span
    spans = []
    for a, b in zip(rows, rows[1:]):
        if spans and spans[-1][2] == a["cls"]:
            spans[-1][1] = b["t"]
        else:
            spans.append([a["t"], b["t"], a["cls"]])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for ax in (ax1, ax2):
        for s0, s1, cls in spans:
            ax.axvspan(s0, s1, color=COLOR[cls], alpha=0.35, lw=0)

    ax1.step(t, power, where="post", color="black", lw=1.0)
    ax1.set_ylabel("power (uW)")
    ax1.set_title(title, fontsize=11)

    ax2.step(t, energy_y, where="post", color="black", lw=1.2)
    ax2.set_ylabel(energy_label)
    ax2.set_xlabel("device time (s)")

    handles = [Patch(color=COLOR[k], alpha=0.35, label=LEVEL_LABEL[k])
               for k in ("high", "normal", "low", "off")]
    ax1.legend(handles=handles, loc="upper left", fontsize=8)

    ax2.set_xlim(t[0], t[-1])
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Plot power and energy from tb_power_long.vcd")
    ap.add_argument("vcd", help="VCD file from tb_power_long")
    ap.add_argument("--out", default="power_energy.png", help="output PNG")
    args = ap.parse_args()

    series, unit_s = read_vcd(args.vcd)
    rows = build_rows(series, unit_s)
    if not rows:
        sys.exit("no samples found in the VCD")
    tick_s = tick_period_s(series, unit_s)
    if tick_s is None:
        print("warning: no tick pulses found; energy is shown in counts")

    time_at, energy = summarise(rows, tick_s)
    print_summary(rows, time_at, energy, tick_s)

    plot(rows, tick_s, args.out, f"{os.path.basename(args.vcd)}: power and energy over device time")
    print(f"\nwrote {args.out}")
    plt.show()


if __name__ == "__main__":
    main()