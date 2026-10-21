#!/usr/bin/env python3
"""
compare_relay.py

Isolates the --relay variable: runs v2x_city.py twice (headless, same
seed so both see identical traffic) - full V2X as-is, and full V2X with
multi-hop V2V relay turned on - and compares the totals.

This is deliberately separate from compare.py (which compares
baseline/signals/full and is left completely untouched) so the existing,
already-reported comparison.png/comparison.csv are never regenerated or
overwritten by this script.

Writes results/comparison_relay.png and results/comparison_relay.csv.

    python compare_relay.py
    python compare_relay.py --seed 3 --latency 0.05 --loss 0.1
"""

import argparse
import csv
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

RUNS = [
    ("full", [], "Full V2X (no relay)", "#c0392b"),
    ("full_relay", ["--relay"], "Full V2X + multi-hop relay", "#8e44ad"),
]


def load(tag):
    with open(os.path.join(HERE, "results", f"ambulances_{tag}.csv")) as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--latency", type=float, default=0.0)
    ap.add_argument("--loss", type=float, default=0.0)
    args, extra = ap.parse_known_args()

    for tag, flags, label, _ in RUNS:
        print(f"running: {label} ...", flush=True)
        cmd = [sys.executable, os.path.join(HERE, "v2x_city.py"), "--nogui",
               "--seed", str(args.seed),
               "--latency", str(args.latency), "--loss", str(args.loss)]
        cmd += flags + extra
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)

    data = {tag: load(tag) for tag, *_ in RUNS}
    ambs = [r["ambulance"] for r in data["full"]]

    print()
    print(f"{'':<10}" + "".join(f"{label:>28}" for _, _, label, _ in RUNS))
    rows = []
    for metric, key, unit in (("travel time", "travel_s", "s"),
                              ("stopped", "stopped_s", "s"),
                              ("stops", "stops", "")):
        totals = [sum(float(r[key]) for r in data[tag]) for tag, *_ in RUNS]
        print(f"{metric:<10}" + "".join(f"{t:>26.1f} {unit:<1}" for t in totals))
        rows.append([metric] + totals)

    base, relay = rows[0][1], rows[0][2]
    if base > 0:
        print(f"\nambulance travel time saved by multi-hop relay (on top of "
              f"the existing V2I+I2I): {base - relay:.0f} s "
              f"({100 * (base - relay) / base:.0f} %)")

    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    with open(os.path.join(HERE, "results", "comparison_relay.csv"), "w",
              newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric"] + [tag for tag, *_ in RUNS])
        w.writerows(rows)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    width = 0.35
    for ax, key, title in ((axes[0], "travel_s", "Travel time per ambulance (s)"),
                           (axes[1], "stopped_s", "Time stopped per ambulance (s)")):
        for j, (tag, _, label, color) in enumerate(RUNS):
            ax.bar([i + (j - 0.5) * width for i in range(len(ambs))],
                   [float(r[key]) for r in data[tag]],
                   width, label=label, color=color)
        ax.set_xticks(range(len(ambs)))
        ax.set_xticklabels(ambs)
        ax.set_title(title)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False)
    fig.suptitle("Effect of multi-hop V2V relay on top of existing V2I + I2I",
                 fontweight="bold")
    fig.tight_layout()
    out = os.path.join(HERE, "results", "comparison_relay.png")
    fig.savefig(out, dpi=150)
    print(f"\nchart: {out}")


if __name__ == "__main__":
    main()
