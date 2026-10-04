#!/usr/bin/env python3
"""
compare.py

Runs the scenario three times (headless) and compares them:

    baseline   no V2X, signals run their normal programs
    signals    V2I + I2I signal preemption
    full       V2V lane clearing + V2I + I2I

Writes results/comparison.png and results/comparison.csv and prints a table.

    ./compare.sh
    ./compare.sh --latency 0.1 --loss 0.2     # extra options go to every run
"""

import csv
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = [("baseline", ["--baseline"], "Baseline (no V2X)", "#c0392b"),
        ("signals", ["--no-v2v"], "V2I + I2I", "#e6a100"),
        ("full", [], "V2V + V2I + I2I", "#27ae60")]


def load(tag):
    with open(os.path.join(HERE, "results", f"ambulances_{tag}.csv")) as f:
        return list(csv.DictReader(f))


def main():
    extra = sys.argv[1:]
    for tag, flags, label, _ in RUNS:
        print(f"running: {label} ...", flush=True)
        cmd = [os.path.join(HERE, ".venv", "bin", "python"),
               os.path.join(HERE, "v2x_city.py"), "--nogui"] + flags
        if "--baseline" not in flags:
            cmd += extra
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)

    data = {tag: load(tag) for tag, *_ in RUNS}
    ambs = [r["ambulance"] for r in data["baseline"]]

    print()
    print(f"{'':<10}" + "".join(f"{label:>22}" for _, _, label, _ in RUNS))
    rows = []
    for metric, key, unit in (("travel time", "travel_s", "s"),
                              ("stopped", "stopped_s", "s"),
                              ("stops", "stops", "")):
        totals = [sum(float(r[key]) for r in data[tag]) for tag, *_ in RUNS]
        print(f"{metric:<10}" + "".join(f"{t:>20.1f} {unit:<1}" for t in totals))
        rows.append([metric] + totals)
    base, full = rows[0][1], rows[0][3]
    print(f"\nambulance travel time saved by full V2X: "
          f"{base - full:.0f} s ({100 * (base - full) / base:.0f} %)")

    with open(os.path.join(HERE, "results", "comparison.csv"), "w",
              newline="") as f:
        w = csv.writer(f)
        w.writerow(["ambulance"] + [f"{t}_travel_s" for t, *_ in RUNS]
                   + [f"{t}_stopped_s" for t, *_ in RUNS])
        for i, a in enumerate(ambs):
            w.writerow([a]
                       + [data[t][i]["travel_s"] for t, *_ in RUNS]
                       + [data[t][i]["stopped_s"] for t, *_ in RUNS])

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    width = 0.27
    for ax, key, title in ((axes[0], "travel_s", "Travel time per ambulance (s)"),
                           (axes[1], "stopped_s", "Time stopped per ambulance (s)")):
        for j, (tag, _, label, color) in enumerate(RUNS):
            ax.bar([i + (j - 1) * width for i in range(len(ambs))],
                   [float(r[key]) for r in data[tag]],
                   width, label=label, color=color)
        ax.set_xticks(range(len(ambs)))
        ax.set_xticklabels(ambs)
        ax.set_title(title)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False)
    fig.suptitle(f"Emergency corridor: 8 ambulances, 10 signals - "
                 f"{100 * (base - full) / base:.0f} % less travel time with V2X",
                 fontweight="bold")
    fig.tight_layout()
    out = os.path.join(HERE, "results", "comparison.png")
    fig.savefig(out, dpi=150)
    print(f"\nchart: {out}")


if __name__ == "__main__":
    main()
