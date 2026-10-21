#!/usr/bin/env python3
"""
compare_relay.py

Runs v2x_relay_test.py twice (headless) with a FIXED seed so both runs
see identical traffic - once with multi-hop V2V relay disabled (direct
V2I range only) and once with it enabled - and compares the single
ambulance's travel time, stopped time and stop count between the two.

Same shape as ambulance_v2x_city/compare.py, adapted for v2x_relay's
one-ambulance results/ambulance_<tag>.csv format.

    python compare_relay.py
    python compare_relay.py --seed 7
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
    ("no_relay", ["--no-relay"], "Direct V2I range only", "#c0392b"),
    ("relay", [], "Multi-hop V2V relay", "#27ae60"),
]


def run_one(tag, flags, seed):
    print(f"running: {tag} ...", flush=True)
    cmd = [sys.executable, os.path.join(HERE, "v2x_relay_test.py"), "--nogui"] + flags
    if seed is not None:
        cmd += ["--seed", str(seed)]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


def load(tag):
    path = os.path.join(HERE, "results", f"ambulance_{tag}.csv")
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return rows[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=42,
                         help="SUMO seed shared by both runs (default 42) "
                              "so the comparison is apples-to-apples")
    args = parser.parse_args()

    for tag, flags, label, _ in RUNS:
        run_one(tag, flags, args.seed)

    data = {tag: load(tag) for tag, *_ in RUNS}

    print()
    print(f"{'':<14}" + "".join(f"{label:>24}" for _, _, label, _ in RUNS))
    rows = []
    for metric, key, unit in (
        ("travel time", "travel_s", "s"),
        ("stopped", "stopped_s", "s"),
        ("stops", "stops", ""),
    ):
        values = [float(data[tag][key]) for tag, *_ in RUNS]
        print(f"{metric:<14}" + "".join(f"{v:>22.1f} {unit:<1}" for v in values))
        rows.append([metric] + values)

    no_relay_travel = float(data["no_relay"]["travel_s"])
    relay_travel = float(data["relay"]["travel_s"])

    if no_relay_travel > 0:
        pct = 100 * (no_relay_travel - relay_travel) / no_relay_travel
        print(f"\nambulance travel time saved by multi-hop relay: "
              f"{no_relay_travel - relay_travel:.1f} s ({pct:.0f} %)")

    with open(os.path.join(HERE, "results", "comparison.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric"] + [tag for tag, *_ in RUNS])
        for row in rows:
            w.writerow(row)

    fig, ax = plt.subplots(figsize=(7, 4.8))
    labels = ["Travel time (s)", "Stopped time (s)"]
    for j, (tag, _, label, color) in enumerate(RUNS):
        values = [float(data[tag]["travel_s"]), float(data[tag]["stopped_s"])]
        ax.bar([i + (j - 0.5) * 0.35 for i in range(2)], values, 0.35,
               label=label, color=color)
    ax.set_xticks(range(2))
    ax.set_xticklabels(labels)
    ax.set_title("V2X relay: single ambulance, J1-J5 corridor")
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    out = os.path.join(HERE, "results", "comparison.png")
    fig.savefig(out, dpi=150)
    print(f"\nchart: {out}")


if __name__ == "__main__":
    main()
