#!/usr/bin/env python3
"""Audit and summarize paired runs; do not invent confidence intervals."""
import argparse
import csv
import math
from collections import defaultdict
from statistics import median


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV from `rigidstep benchmark`")
    args = ap.parse_args()
    grouped = defaultdict(dict)
    with open(args.csv, newline="", encoding="utf8") as f:
        for row in csv.DictReader(f):
            key = (row["scene"], row["bodies"], row["seed"], row["frames"], row["repeat"])
            mode = row["mode"]
            if mode in grouped[key]:
                raise ValueError(f"duplicate mode for {key}: {mode}")
            grouped[key][mode] = row
    if not grouped:
        raise ValueError("empty CSV")
    ratios = []
    for key, run in sorted(grouped.items()):
        if set(run) != {"brute", "sap"}:
            raise ValueError(f"unpaired modes for {key}: {list(run)}")
        a, b = run["brute"], run["sap"]
        if a["final_hash"] != b["final_hash"]:
            raise ValueError(f"MISMATCH: physics states differ for {key}")
        for mode, trial in run.items():
            if any(not math.isfinite(float(trial[x])) or float(trial[x]) <= 0
                   for x in ("p50_step_us", "p95_step_us", "p99_step_us", "mean_step_us")):
                raise ValueError(f"non-positive/non-finite timing in {key} {mode}")
        ratio = float(a["mean_step_us"]) / float(b["mean_step_us"])
        ratios.append(ratio)
        print(f"pair {key[-1]}: brute={a['mean_step_us']}us  sap={b['mean_step_us']}us"
              f"  brute/sap={ratio:.3f}x  state={a['final_hash']}")
    print(f"pairs={len(ratios)} equal_final_states=yes "
          f"paired_median_ratio={median(ratios):.3f}x "
          f"range={min(ratios):.3f}x..{max(ratios):.3f}x")
    print("CAUTION: single environment, repeated same seed; not independent workloads or a universal speedup.")


if __name__ == "__main__":
    main()
