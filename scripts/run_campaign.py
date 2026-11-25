#!/usr/bin/env python3
"""Run predeclared multi-scene, multi-seed paired full-step benchmarks.

All output is local. Run conditions are metadata, not performance normalization.
Nothing is silently excluded; any invalid trial fails the campaign.
"""
import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import tempfile
from pathlib import Path
from statistics import median

DEFAULT_CASES = (("sparse", 300), ("rain", 120), ("stack", 100))
DEFAULT_SEEDS = (3, 11, 27)


def audit(rows):
    groups = {}
    for row in rows:
        key = tuple(row[k] for k in ("scene", "bodies", "seed", "frames", "repeat"))
        group = groups.setdefault(key, {})
        if row["mode"] in group:
            raise ValueError(f"duplicate mode: {key} {row['mode']}")
        group[row["mode"]] = row
    ratios = {}
    for key, group in groups.items():
        if set(group) != {"brute", "sap"}:
            raise ValueError(f"incomplete pair: {key}")
        a, b = group["brute"], group["sap"]
        if a["final_hash"] != b["final_hash"]:
            raise ValueError(f"divergent physics: {key}")
        for row in (a, b):
            value = float(row["mean_step_us"])
            if not 0 < value < float("inf"):
                raise ValueError(f"invalid timing: {key}")
        ratios.setdefault((key[0], key[1]), []).append(
            float(a["mean_step_us"]) / float(b["mean_step_us"])
        )
    return {f"{s}-{b}": {"pairs": len(v), "paired_median_brute_over_sap": round(median(v), 4),
                          "min": round(min(v), 4), "max": round(max(v), 4)}
            for (s, b), v in sorted(ratios.items())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default="build/rigidstep")
    ap.add_argument("--output", default="lab-output/campaign")
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--repeats", type=int, default=4)
    args = ap.parse_args()
    if args.frames < 1 or args.repeats < 1:
        ap.error("frames and repeats must be positive")
    dest = Path(args.output)
    dest.mkdir(parents=True, exist_ok=True)
    all_rows = []
    for scene, n in DEFAULT_CASES:
        for seed in DEFAULT_SEEDS:
            with tempfile.TemporaryDirectory() as temp:
                output = Path(temp) / "trial.csv"
                subprocess.run([args.exe, "benchmark", "--scene", scene,
                                "--bodies", str(n), "--seed", str(seed),
                                "--frames", str(args.frames), "--repeats", str(args.repeats),
                                "--out", str(output)], check=True,
                               stdout=subprocess.DEVNULL)
                with output.open(newline="") as f:
                    all_rows.extend(csv.DictReader(f))
    stats = audit(all_rows)
    csv_path = dest / "trials.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    binary_sha = hashlib.sha256(Path(args.exe).read_bytes()).hexdigest()
    metadata = {"scope": "single-machine, synthetic scenes, fixed seeds; exploratory full-step timing",
                "platform": platform.platform(), "processor": platform.processor(),
                "python": platform.python_version(), "binary_sha256": binary_sha,
                "frames": args.frames, "repeats_per_seed": args.repeats,
                "seeds": list(DEFAULT_SEEDS), "results": stats,
                "csv_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
                "caveats": ["same host; no CPU affinity/frequency controls", "same seeds and scene types",
                            "timings include scheduler interference; no engine-wide speedup claim"]}
    (dest / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(stats, indent=2))
    print("PASS: no missing pairs, no final-state disagreement")


if __name__ == "__main__":
    main()
