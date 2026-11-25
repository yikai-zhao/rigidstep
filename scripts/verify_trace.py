#!/usr/bin/env python3
"""Structural trace validator: strict enough to catch truncated/incoherent demos."""
import argparse
import json
import math


def main():
    p = argparse.ArgumentParser()
    p.add_argument("trace")
    args = p.parse_args()
    count = 0
    identity = None
    dt0 = None
    hashes = set()
    with open(args.trace, encoding="utf8") as f:
        for line_no, line in enumerate(f, 1):
            if not line.strip():
                continue
            record = json.loads(line)
            if record["tick"] != count:
                raise ValueError(f"line {line_no}: non-contiguous frame number")
            dt = record["dt"]
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError(f"line {line_no}: bad dt")
            if dt0 is None:
                dt0 = dt
            if dt != dt0:
                raise ValueError(f"line {line_no}: dt changed")
            bodies = record["bodies"]
            ids = tuple(b["id"] for b in bodies)
            if len(set(ids)) != len(ids):
                raise ValueError(f"line {line_no}: duplicate body ID")
            if identity is None:
                identity = ids
            if ids != identity:
                raise ValueError(f"line {line_no}: missing/reordered body")
            for b in bodies:
                if b["type"] not in ("sphere", "box"):
                    raise ValueError(f"line {line_no}: unsupported shape")
                if len(b["p"]) != 3 or len(b["v"]) != 3 or len(b["q"]) != 4:
                    raise ValueError(f"line {line_no}: malformed vector")
                for v in b["p"]+b["v"]+b["q"]:
                    if not isinstance(v, (float, int)) or not math.isfinite(v):
                        raise ValueError(f"line {line_no}: non-finite state")
            hashes.add(record["hash"])
            count += 1
    if count == 0:
        raise ValueError("empty trace")
    print(f"PASS: {count} consecutive frames; {len(identity)} consistent bodies; "
          f"dt={dt0}; {len(hashes)} distinct hashes")


if __name__ == "__main__":
    main()
