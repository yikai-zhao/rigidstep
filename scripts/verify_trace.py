#!/usr/bin/env python3
"""Structural trace validator: strict enough to catch truncated/incoherent demos."""
import argparse
import json
import math


def main():
    p = argparse.ArgumentParser()
    p.add_argument("trace")
    p.add_argument("--require-warm-applied", action="store_true")
    args = p.parse_args()
    count = 0
    identity = None
    dt0 = None
    hashes = set()
    total_warm_applied = 0
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
            diagnostics = ("contacts", "warm_matched", "warm_applied", "cached_contacts")
            if any(name in record for name in diagnostics[1:]):
                if not all(isinstance(record.get(key), int) and record[key] >= 0 for key in diagnostics):
                    raise ValueError(f"line {line_no}: invalid contact counters")
                if not (record["warm_applied"] <= record["warm_matched"] <= record["contacts"] and
                        record["cached_contacts"] <= record["contacts"]):
                    raise ValueError(f"line {line_no}: invalid contact cache accounting")
                total_warm_applied += record["warm_applied"]
            if "edge_contacts" in record or "face_contacts" in record:
                if not all(isinstance(record.get(k),int) and record[k]>=0
                           for k in ("edge_contacts","face_contacts")):
                    raise ValueError(f"line {line_no}: invalid SAT feature counts")
                if record["edge_contacts"] + record["face_contacts"] > record["contacts"]:
                    raise ValueError(f"line {line_no}: SAT feature counts exceed contact total")
            if "contact_snapshot" in record:
                if not isinstance(record["contact_snapshot"],list) or len(record["contact_snapshot"]) > 100000:
                    raise ValueError(f"line {line_no}: invalid contact snapshot size")
                for c in record["contact_snapshot"]:
                    if not (isinstance(c.get("a"),int) and isinstance(c.get("b"),int) and
                            c["a"]>=0 and c["b"]>=0 and c["a"]!=c["b"] and
                            c["feature"] in {"sphere-sphere","sphere-box","box-face","box-edge","ground"} and
                            len(c["p"])==3 and len(c["n"])==3 and
                            isinstance(c["depth"],(int,float)) and math.isfinite(c["depth"]) and c["depth"]>=0):
                        raise ValueError(f"line {line_no}: malformed contact snapshot")
                    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in c["p"]+c["n"]):
                        raise ValueError(f"line {line_no}: nonfinite contact")
                    if not 0.999 <= math.sqrt(sum(v*v for v in c["n"])) <= 1.001:
                        raise ValueError(f"line {line_no}: contact normal not unit length")
            hashes.add(record["hash"])
            count += 1
    if count == 0:
        raise ValueError("empty trace")
    if args.require_warm_applied and total_warm_applied <= 0:
        raise ValueError("no warm-start impulses were applied")
    print(f"PASS: {count} consecutive frames; {len(identity)} consistent bodies; "
          f"dt={dt0}; {len(hashes)} distinct hashes; warm applied={total_warm_applied}")


if __name__ == "__main__":
    main()
