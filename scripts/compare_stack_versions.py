#!/usr/bin/env python3
"""Cross-version stack ablation: compare measured traces, never infer causality from one seed.

Baseline is the checked-in historical physics.cpp compiled with current CLI shell;
only the solver implementation differs. No external dependency is needed.
"""
import argparse
import csv
import hashlib
import json
import math
import subprocess
import tempfile
from pathlib import Path
from statistics import mean, median, pstdev

SEEDS = (3, 11, 27)
SIZES = (32, 64, 120)
FRAMES = 600
TAIL = 240


def qrotate(q, v):
    w, x, y, z = q
    cx = y * v[2] - z * v[1]
    cy = z * v[0] - x * v[2]
    cz = x * v[1] - y * v[0]
    tx, ty, tz = [2 * a for a in (cx, cy, cz)]
    tx2 = y * tz - z * ty
    ty2 = z * tx - x * tz
    tz2 = x * ty - y * tx
    return (v[0] + w * tx + tx2, v[1] + w * ty + ty2,
            v[2] + w * tz + tz2)


def penetration(body):
    if body['type'] == 'sphere':
        radius = body['radius']
    else:
        half = body['half']
        q = body['q']
        radius = sum(abs(qrotate(q, [float(i == j) for i in range(3)])[1]) * half[j]
                     for j in range(3))
    return max(0.0, radius - body['p'][1])


def evaluate(rows):
    tail = rows[-TAIL:]
    if len(rows) != FRAMES + 1 or len(tail) != TAIL:
        raise ValueError('trace frame count mismatch')
    all_energies, heights, penetrations = [], [], []
    for row in tail:
        all_energies.append(row['energy'])
        heights.append(mean(b['p'][1] for b in row['bodies']))
        penetrations.extend(penetration(b) for b in row['bodies'])
    for series in (all_energies, heights, penetrations):
        if not all(math.isfinite(v) for v in series):
            raise ValueError('non-finite metric')
    return {'median_kinetic_last240': round(median(all_energies), 6),
            'com_y_std_last240': round(pstdev(heights), 6),
            'mean_floor_penetration_last240': round(mean(penetrations), 6),
            'max_floor_penetration_last240': round(max(penetrations), 6)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--baseline', required=True, help='executable compiled with benchmarks/legacy-v01/physics.cpp')
    p.add_argument('--candidate', default='build/rigidstep')
    p.add_argument('--output', default='lab-output/stack-ablation')
    args = p.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    all_rows = []
    for size in SIZES:
        for seed in SEEDS:
            for variant, exe in (('baseline', args.baseline), ('face_clipping', args.candidate)):
                with tempfile.TemporaryDirectory() as temp:
                    trace = Path(temp) / 'trace.jsonl'
                    subprocess.run([exe, 'demo', '--scene', 'stack', '--bodies', str(size),
                                    '--frames', str(FRAMES), '--seed', str(seed),
                                    '--out', str(trace)], check=True, stdout=subprocess.DEVNULL)
                    data = [json.loads(line) for line in trace.read_text().splitlines()]
                all_rows.append({'variant': variant, 'size': size, 'seed': seed,
                                 **evaluate(data)})
    dest = out / 'results.csv'
    with dest.open('w', newline='', encoding='utf8') as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    # Stable cross-platform newline normalization for the reproducibility hash.
    dest.write_bytes(dest.read_bytes().replace(b'\r\n', b'\n'))
    metadata = {'test': 'v0.1 single-point vs v0.2 face-clipped OBB stack ablation',
                'baseline_source_sha256': hashlib.sha256(Path('benchmarks/legacy-v01/physics.cpp').read_bytes()).hexdigest(),
                'candidate_source_sha256': hashlib.sha256(Path('src/physics.cpp').read_bytes()).hexdigest(),
                'seeds': list(SEEDS), 'sizes': list(SIZES), 'frames': FRAMES, 'tail': TAIL,
                'csv_sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
                'limitations': ['single host', 'three fixed seeds', 'the solver remains approximate',
                                'not a calibrated physical truth test', 'not a performance benchmark']}
    (out / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('PASS:', len(all_rows), 'traces across two solver versions; see', dest)


if __name__ == '__main__':
    main()
