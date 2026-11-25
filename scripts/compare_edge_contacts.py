#!/usr/bin/env python3
"""Paired geometric edge-witness ablation, not a physical-accuracy oracle.

Predeclared scenes and seeds, cold-start solver, same initial body states.
Modes differ only by SAT edge witness approximation vs segment-segment minimum.
"""
import argparse
import csv
import hashlib
import json
import math
import platform
import subprocess
import tempfile
from pathlib import Path
from statistics import mean, median, pstdev


def one_run(exe, scene, size, seed, enabled, frames, tail):
    with tempfile.TemporaryDirectory() as td:
        trace = Path(td) / 'trace.jsonl'
        cmd = [str(exe), 'demo', '--scene', scene, '--bodies', str(size),
               '--seed', str(seed), '--frames', str(frames), '--warm-start', 'off',
               '--edge-contact', 'on' if enabled else 'off', '--out', str(trace)]
        subprocess.run(cmd, check=True, text=True, capture_output=True)
        data = [json.loads(line) for line in trace.read_text().splitlines()]
    if len(data) != frames + 1 or any(r['tick'] != i for i, r in enumerate(data)):
        raise RuntimeError('missing frames')
    if any(len(r['bodies']) != size or r['edge_contacts'] > r['contacts'] or
           r['face_contacts'] + r['edge_contacts'] > r['contacts'] or
           r['warm_applied'] or r['cached_contacts'] for r in data):
        raise RuntimeError('invalid contact or cold-start accounting')
    for r in data:
        if not all(math.isfinite(v) for v in [r['energy'], r['max_penetration']] +
                   [v for b in r['bodies'] for v in b['p'] + b['v'] + b['q']]):
            raise RuntimeError('non-finite simulation')
    last = data[-tail:]
    # Engine state hashes intentionally include the experimental setting flag.
    # Compare serialized physical *body states* for unaffected-scene parity.
    frames_body_hash = hashlib.sha256(
        ''.join(json.dumps(r['bodies'], sort_keys=True, separators=(',', ':')) + '\n'
                for r in data).encode('utf8')).hexdigest()
    last_body_hash = hashlib.sha256(
        json.dumps(data[-1]['bodies'], sort_keys=True, separators=(',', ':')).encode('utf8')
    ).hexdigest()
    return {'scene': scene, 'bodies': size, 'seed': seed,
            'edge_witness': 'on' if enabled else 'off',
            'edge_contacts_total': sum(r['edge_contacts'] for r in data),
            'median_tail_kinetic': median(r['energy'] for r in last),
            'com_height_std': pstdev(mean(b['p'][1] for b in r['bodies']) for r in last),
            'median_max_penetration': median(r['max_penetration'] for r in last),
            'final_hash': data[-1]['hash'],
            'final_body_sha256': last_body_hash,
            'all_frames_body_sha256': frames_body_hash}, data[0]['bodies']


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--exe', default='build/rigidstep')
    p.add_argument('--output', default='lab-output/edge-ablation')
    p.add_argument('--scenes', nargs='+', default=['rain', 'stack'])
    p.add_argument('--sizes', nargs='+', type=int, default=[48, 96])
    p.add_argument('--seeds', nargs='+', type=int, default=[3, 11, 29])
    p.add_argument('--frames', type=int, default=300)
    p.add_argument('--tail', type=int, default=100)
    args = p.parse_args()
    if (not args.scenes or not args.sizes or not args.seeds or
        any(len(set(v)) != len(v) for v in (args.scenes, args.sizes, args.seeds)) or
        any(n <= 0 for n in args.sizes + args.seeds) or
        args.frames < 2 or not (1 <= args.tail <= args.frames)):
        p.error('invalid scene/size/seed/frame settings')
    exe = Path(args.exe).resolve()
    if not exe.is_file(): p.error('missing binary')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    rows, comparisons = [], []
    for scene in args.scenes:
        for size in args.sizes:
            for seed in args.seeds:
                order = (False, True) if seed % 2 else (True, False)
                starts = []
                pair = {}
                for enabled in order:
                    result, start = one_run(exe, scene, size, seed, enabled, args.frames, args.tail)
                    starts.append(start)
                    rows.append(result)
                    pair['on' if enabled else 'off'] = result
                if starts[0] != starts[1]: raise RuntimeError('different paired initial states')
                if pair['off']['edge_contacts_total'] == 0:
                    # A useless comparison for that pair, still report it honestly.
                    eligible = False
                else:
                    eligible = True
                same_physics = (pair['off']['all_frames_body_sha256'] ==
                                pair['on']['all_frames_body_sha256'])
                if not eligible and not same_physics:
                    raise RuntimeError('no edge geometry exercised yet body trajectories diverged')
                comparisons.append({
                    'scene': scene, 'bodies': size, 'seed': seed,
                    'eligible_edge_contacts': eligible,
                    'baseline_edge_count': pair['off']['edge_contacts_total'],
                    'precise_edge_count': pair['on']['edge_contacts_total'],
                    'delta_median_kinetic': pair['on']['median_tail_kinetic'] - pair['off']['median_tail_kinetic'],
                    'delta_com_height_std': pair['on']['com_height_std'] - pair['off']['com_height_std'],
                    'delta_median_max_penetration': pair['on']['median_max_penetration'] - pair['off']['median_max_penetration'],
                    'physical_trajectory_equal': same_physics,
                    'final_body_equal': pair['off']['final_body_sha256'] == pair['on']['final_body_sha256']
                })
    for name, values in [('results.csv', rows), ('pairs.csv', comparisons)]:
        with (out/name).open('w', newline='', encoding='utf8') as f:
            w = csv.DictWriter(f, fieldnames=list(values[0]))
            w.writeheader();w.writerows(values)
        (out/name).write_bytes((out/name).read_bytes().replace(b'\r\n', b'\n'))
    eligible = [r for r in comparisons if r['eligible_edge_contacts']]
    summary = {'purpose': 'experimental segment-segment edge contact vs old midpoint',
               'interpretation': 'No ground truth engine; motion, energy and penetration are proxies, not physical correctness',
               'scenes': args.scenes, 'sizes': args.sizes, 'seeds': args.seeds,
               'frames': args.frames, 'tail': args.tail, 'pairs': len(comparisons),
               'eligible_pairs': len(eligible), 'positive_edge_contacts_total': sum(r['precise_edge_count'] for r in comparisons),
               'median_delta_com_height_std_eligible': median(r['delta_com_height_std'] for r in eligible) if eligible else None,
               'positive_delta_count_eligible': sum(r['delta_com_height_std'] > 0 for r in eligible),
               'source_sha256': hashlib.sha256(Path('src/physics.cpp').read_bytes()).hexdigest(),
               'header_sha256': hashlib.sha256(Path('include/rigidstep/physics.hpp').read_bytes()).hexdigest(),
               'binary_sha256': hashlib.sha256(exe.read_bytes()).hexdigest(),
               'platform': platform.platform(),
               'files_sha256': {name: hashlib.sha256((out/name).read_bytes()).hexdigest() for name in ('results.csv','pairs.csv')}}
    (out/'metadata.json').write_text(json.dumps(summary, indent=2) + '\n')
    print('PASS',summary['pairs'],'pairs; eligible',summary['eligible_pairs'],
          'edge contacts',summary['positive_edge_contacts_total'])
    print('median delta height std:',summary['median_delta_com_height_std_eligible'],
          'positive/worse:',summary['positive_delta_count_eligible'])

if __name__ == '__main__':main()
