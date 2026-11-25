#!/usr/bin/env python3
"""Fixed-seed warm-start on/off ablation with controls, no cherry-picked winners.

Tests a local engine research prototype, not a ground-truth physical simulator.
Every pair shares the exact initial body state, dt, collision and solver code.
Generated per-tick traces are temporary; complete per-run metrics and provenance
are written to CSV/JSON. Uses only the Python standard library.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import platform
import subprocess
import tempfile
from pathlib import Path
from statistics import mean, median, pstdev

SEEDS = (3, 11, 27)
SIZES = (24, 64, 120)
ITERATIONS = (2, 4)
FRAMES = 480
TAIL = 160


def rotate_y_component(q, basis):
    w, x, y, z = q
    vx, vy, vz = basis
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return vy + w*ty + (z*tx-x*tz)


def floor_penetration(body):
    if body['type'] == 'sphere':
        support = body['radius']
    else:
        h = body['half']
        q = body['q']
        support = sum(abs(rotate_y_component(q, tuple(int(axis == j) for axis in range(3)))) * h[j]
                      for j in range(3))
    return max(0.0, support - body['p'][1])


def run(exe, size, seed, iterations, warm, frames, tail):
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)/'trace.jsonl'
        cmd = [str(exe), 'demo', '--scene', 'stack', '--bodies', str(size),
               '--frames', str(frames), '--seed', str(seed), '--iterations', str(iterations),
               '--warm-start', warm, '--out', str(path)]
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        rows = [json.loads(line) for line in path.read_text(encoding='utf8').splitlines()]
    if len(rows) != frames+1 or any(r['tick'] != i for i, r in enumerate(rows)):
        raise RuntimeError('missing/reordered frames')
    if any(len(row['bodies']) != size for row in rows):
        raise RuntimeError('body count drift')
    for row in rows:
        vals = [row['energy'], row['max_penetration']]
        for body in row['bodies']:
            vals.extend(body['p']+body['v']+body['q'])
        if not all(math.isfinite(x) for x in vals):
            raise RuntimeError('non-finite state or diagnostic')
        if row['warm_applied'] > row['warm_matched'] or row['warm_matched'] > row['contacts']:
            raise RuntimeError('contact match accounting invalid')
        if row['cached_contacts'] > row['contacts']:
            raise RuntimeError('cache size exceeded live contacts')
        if warm == 'off' and (row['warm_applied'] or row['warm_matched'] or row['cached_contacts']):
            raise RuntimeError('cold control actually used a cache')
    tail_rows = rows[-tail:]
    heights = [mean(body['p'][1] for body in row['bodies']) for row in tail_rows]
    energies = [row['energy'] for row in tail_rows]
    penetrations = [floor_penetration(body) for row in tail_rows for body in row['bodies']]
    # Avoid hiding all pair complexity in one scalar: retain separate metrics.
    result = dict(warm=warm, size=size, seed=seed, iterations=iterations,
                  frames=frames, tail=tail,
                  median_kinetic=median(energies),
                  com_y_std=pstdev(heights),
                  mean_floor_penetration=mean(penetrations),
                  max_floor_penetration=max(penetrations),
                  median_contact_penetration=median(row['max_penetration'] for row in tail_rows),
                  warm_matched_total=sum(row['warm_matched'] for row in rows),
                  warm_applied_total=sum(row['warm_applied'] for row in rows),
                  max_cache=max(row['cached_contacts'] for row in rows),
                  final_hash=rows[-1]['hash'])
    return result, rows[0]['bodies']


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--exe', default='build/rigidstep')
    p.add_argument('--seeds', nargs='+', type=int, default=list(SEEDS))
    p.add_argument('--sizes', nargs='+', type=int, default=list(SIZES))
    p.add_argument('--iterations', nargs='+', type=int, default=list(ITERATIONS))
    p.add_argument('--frames', type=int, default=FRAMES)
    p.add_argument('--tail', type=int, default=TAIL)
    p.add_argument('--output', default='lab-output/warmstart-ablation')
    args = p.parse_args()
    seeds=tuple(args.seeds);sizes=tuple(args.sizes);iterations=tuple(args.iterations)
    if (not seeds or len(seeds)!=len(set(seeds)) or
        not sizes or len(sizes)!=len(set(sizes)) or
        not iterations or len(iterations)!=len(set(iterations)) or
        any(v<=0 for v in seeds+sizes+iterations) or args.frames<1 or args.tail<1 or args.tail>args.frames):
        p.error('positive unique seeds/sizes/iterations and 1 <= tail <= frames required')
    exe = Path(args.exe).resolve()
    if not exe.is_file():
        p.error('executable missing; build first')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for size in sizes:
        for seed in seeds:
            for iters in iterations:
                # Alternating order over predeclared seed/size/iterations to
                # reduce systematic differences from scheduling/thermal drift.
                modes = ('on','off') if (size+seed+iters)%2 else ('off','on')
                initial = None
                for mode in modes:
                    result, bodies = run(exe,size,seed,iters,mode,args.frames,args.tail)
                    if initial is None:
                        initial = bodies
                    elif bodies != initial:
                        raise RuntimeError('paired initial body states differ')
                    if mode == 'on' and result['warm_applied_total'] == 0:
                        raise RuntimeError('warm start enabled but never used')
                    records.append(result)
    paths = output/'results.csv'
    with paths.open('w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0]))
        writer.writeheader();writer.writerows(records)
    paths.write_bytes(paths.read_bytes().replace(b'\r\n',b'\n'))
    grouped={}
    for entry in records:
        key=(entry['size'],entry['seed'],entry['iterations'])
        grouped.setdefault(key,{})[entry['warm']]=entry
    if len(grouped)!=len(sizes)*len(seeds)*len(iterations) or any(set(pair)!={'off','on'} for pair in grouped.values()):
        raise RuntimeError('incomplete paired campaign')
    comparisons=[]
    for key,pair in sorted(grouped.items()):
        off,on=pair['off'],pair['on']
        comparisons.append({'size':key[0],'seed':key[1],'iterations':key[2],
                            **{f'{name}_on_minus_off':on[name]-off[name]
                               for name in ('median_kinetic','com_y_std','mean_floor_penetration')},
                            'warm_applied_total':on['warm_applied_total']})
    paired=output/'pairs.csv'
    with paired.open('w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(comparisons[0]))
        writer.writeheader();writer.writerows(comparisons)
    paired.write_bytes(paired.read_bytes().replace(b'\r\n',b'\n'))
    summary={}
    for size in sizes:
        for iters in iterations:
            row=[pair for pair in comparisons if pair['size']==size and pair['iterations']==iters]
            summary[f'{size}-iters{iters}']={
                name:{'median_on_minus_off':median(item[name] for item in row),
                      'on_better_count':sum(item[name]<0 for item in row),
                      'off_better_count':sum(item[name]>0 for item in row)}
                for name in ('median_kinetic_on_minus_off','com_y_std_on_minus_off','mean_floor_penetration_on_minus_off')}
    metadata={
        'hypothesis':'Does prior-contact impulse reuse improve low-iteration approximate box/sphere stacks?',
        'limits':'Not physical ground truth; 3 fixed seeds, synthetic scenes, no strong causality or universal speedup',
        'seeds':seeds,'sizes':sizes,'iterations':iterations,'frames':args.frames,'tail':args.tail,
        'python':platform.python_version(),'platform':platform.platform(),
        'compiler_info':subprocess.run(['c++','--version'],capture_output=True,text=True).stdout.splitlines()[0],
        'source_sha256':hashlib.sha256(Path('src/physics.cpp').read_bytes()).hexdigest(),
        'header_sha256':hashlib.sha256(Path('include/rigidstep/physics.hpp').read_bytes()).hexdigest(),
        'binary_sha256':hashlib.sha256(exe.read_bytes()).hexdigest(),
        'results_sha256':hashlib.sha256(paths.read_bytes()).hexdigest(),
        'pairs_sha256':hashlib.sha256(paired.read_bytes()).hexdigest(),
        'total_traces':len(records),'total_pairs':len(grouped),'summary':summary}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf8')
    print(f'PASS: {len(grouped)} complete fixed-seed paired cases; {len(records)} traces; written to {output}')
    for key,value in summary.items():
        print(key,'motion',value['com_y_std_on_minus_off'],
              'energy',value['median_kinetic_on_minus_off'])

if __name__ == '__main__':
    main()
