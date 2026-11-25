#!/usr/bin/env python3
"""Prove off-path bodies match the archived V0.2 solver on fixed seeded scenes.

The new state hash includes solver flags/cache and intentionally differs from V0.2;
we instead compare exact per-tick serialized body poses, velocities, and dynamics
metrics. This is a same-compiler regression test, not physical accuracy proof.
"""
import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

CASES=(("rain",40,3),("rain",40,17),("stack",64,3),("stack",64,17),
       ("sparse",80,3),("sparse",80,17))
FRAMES=180


def trace(exe, scene, bodies, seed, output):
    with tempfile.TemporaryDirectory() as tmp:
        out=Path(tmp)/'trace.jsonl'
        subprocess.run([str(exe),'demo','--scene',scene,'--bodies',str(bodies),
                        '--seed',str(seed),'--frames',str(FRAMES),
                        '--warm-start','off','--out',str(out)],
                       check=True, stdout=subprocess.DEVNULL)
        return [json.loads(line) for line in out.read_text().splitlines()]


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--baseline',required=True)
    p.add_argument('--candidate',default='build/rigidstep')
    p.add_argument('--output',default='lab-output/cold-v02-regression.json')
    args=p.parse_args()
    baseline=Path(args.baseline).resolve()
    candidate=Path(args.candidate).resolve()
    for exe in (baseline,candidate):
        if not exe.is_file(): p.error(f'missing executable: {exe}')
    results=[]
    for scene,count,seed in CASES:
        a=trace(baseline,scene,count,seed,args.output)
        b=trace(candidate,scene,count,seed,args.output)
        if len(a)!=len(b) or len(a)!=FRAMES+1: raise RuntimeError('trace length mismatch')
        for i,(left,right) in enumerate(zip(a,b)):
            for key in ('tick','candidates','contacts','max_penetration','energy','bodies'):
                if left[key]!=right[key]:
                    raise RuntimeError(f'{scene}/{count}/{seed} tick {i}: {key} diverged')
            if right['warm_applied'] or right['cached_contacts']:
                raise RuntimeError('disabled cache unexpectedly used')
        results.append({'scene':scene,'bodies':count,'seed':seed,'frames_matched':len(a),
                        'baseline_hash':a[-1]['hash'],'candidate_hash':b[-1]['hash'],
                        'body_state_equal':True,'hash_note':'different schema, not compared'})
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    report={'scope':'V0.2 baseline vs V0.3 warm-start off, exact JSON body equality',
            'baseline_source_sha256':hashlib.sha256(Path('benchmarks/legacy-v02/physics.cpp').read_bytes()).hexdigest(),
            'candidate_source_sha256':hashlib.sha256(Path('src/physics.cpp').read_bytes()).hexdigest(),
            'compiler':'both built with -std=c++20 -O2 on the same local system',
            'limitations':'same-host, source-file archive, does not prove physical accuracy',
            'results':results}
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(f'PASS: {len(results)} cases × {FRAMES+1} exact per-tick physical-state comparisons')

if __name__=='__main__':main()
