# Box/box face-clipping ablation — negative and mixed results

## Why we tested it

Adding four contact points to a face/face overlap is a meaningful geometric change, **not** automatic evidence of better numerical simulation. A naive iterative impulse solver can amplify contact ordering, bias and friction errors. We compared the older single-point implementation against the new face-clipped one before claiming any stability gain.

## Reproduce the exact comparison

Both executables use the same current header and CLI shell; the **only physics implementation** switched is `physics.cpp`, with the historical original checked in under [`benchmarks/legacy-v01/physics.cpp`](../benchmarks/legacy-v01/physics.cpp). The default `--sphere-plane-ccd off` makes CCD inactive in this ablation. Compile both with the same compiler optimization and repeat the fully predeclared seeds and scenes:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
c++ -std=c++20 -O2 -Iinclude benchmarks/legacy-v01/physics.cpp \
  apps/rigidstep.cpp -o /tmp/rigidstep-v01
python3 scripts/compare_stack_versions.py \
  --baseline /tmp/rigidstep-v01 --candidate build/rigidstep \
  --output lab-output/stack-ablation
```

Three seeds (3, 11, 27), three stack sizes (32, 64, 120) and 600 fixed steps/run produce **18 independent program executions**, paired by scene geometry, seed, fixed dt and 240-step evaluation window. The script computes median reported kinetic energy, standard deviation of mean body-center Y, and geometric floor penetration from body poses. `com_y_std` is a motion proxy, not direct contact jitter; it includes any genuine moving dynamics. The baseline compiler and candidate should be identical for a clean repeat. No p-values, broad generalization or true-physics claim follows from these three seeds.

## Observations from one GCC/Linux local campaign

The following values are medians across three **paired per-seed ratios** (face-clipping / single-point); lower is generally preferable for these proxy metrics but not sufficient to prove physical fidelity:

| Stack size | Tail kinetic ratio | COM-height-variability ratio | Mean floor penetration ratio |
|---:|---:|---:|---:|
| 32 | 1.136× | 1.092× | 0.996× |
| 64 | 0.992× | 1.114× | 0.983× |
| 120 | 0.932× | 0.721× | 1.013× |

**Interpretation:** the 120-body scene has lower movement variability in this small comparison, but both 32/64-body scenarios have equal or worse motion variability, and floor penetration does not improve consistently. This is a **mixed result**. The face manifold passes functional tests, but we explicitly **reject** an all-scene stability or accuracy improvement claim.

Full source/CSV hashes and 18 raw summary rows are preserved at [`evidence/stack-ablation-20261125`](../evidence/stack-ablation-20261125/). Source snapshots can be compiled exactly from this repo. Generated full traces were not checked in because they are deterministic intermediates and large; the audit script regenerates them.

## What this changes in the roadmap

Prioritize contact persistence, stable feature IDs, warm-started impulses, separate position correction, stronger angular/stack invariants and comparison to an independent reference engine (Bullet/Jolt) before trying to optimize this solver for more bodies. Improving collision geometry alone cannot validate response dynamics. Do not advertise face clipping as a win without revisiting these regression cases.
