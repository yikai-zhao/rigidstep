# Experimental method and evidence integrity

## Hypothesis

Sweep-and-Prune (SAP) should reduce *full-step* wall time compared with brute-force AABB candidate checks when large numbers of objects have few X-axis overlaps. SAP may be neutral or slower in dense scenes because sorting and generating the same contacts become significant overhead.

## Experimental conditions

- Host shown during capture: Linux x86_64 container, AMD EPYC 9V74, 5 logical CPUs exposed, GCC 14.2.0 (`-O3` equivalent CMake Release default flags, unless toolchain overrides), C++20.
- Scenarios use a fixed seed (11) and identical initial conditions for SAP/brute per paired repetition.
- `sparse`: 800 moving spheres in a grid, 120 simulation steps per run, seven repetitions; `stack`: 120 mixed boxes/spheres, 120 steps per run, seven repetitions.
- The order of algorithms alternates each repetition to mitigate first/second run effects. Each mode starts from the same initial state and integrates its **entire** full-step physics independently.
- Per-step latency comes from `std::chrono::steady_clock`, includes gravity/integration, broadphase, contact generation, impulses, stabilization, hashing; only process wall time, **not** CPU hardware performance counters or GPU frame time.
- Report medians of **paired ratios** of mean step latency. Do not divide an isolated minimum by an isolated maximum.
- Every pair is **rejected** if final state hashes disagree; every value must be strictly positive and finite. `scripts/benchmark_report.py` enforces these gates. Source code contains full command-line parameters, output CSV and the report algorithm.

## Reproduction

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/rigidstep benchmark --scene sparse --bodies 800 --frames 120 --repeats 7 --seed 11 --out local-sparse.csv
python3 scripts/benchmark_report.py local-sparse.csv
./build/rigidstep benchmark --scene stack --bodies 120 --frames 120 --repeats 7 --seed 11 --out local-stack.csv
python3 scripts/benchmark_report.py local-stack.csv
```

## Observed results

`sparse`, seven pair ratios: 1.593, 1.629, 1.612, 1.469, 1.579, 1.635, 1.468; **paired median 1.593×**, all pairs identical final hash `9d42fb9701067755`.

`stack`, seven pair ratios: 0.908, 1.031, 0.867, 0.831, 0.982, 0.984, 0.954; **paired median 0.954×**, all pairs identical final hash `61e70923e65cc831`.

Numbers are printed to three decimals; source CSV stores actual values to 0.001 µs. Run-to-run fluctuation is real. The case where SAP loses is retained as a **negative control** against universal speedup claims.

## What remains unproven

- No core affinity or CPU frequency stabilization was enforced, and container scheduling/other tenants may affect data.
- Single hardware environment and one seed per scenario. Repeated frames inside a run are serially correlated, and repeats using the same seed do not represent different world realizations.
- Hot-cache/memory allocations are not isolated. The benchmark is full-step latency, so the numeric ratio cannot be attributed solely to SAP.
- Raw trial results are **not** wall-clock-normalized across machines. p99 is a sample quantile, not a reliable production tail guarantee.
- Must add multiple workload seeds, wider object-count scaling, randomized order, perf/flamegraph sampling on a controlled host, and regression confidence intervals before making general conclusions.

In hiring discussions, the strongest reproducible statement is: on the reported synthetic sparse scene, SAP reduced full-step mean time without changing the simulated state; on the dense scene it did not. That is a bounded result, not blanket engine optimization.
