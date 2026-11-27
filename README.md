# RigidStep

[![C++ correctness](https://github.com/yikai-zhao/rigidstep/actions/workflows/ci.yml/badge.svg)](https://github.com/yikai-zhao/rigidstep/actions/workflows/ci.yml)

**Real-time C++20 3D rigid-body physics and performance lab (V0.4).** A small, readable simulation core with sphere/rotated-box collision handling, a fixed-timestep sequential-impulse solver, replayable state traces, a dependency-free WebGL2 GPU trace viewer, independent analytic validation, and measured broadphase trade-offs.

**Why this exists:** Real-time physics engines must trade numerical stability, frame-time budgets, collision-query cost, and deterministic testability. RigidStep builds and measures a concrete subset of that problem rather than wrapping an existing game engine and calling it original engine work.

> **Scope:** This is an engine research prototype, **not** Roblox's engine, a high-fidelity physics engine, or a general production simulator. OBB face contact uses clipped manifolds (up to four points), but edge-edge contact remains an approximate single point; **optional CCD handles downward-moving spheres against horizontal ground only**. General CCD, constraints/joints, and sleeping are not implemented. The WebGL2 viewer draws recorded simulation traces on the GPU, but is not a live coupled C++ engine renderer or a custom Vulkan/Metal/DirectX backend. **Warm starting and exact segment-edge witnesses are experimental opt-ins, not stable improvements.** Benchmarks are one Linux-container observation, not universal performance claims.

## Quick start (CMake, C++20; no third-party runtime dependencies)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# Produce a simulation trace.
./build/rigidstep demo --scene rain --bodies 64 --frames 360 \
  --seed 11 --broadphase sap --out frames.jsonl
python3 scripts/verify_trace.py frames.jsonl

# Compare two broadphases on the *same physical simulation*.
./build/rigidstep benchmark --scene sparse --bodies 800 --frames 120 \
  --repeats 7 --seed 11 --out benchmark.csv
python3 scripts/benchmark_report.py benchmark.csv

# Run a predeclared 3-scene × 3-seed paired performance campaign.
python3 scripts/run_campaign.py --exe build/rigidstep --output lab-output/campaign

# Reproducible contact-debug trace (recorded, not live rendering)
./build/rigidstep demo --scene stack --bodies 36 --frames 180 \
  --edge-contact on --contact-overlay on --out contact-debug.jsonl
python3 scripts/verify_trace.py contact-debug.jsonl
```

On Windows / MSVC, the executable normally lives under `build/Release/`. All core physics is standard C++20; Linux GCC and AddressSanitizer+UBSan were run locally, while Windows cross-platform validation is run in GitHub Actions (see the live workflow status above).

## WebGL2 GPU trace viewer (V0.4)

Run `python3 -m http.server 8000` from the repository root and visit **http://localhost:8000/viewer/**. The default viewer now uses **actual WebGL2 GPU line rasterization** (not the old Canvas 2D projection) and automatically loads a checked-in C++ trace (`viewer/demo.jsonl`). Open your own `frames.jsonl` via **Open JSONL trace**. Drag to orbit, scroll to zoom, scrub the timeline or toggle colored contact-normal overlays. If WebGL2 is unavailable, use the retained [Canvas fallback](viewer/canvas.html).

**Boundary:** The GPU renders a recorded C++ simulation, not a physics solver running in the browser, a live coupled 3D game engine, or a Vulkan/Metal/DirectX renderer. JavaScript controls playback and builds line segments; GLSL transforms and rasterizes vertices. A Node.js geometry/trace test suite checks mathematical and input invariants; actual WebGL browser visual QA requires a supported browser.


## What is implemented

| Subsystem | Implementation | How it is validated |
|---|---|---|
| 3D math | Vec3, quaternion rotation, body-space inertia tensors | Rotation/inverse-rotation and free-flight tests |
| Body geometry | Dynamic spheres, oriented boxes, infinite horizontal static planes | Sphere/sphere, sphere/OBB, OBB/OBB, ground scenarios |
| Broadphase | Sweep-and-Prune (SAP) compared with O(n²) brute-force reference | 20 randomized 90-body pair-set oracles; 3×180-tick trajectory parity |
| Narrowphase | Sphere/sphere, sphere/OBB, 15-axis OBB SAT; face clipping up to 4 contacts, optional edge-segment witness | Face manifold, 800 rotated-box stress samples, segment-sampling oracle |
| Dynamics | Fixed-step Euler, accumulated normal/friction impulses, angular motion; limited sphere-plane TOI; **experimental half-impulse warm start** | Momentum, bounce, negative/positive CCD controls, deterministic cache replay and on/off ablation |
| Determinism | Seeded scenes, stable contact ordering, FNV-based state/config hashes (excluding broadphase mode) | Tick-by-tick seeded replays; algorithm parity |
| Diagnostics & GPU | JSONL frame dump, labeled contact normals, CSV step times, dependency-free WebGL2 GPU trace viewer | Strict trace validator, JS geometry tests, paired hash gate; browser visual validation separately required |
| Quality | CTest, GCC/Clang warnings, ASan+UBSan, Linux/Windows CI matrix, independent reference program | Legacy and analytic C++ tests, Node.js geometry tests; inspect live CI for final status |

## V0.2: face manifold + narrowly scoped continuous collision handling

- A pair of overlapping axis-aligned boxes now generates a four-point contact patch rather than one midpoint. Face-feature OBB overlaps clip the incident face against four reference side planes (Sutherland–Hodgman); edge features still use one approximate contact. Deterministic point reduction is capped at 4.
- `--sphere-plane-ccd on` enables a **limited** sphere sweep against the highest infinite horizontal plane. A 400 m/s negative control visibly crosses the ground with discrete collision alone; the optional sweep prevents penetration and reflects velocity in this controlled example. This is not general box/mesh/sphere CCD or a robust all-object event scheduler.
- Expanded input-finiteness checks and 15 local test suites, including rotated contact stress, high-speed negative/positive controls and CCD replay.
- The [round-2 numerical audit](docs/ROUND2_VALIDATION.md) records the testing scope and observed performance trade-offs.
- A separate **single-point vs four-point** [stack stability ablation](docs/MANIFOLD_ABLATION.md), across three seeds and three scene sizes, returned **mixed** results. Face clipping improved one tested large-stack motion proxy, but did not consistently improve kinetic energy, center-of-mass variability or floor penetration in smaller stacks. No general stability/accuracy improvement is claimed.

## V0.3: experimental persistent contacts, edge-segment witnesses, strict audit

- **Warm-start cache** uses per-contact body-local anchors and normal similarity, matched one-to-one, with a half-impulse initialization and friction re-projection. It is automatically evicted when contacts disappear, manually invalidated on external body mutations and included in the replay hash. It is **off by default** because the warm-start-on/off results are mixed and a predeclared disjoint holdout did not establish broad stability improvement.
- **Opt-in segment edge witnesses** replace the old support-vertex midpoint on SAT cross-axis contacts with the midpoint of two closest support-edge segment witnesses, handling parallel/degenerate cases. **Off by default** while accuracy remains unvalidated against an independent physics engine.
- **Read-only `collision_contacts()`** supports geometry debugging; a `--contact-overlay on` trace includes normalized contact vectors, feature classes and points. The HTML viewer displays these recorded vectors, but **is not a real-time GPU renderer**.
- **Adversarial correctness audit:** a fully submerged box previously emitted up to eight ground constraints. It now emits at most four; this fixes a documented mismatch, not necessarily general stability.
- **Cold behavior regression:** six scene/seed cases × 181 frames matched the archived V0.2 body's exact serialized state and step diagnostics when new options are off. Source and data provenance are recorded.
- **Multi-variant ablation:** 36 complete warm-start pairs across training and independently chosen holdout seeds/sizes, plus 12 edge-witness pairs (only six had actual edge features; the other six were proven exactly trajectory-invariant). The outcomes are mixed; **no blanket physical-accuracy or performance improvement is claimed**.

**Commands:** `--warm-start on|off`, `--edge-contact on|off`, and `--contact-overlay on|off` (default off). Details, re-run commands and counterexamples: [V0.3 Audit](docs/V03_AUDIT.md). All raw summary CSVs and source/binary SHA-256 identities are in `evidence/`.

## V0.4: independent analytic oracles and WebGL2 rendering

- Added **independent reference tests** for semi-implicit ballistic motion, closed-form unequal-mass elastic collisions, eight-corner rotated-box AABBs, and sphere-vs-OBB closest-point classification. These check the same outputs against independent mathematics instead of merely comparing two solver implementations. See [method and limitations](docs/V04_REFERENCE_AND_GPU.md).
- Added an actual **WebGL2 GPU-rendered** trajectory debugger with perspective camera, quaternion box transformations, sphere wireframes, contact-normal vectors, file import and scrub/playback controls. The previous projection-only viewer is retained as `viewer/canvas.html`.
- Updated CI to compile/run the new C++ reference executable on Linux, Windows, and under ASan/UBSan, and to run Node.js trace geometry tests on Linux. This does **not** claim comparative accuracy against Bullet/Jolt or production graphics throughput.

```bash
ctest --test-dir build --output-on-failure
node --test tests/test_viewer.mjs
python3 -m http.server 8000
# In a browser: http://localhost:8000/viewer/
```

## Measured performance — no cherry-picking

Repeated same-seed paired measurements on one AMD EPYC 9V74 Linux container (GCC 14.2, Release C++20). `brute/sap` >1 means SAP was faster. **Both broadphases produced identical final state hashes in all reported pairs.** Each full simulation step is timed, not only the collision broadphase. Paired order alternates across repeats; this is still a small, non-randomized, single-environment experiment.

| Scenario | Bodies | Frames per run | Paired repeats | Median brute/SAP ratio | Pair range | Outcome |
|---|---:|---:|---:|---:|---:|---|
| `sparse` | 800 | 120 | 7 | **1.593×** | 1.468–1.635× | SAP helped on sparse scene |
| `stack` | 120 | 120 | 7 | **0.954×** | 0.831–1.031× | SAP did **not** help on densely interacting scene |

Raw records: [`evidence/local-sparse-800.csv`](evidence/local-sparse-800.csv), [`evidence/local-stack-120.csv`](evidence/local-stack-120.csv). Re-run the independent checker via `python3 scripts/benchmark_report.py evidence/local-sparse-800.csv`.

**Do not conclude:** a general 59% engine speedup, stable tail-latency gain, a validated physical real-world model, CPU cycle-level attribution, or performance portability to desktop/mobile consoles. `sparse`/`stack` use synthetic procedural scenes; same-seed repeats are *not* independent scene samples. The dense case is a measured counterexample to a universal win.

### New V0.2 multi-seed campaign (one host)

The default campaign exercises **rain (120 bodies), sparse (300), and stack (100)** for 3 preselected seeds × 4 paired repetitions (12 paired trials per scene). Every SAP/Brute comparison passed the final-state hash gate.

| Scene | Paired median brute/SAP | Pair range | Interpretation |
|---|---:|---:|---|
| `sparse-300` | **1.201×** | 1.173–1.342× | Benefit for this sparse synthetic workload |
| `rain-120` | **1.058×** | 1.021–1.147× | Small benefit under these settings |
| `stack-100` | **0.998×** | 0.980–1.056× | No meaningful advantage; performance ties/noise |

See [`evidence/campaign-20261125/trials.csv`](evidence/campaign-20261125/trials.csv) and [`metadata.json`](evidence/campaign-20261125/metadata.json). This campaign was **not** CPU-affinity or frequency controlled. Older seven-pair numbers above refer to a previous implementation and cannot be interpreted as a controlled before/after speedup.

## Repository map

```
include/rigidstep/physics.hpp  -- Public C++ physics data types and World API
src/physics.cpp              -- Quaternion math, broad/narrow collision, solver
apps/rigidstep.cpp           -- Demo/benchmark CLI and JSONL/CSV export
tests/test_physics.cpp       -- Invariants, randomized oracle, trajectory parity
scripts/verify_trace.py     -- Reject invalid trace structure
scripts/benchmark_report.py -- Reject incomparable/mismatched benchmark pairs
viewer/                     -- Standalone offline trace viewer + small sample
evidence/                   -- Raw, local benchmark observations and cross-hash audit
docs/                       -- Numerical method, limitations and next milestones
.github/workflows/ci.yml    -- Linux, sanitizer and Windows build/test jobs
```

## Build and quality gates

`cmake --build build --parallel && ctest --test-dir build --output-on-failure` must pass before presenting a change as validated. GCC/Clang ASan+UBSan can be enabled with `-DRIGIDSTEP_SANITIZERS=ON`. CI also tests a Windows/MSVC build; a configured job is not evidence of a successful hosted run.

The browser-based viewer is an offline **trace viewer**, not a runtime native GPU renderer. Interactive visual inspection and truly independent physics-engine comparison remain outstanding.

## Further reading

- [V0.3 experimental solver/geometry audit, holdout controls and provenance](docs/V03_AUDIT.md)
- [Architecture, governing equations and solver compromises](docs/ARCHITECTURE.md)
- [Benchmark protocol, data integrity and open caveats](docs/BENCHMARK_METHOD.md)
- [Mixed-results face-manifold ablation](docs/MANIFOLD_ABLATION.md)
- [Engineering roadmap for engine-level competitiveness](docs/ROADMAP.md)

## License

MIT. No proprietary Roblox, employer, or user content is included. The project is independent of Roblox.
