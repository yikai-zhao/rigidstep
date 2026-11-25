# V0.3 solver and geometry audit (2026-11-25)

**Status: experimental additions, not an accuracy or performance victory.**

This report uses a set of predeclared, synthetic meter-scale workloads and explicit on/off controls. Its primary contribution is exposing two tempting but unproven physics changes to falsifiable tests rather than shipping them enabled by default.

## 1. Four audit gates

1. **Correctness / geometry.** The new `closest_segment_points` handles interior, endpoint, parallel and degenerate segments. Tests include 600 seeded segment-swap / range invariants, 100 independent sampled-grid lower-bound checks and 800 seeded rotated-box arrangements that hit both SAT face and edge branches. A new `collision_contacts()` API returns read-only, labeled contact points and normals for inspector tools. One concrete bug was corrected: a deeply immersed oriented box could emit eight ground vertex contacts; the constraint count is now capped to four per box/plane pair.
2. **Default-path regression.** The old V0.2 solver file is archived as `benchmarks/legacy-v02/physics.cpp`. With **both new experiments off**, six scene/seed combinations match the archived V0.2 solver in *every physical body state and pre-solve energy/contact metric at every one of 181 ticks* (1,086 state pairs). The new solver hash intentionally differs because it includes additional experimental settings and cache state. This is same-host and same-compiler behavior preservation, **not** a ground-truth physics oracle.
3. **Warm-start on/off.** Run 36 complete pairs (72 trajectories): the original fixed-seed panel and a separate set of **disjoint holdout seeds and sizes**, both with 2/4 impulse iterations. Every pair shares initial conditions; all traces are checked for missing frames, non-finite state, cache-count impossibilities and actual warm impulse reuse. The metrics are tail kinetic energy, center-of-mass vertical standard deviation, and floor penetration. Results are **mixed**: some stacks benefit while others become more active or less stable. No universal stabilization is claimed. Warm Starting remains `off` by default.
4. **Edge witness on/off.** Run 12 complete pairs with the same seed and scene. **Only six pairs were eligible**, because the other six did not exercise a SAT cross-axis edge contact at all. All six non-eligible controls matched **every serialized physical-body frame** between the settings; comparing the engine state hash itself would be invalid because it intentionally includes the edge setting flag. In the eligible pairs there are both positive and negative tail center-height variance differences. This is precisely why `--edge-contact on` is opt-in until an independent reference and an expanded corner-case corpus exist.

## 2. Reproduction (Linux, CMake C++20 and Python standard library)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# Same-compiler archived cold-path regression
c++ -std=c++20 -O2 -DRIGIDSTEP_BUILD_LEGACY=1 -I include apps/rigidstep.cpp \
  benchmarks/legacy-v02/physics.cpp -o build/rigidstep-v02
c++ -std=c++20 -O2 -I include apps/rigidstep.cpp \
  src/physics.cpp -o build/rigidstep-candidate-o2
python3 scripts/verify_cold_regression.py \
  --baseline build/rigidstep-v02 --candidate build/rigidstep-candidate-o2 \
  --output lab-output/cold-regression.json

# Fixed-seed and holdout, randomized alternating on/off order
python3 scripts/compare_warmstart.py --exe build/rigidstep \
  --seeds 3 11 27 --sizes 24 64 120 --iterations 2 4 \
  --frames 300 --tail 100 --output lab-output/warmstart-current
python3 scripts/compare_warmstart.py --exe build/rigidstep \
  --seeds 17 29 53 --sizes 40 80 160 --iterations 2 4 \
  --frames 300 --tail 100 --output lab-output/warmstart-holdout

# Geometry-only opt-in intervention with cold solver in both arms
python3 scripts/compare_edge_contacts.py --exe build/rigidstep \
  --output lab-output/edge-ablation

# Contact-point/normal debugging overlay in the trace viewer
./build/rigidstep demo --scene stack --bodies 36 --frames 120 \
  --edge-contact on --contact-overlay on --out frames.jsonl
python3 scripts/verify_trace.py frames.jsonl
python3 -m http.server 8000
# Navigate to localhost:8000/viewer, then open frames.jsonl
```

A hosted `gcc` / `windows` / `sanitizers` CI result is a separate validation gate. An edited workflow is not proof until an actual GitHub Actions run passes.

## 3. Interpretation and limitations

- The cold-v0.2 parity is a **regression guard**, not validation of V0.2 physical truth. The old and new versions share major solver assumptions and therefore cannot serve as independent numerical or mechanical references.
- Warm-start attempts to match **body-local anchors and contact normals** and reuses only half of the prior impulse. It is not a proven feature-ID tracker; contact feature switching and approximate position correction still produce instability.
- The read-only contact snapshot is calculated at the *end-of-frame geometry*. Its count can differ from the **pre-solve** contact count in the same trace line; they intentionally have different labels.
- An edge contact feature is detected through SAT, but the optional closest-segment result is not proof of correct real-world collision response. Differences in proxy metrics may come from solver artifacts, not better physics.
- Timings were not included in this round's physical-accuracy comparisons. There is no proof of frame-time speedup or hardware portability. Benchmarks do not run GPU rendering.
- A ground plane is infinite and horizontal; there are no general CCD, joints, sleeping islands, mesh collision or self-balancing articulated bodies.

## 4. Priority next milestones

**P0 — independent physics differential oracle:** reproduce controlled sphere/box and box/box incidents against Bullet or Jolt with matched masses, poses, gravity, friction, time stepping and documented solver differences. Falsify our solver via counterexamples. Add analytically soluble conservation and geometry tests before claiming any improved accuracy.

**P1 — live GPU debug viewer:** a small OpenGL/WebGPU viewport driven by the C++ physics loop with frame-time histogram, collision normals and optional broadphase AABBs. The current HTML viewer is replay-only 2D-canvas projection and should never be described as GPU rendering.

**P2 — robust constraint systems:** feature-ID tracking for stable warm starts, sleeping/islands, verified substep/TOI CCD (not just horizontal-plane sphere sweeps), distance joints and articulated-chain tests. Performance optimization must follow an explicit correctness gate.
