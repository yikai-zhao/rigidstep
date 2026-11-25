# Roadmap: what would make this an engine-quality project?

This list is prioritized by **value of verified engineering evidence**, not number of features. V0.2 now implements limited face clipping and sphere/horizontal-plane TOI, with gaps below still prioritized.

## P0: Correctness before speed claims

1. **Persistent manifolds and edge-edge geometry.** Face clipping (up to 4 contacts) is implemented; still add edge-edge closest-point computation, persistent feature IDs, warm-start caches, resting-stack penetration/drift comparisons and stronger conservation tests.
2. **General CCD.** Limited sphere-horizontal-plane TOI is implemented with a matching negative control; still add sphere/box and sphere/sphere sweeps, rotating-box cases, overlapping TOI event ordering, and robust multi-impact substepping.
3. **Differential oracle.** Compare a controlled subset against an independently sourced reference solver for collisions, conservation, and corner cases; document coordinate and integrator differences. An internal SAP-vs-brute check is *not* independent physics validation.

## P1: Data-driven performance engineering

4. **Temporal coherence broadphase.** Incremental SAP/insertion-sort vs full re-sort; measure swap count, candidate-pair misses, dense/degenerate cases and full-step budget.
5. **Persistent warm starting and sleep/island systems.** Measure frames-per-second and resting contacts on several procedural scene families, not one cherry-picked seed.
6. **Broad stress matrix.** Sizes 50→5,000, low/high density, dynamic/static ratios, multiple random seeds, fixed hardware and microprofile flamegraphs. Include a no-improvement result when applicable.

## P2: Better visual/engine credibility

7. **Genuine real-time renderer.** Prefer a constrained OpenGL or WebGPU interactive native/portable front-end; visualize contact normals, broadphase boxes and time-step budget. Do not call the current trace viewer GPU rendering.
8. **Constrained dynamics.** Add distance joints, simple articulated links and a joint solver with stability/constraint-error metrics.
9. **Portable CI and release packaging.** Confirm Windows + Linux builds, compare floating-point drift across platforms, capture artifacts and release a one-command demo.

**Definition of completion:** tests pass, representative raw benchmarks and failure cases are committed, code version and machine are recorded, repeatability is documented, and no headline claims exceed measurement scope.
