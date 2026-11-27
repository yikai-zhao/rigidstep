# V0.4 independent oracles and GPU trace debugger

## Engineering claim

This is a C++20 rigid-body **research prototype** that produces recorded JSONL states. The new reference tests compare selected outputs with independently computed mathematical results; the new viewer transforms and rasterizes those recorded states using **WebGL2 vertex and fragment shaders**. These are two separate capabilities. The viewer does **not** simulate physics itself or run a coupled real-time C++ engine loop.

## Independent analytic checks

`tests/test_analytic_reference.cpp` is a separate CTest executable. Its expectations are derived without invoking `World::collision_contacts()` or the solver to compute the predicted result.

| Test | Independent method | Coverage | Limitation |
|---|---|---|---|
| Ballistic dynamics | Closed-form discrete semi-implicit Euler sum `p_n=p0+v0*n*dt+g*dt²*n(n+1)/2` | 30 non-contact seeds, position and velocity x/y/z | Validates integrator under constant gravity, not real-world fluid dynamics |
| Elastic unequal-mass impact | One-dimensional two-body conservation and restitution=1 closed form | Four varied mass/velocity cases, including momentum | Central impacts only; not frictional or oblique contacts |
| Rotated OBB bounds | Transform 8 distinct box corners, take extrema | 400 deterministic random orientations and extents, all six AABB extrema | AABB only, not collision detection correctness for arbitrary mesh topology |
| Sphere/OBB contact | Independently clamp sphere center in box-local coordinates and compare closest distance with radius | 650 seeded positions, both contact and separation cases | Point-to-OBB distance oracle does not validate contact normals/penetration impulse accuracy |

These controlled tests are **additional** to existing cross-algorithm (SAP vs brute-force) trajectory comparisons and archived baseline cold-path regression. They cannot establish equivalence with Bullet or Jolt: neither was built or run in this iteration. No new physical-performance gains are claimed.

## GPU path

- Start a static server from the repository root: `python3 -m http.server 8000`.
- Open `http://localhost:8000/viewer/` (WebGL2) or `http://localhost:8000/viewer/canvas.html` (legacy CPU Canvas projection).
- `viewer/trace_geometry.mjs` validates frame continuity, identity stability, finite body geometry and bounded input size; it builds line vertices for ground grids, OBB edges, sphere great circles and contact normals.
- `viewer/gpu.mjs` creates a **WebGL2** context and a GLSL program, uploads vertex/color buffers, calculates the orbit camera view-projection matrix and executes `gl.drawArrays(gl.LINES, ...)`.
- The viewer is intentionally wireframe. It does not provide PBR, shadows, instancing, occlusion culling, GPU profiling or a Roblox-style content pipeline.

## Auditable local validation

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
node --check viewer/gpu.mjs
node --test tests/test_viewer.mjs
python3 scripts/verify_trace.py viewer/demo.jsonl
```

**Browser limitation:** The available headless Chromium sandbox rejected local URLs before rendering. Thus shader correctness is supported by source inspection and Node geometry checks, **not** by a completed independent browser screenshot or cross-browser GPU compatibility study. Run a graphical WebGL2 browser before claiming visual QA complete.

## Next falsifiable milestones

1. Reference-engine comparison (Bullet or Jolt) on matched initial conditions, contact-event timing, trajectories and penetration error, retaining contradictory cases.
2. A coupled interactive C++ simulation window using an actual graphics API and debug overlays (separate from JSONL replay).
3. Warm-start solver stabilization only if held-out cases show a credible improvement; otherwise remain opt-in.
4. Hardware-controlled frame-time measurements with reproducible full scene complexity, GPU upload cost and profiling.
