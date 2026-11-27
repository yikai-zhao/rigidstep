# V0.4 local validation (pre-remote CI)

Environment: Linux x86_64 container, GCC 14.2.0, Node 22.16.0. No real GPU browser screenshot verification was possible in this environment because Chromium blocked local resource navigation before page load.

Reproduced locally:

- `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`, `cmake --build build -j4`, `ctest --test-dir build --output-on-failure`: **2/2 CTest executables passed**.
- Existing core harness: **16,511 checks in 21 test suites**.
- New independent analytic harness: **3,247 assertions passed** across ballistic motion, elastic central impacts, rotated-box AABB, sphere/OBB closest-point classification.
- `cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DRIGIDSTEP_SANITIZERS=ON`, `cmake --build build-asan -j4`, `ctest --test-dir build-asan --output-on-failure`: **2/2 CTest executables passed**.
- `node --check viewer/gpu.mjs` and `node --test tests/test_viewer.mjs`: **5/5 Node tests passed**; checked-in C++ demo trace parsed, line vertices were finite.
- `python3 scripts/verify_trace.py viewer/demo.jsonl`: **121 consecutive frames, 14 consistent bodies**, no warm-start impulses expected.
- Archived V0.2 versus candidate core: **6 cases x 181 frames, exact body-state equality**, compiling both at `-O2`. This checks preservation of the existing cold-start physical path, not the correctness of new WebGL rendering.

Source SHA-256 identities:

```text
0695d9990eba5369275c5c261ab8ab57a218cd76248d72351de3e687e645b340  src/physics.cpp
23e9d545c1a3dbc88b52854c74338750a552b6d1540b6c258f2cdc32a22ca0dc  tests/test_analytic_reference.cpp
8922af4cbbc4b2b8fe3f799416afa2f0301e1640f83c6ceec9010693cb6f8a43  viewer/gpu.mjs
e3ed40c73d7828bd3fd57834994b5cdb8456d2e9419e07fb8513ffb1a6f07248  viewer/trace_geometry.mjs
```

Scientific limits: synthetic deterministic scenes only; reference oracles cover selected cases, not general physically correct simulation. We did not build or execute Bullet/Jolt, did not profile GPU frame times, and did not complete visual/browser-level QA. Failed/negative prior Warm Starting results remain visible in V0.3 evidence and default off.
