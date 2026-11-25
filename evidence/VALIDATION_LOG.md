# V0.3 local validation log — 2026-11-25 UTC

All measurements use synthetic meter-scale scenes and standard C++20, not Roblox's proprietary engine.

## Gates actually executed

- Linux GCC 14.2 Release compile + CTest: **PASS**, 21 named C++ suites, 16,511 assertions. (The assertion count can change when new tests are added; reproduce via `build/rigidstep_tests`.)
- Linux Debug ASan+UBSan compile + CTest: **PASS**. A passing run is not evidence of thread safety or data-race freedom.
- Python adversarial JSONL parser: **7 tests PASS**, including NaN contact, zero normal, impossible edge count, invalid cache count, bad feature and noncontiguous tick.
- JS viewer's embedded code: **Node `--check` PASS**. Interactive browser UI still needs end-to-end manual/browser validation.
- Archived V0.2 same-compiler cold regression: **6 cases × 181 physical-state frames (1,086 matched)** exactly equal, while new settings off. Different state-hash schemas are intentionally not compared.
- Read-only collision-geometry query: stochastic rotated-box SAT edge and face feature coverage; 600 randomized segment symmetry/range tests plus 100 sampled-grid lower-bound checks. Deep-plane box contact cap tested.
- Warm-start panel and disjoint-seed/size holdout: **36 complete pairs / 72 trajectories**, 300 frames each, tail 100. Warm start improved the center-height-variance metric in **11/36 pairs**, worsened it in **25/36**; other kinetic/penetration outcomes are mixed. **Keep off by default.**
- SAT edge-witness ablation: **12 complete pairs**, but **only 6 physically eligible pairs** saw edge features; center-height variability improved in **3/6** and worsened in **3/6**. **Keep off by default.**
- 121-frame, 14-body bundled contact-overlay sample passes the strict structural trace validator.

## Artifacts and provenance

- `evidence/warmstart-v03-panel/` and `evidence/warmstart-v03-holdout/`: raw CSV results/pairs and metadata with exact source/header/binary/data SHA-256.
- `evidence/edge-ablation-v03/`: all trial-level data, eligibility and counterexample labels. Noneligible trials are **retained** rather than silently dropped.
- `evidence/cold-v02-postedge-final.json`: per-scene regression declarations, baseline and candidate source SHA.
- `evidence/campaign-20261125/` and earlier V0.2 artifacts: **historical**, different source revisions; do not mix their timing with V0.3.

### Still not established

Independent Bullet/Jolt physical accuracy, precise cross-platform floating point bitwise equality, native live GPU rendering, production physics performance, general CCD, joints, stability at large scale, or actual browser interaction testing. See `docs/V03_AUDIT.md` for limitations and reproduction commands.
