# Local validation log — 2026-11-25 UTC

The **current** implementation has been built with local GCC 14.2 in Release and with Debug ASan+UBSan. The single CTest executable containing 15 named test suites passed in both builds. Tests include manifold clipping, randomized OBB stress, high-speed sphere/plane tunneling negative/positive controls, seeded CCD replay and long-horizon stability checks. `python3 scripts/verify_trace.py viewer/demo.jsonl` passed for 121 contiguous frames and 14 bodies.

The predeclared 3-scene × 3-seed × 4-repeat broadphase campaign generated 72 timed simulation runs (36 paired comparisons); all pairs passed final-state hash equality. Raw CSV plus metadata and hashes are checked in under `evidence/campaign-20261125/`. These are **local single-host synthetic measurements** only.

Original V0.1 seven-pair reports are retained as historical observations rather than overwritten. They are different source versions; compare them only as context, not a controlled performance regression/speedup.

**Still unverified:** interactive viewer browser test, hosted Windows/Linux CI status until externally checked, independent collision/solver reference physics, and mobile/console GPU behaviour. Details: [`docs/ROUND2_VALIDATION.md`](../docs/ROUND2_VALIDATION.md).
