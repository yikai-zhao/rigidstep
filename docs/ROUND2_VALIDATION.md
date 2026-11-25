# V0.2 local numerical and performance audit — 2026-11-25 UTC

## Scope

This is a reproducible **local development** result, not a production-game-engine benchmark. The baseline V0.1 had one support-point OBB/OBB contact. V0.2 adds SAT face clipping to up to four points, plus **opt-in** downward sphere sweep against the highest horizontal ground plane. The solver, penetration bias, restitution, and friction remain approximate. The new contact patch has not yet been validated against an independent reference physics engine or compared quantitatively for long-term stack stability.

## Positive/negative tests

- An overlapping axis-aligned box/box face yields **four** solver contacts; an oblique oriented-box pair yields **one to four** contacts per tested configuration.
- **High-speed sphere-plane positive/negative pair:** sphere starts at y=2m with r=.25m, vertical speed -400m/s, no gravity, dt=1/60s, restitution .8. Without CCD the one-tick discrete solver leaves the sphere below floor height+r (negative control). With optional sphere-plane CCD, the sphere finishes above floor and has upward velocity. This is **not** proof of general continuous collision handling.
- Repeated seeded rotated-box collisions, 200-tick CCD runs, replay parity, randomized broadphase pair oracle, momentum test, and long-horizon floor tests pass locally.
- `ctest` runs a single executable containing 15 explicitly named logical test suites; the suite count is not 15 independently built CTest targets.
- GCC 14.2 Release and Debug ASan+UBSan builds and CTest passed locally. Windows CI configured but not verified in this record.

## Paired synthetic full-step performance

The [command](../scripts/run_campaign.py) predeclares rain(120), sparse(300), stack(100), seeds 3/11/27 and four pairs/seed. Each run times 120 complete simulation steps. All **36 pairs** contain two modes with identical final-state hash. `brute/sap` >1 favours SAP.

| Workload | Median brute/SAP | Min .. Max | n |
| --- | ---: | ---: | ---: |
| rain-120 | 1.0578 | 1.0211 .. 1.1466 | 12 |
| sparse-300 | 1.2013 | 1.1727 .. 1.3418 | 12 |
| stack-100 | 0.9978 | 0.9803 .. 1.0562 | 12 |

Raw records: [`trials.csv`](../evidence/campaign-20261125/trials.csv); binary provenance / SHA-256: [`metadata.json`](../evidence/campaign-20261125/metadata.json). **Caveats:** single AMD EPYC 9V74 Linux container, GCC 14.2, synthetic scenes, fixed source and seeds, nonrandomized paired ordering, no pinned frequency/core/thermal control. Ratios are workload-specific, not statistically validated platform performance improvements. Earlier V0.1 measurements were made with different code and are not suitable for direct before/after speedup estimates.

## Non-achievements and next evidence gates

No independent Bullet/Jolt physics correctness oracle, persistent warm starting, dynamic constraint islands, mesh collisions, GPU renderer or high-fidelity scene footage. Future work must quantify contact drift, momentum/energy error and penetration against an external oracle before claiming superior stability. The trace viewer needs independent browser interaction verification.
