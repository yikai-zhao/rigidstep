# Architecture and numerical method

RigidStep is a **fixed-step, 3D** collision-response experiment. It prioritizes inspectability, fully defined interfaces and reproducibility. It does not attempt to implement every feature of a production physics engine.

## State and stepping

The state of a rigid body contains center-of-mass position `x`, quaternion orientation `q`, linear velocity `v`, world angular velocity `ω`, inverse mass, local diagonal inverse inertia, and shape geometry. Local-space box inertia is computed analytically for a uniform-density cuboid; spheres use `I = 2/5 m r²`. World inverse inertia maps a vector with `R I^-1 R^T` using quaternion rotation and conjugation.

Each step is the same `dt = 1/120` seconds by default:

1. Semi-implicit Euler gravity update: `v[t+1] = v[t] + g dt`.
2. If limited sphere-plane CCD is enabled, handle downward sphere/plane time-of-impact within the integrated step; otherwise update position `x += v dt`. Update orientation `q <- normalize(exp(ω dt) q)`.
3. Create AABBs for the current rotated geometry.
4. Compute potentially overlapping body pairs with either Sweep-and-Prune (sort on min X, early exit by max X, then check Y/Z) or brute-force O(n²) overlap checks. The planes are special-case contacts and excluded from finite AABB broadphase.
5. Generate sphere/sphere, sphere/oriented-box or rotated-box/rotated-box SAT contacts and static plane contacts. Box-box SAT checks 3+3 face axes plus 9 cross-product axes; near-zero cross axes are ignored. Face contacts use Sutherland–Hodgman clipping against reference side planes followed by deterministic reduction to four spread points; cross-axis edge contact uses an approximate single midpoint.
6. Solve normal and tangential contact impulses for a fixed number of iterations, applying angular Jacobian terms. Normal impulses are accumulated and projected to `>= 0`; friction impulses are projected to a Coulomb cone approximation `|jt| <= sqrt(μa μb) jn`. A velocity-level penetration bias and small mass-weighted positional correction help avoid sinking.
7. Emit summary diagnostics and a 64-bit hash over physical state, body geometry/materials and settings (excluding broadphase mode) in stable body-ID order.

## Normal impulse denominator

For contact offsets `r_a`, `r_b` and unit normal `n`, the denominator used is

`k = m_a^-1 + m_b^-1 + n · ((I_a^-1 (r_a × n)) × r_a + (I_b^-1 (r_b × n)) × r_b)`.

The normal impulse magnitude is computed from relative contact velocity and an accumulated, nonnegative constraint. Tangential impulses use the same effective-mass mechanism along the instantaneous tangent. Restitution is activated for sufficiently approaching contacts.

This is *not* a mathematically exact complementarity solver, an energy-conserving integrator, or an independently validated robotics simulator. Different world topologies/solvers may differ in accuracy and stability.

## Determinism contract

Identical source, compiler/runtime configuration, initial seed, body order and settings are expected to produce the same per-tick state hashes. Hash equality has actually been checked for deterministic replay and for the two broadphase algorithms on seeded scenes. No guarantee is made for **bitwise reproducibility across compilers, architectures or `-ffast-math` builds**. Collision detection output is sorted by numeric body ID so sorting implementation details do not reorder contacts.

The hash is an integrity aid, not a cryptographic checksum or proof of correct physics. There is no network state replication, rollback, authoritative server reconciliation or joint/constraint solver.

## Known limitations / failure modes

- **Partial CCD only:** opt-in downward sphere sweeps against the highest horizontal plane prevent one well-defined tunneling case. Fast sphere/sphere, sphere/box, box/box and mesh contacts can still tunnel; general multi-contact TOI processing is absent.
- **Partial OBB-OBB manifold:** 15-axis SAT with clipped face contact patches, up to four deterministic points. Edge-edge still uses a midpoint approximation; there is no persistent warm starting. Stacked boxes may still jitter, drift or settle artificially.
- **No sleeping/islands/warm starting:** high resting-contact counts waste CPU; no production scalability claim.
- **Simplified friction and correction:** no torsional/rolling friction, material mixing calibration or angular position correction.
- **No high-performance spatial tree or SIMD:** SAP sorts on every tick and O(n²) brute force is only an oracle/comparator. SAP can be slower for dense/stacked scenes (see measured evidence).
- **One horizontal ground plane type:** not triangle meshes, heightfields or general static polygon soups.
- **Visualization is trace projection:** no Vulkan/DirectX/Metal/OpenGL render backend, shader performance or GPU frame-time profiling.

These restrictions are intentionally explicit to prevent a compact educational research prototype being misrepresented as an industrial game engine.
