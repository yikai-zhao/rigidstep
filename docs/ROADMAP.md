# RigidStep prioritized upgrade roadmap (V0.3)

The goal is a **credible C++ realtime engine engineering portfolio** for graphics/physics/simulation internships. This is a ranking by provable recruiter value, not a feature checklist.

## P0 — external numerical correctness (highest ROI)

**Independent oracle against Bullet/Jolt, plus analytic controls:** fix joint scenario inputs, shapes, inertia, friction/restitution, dt and iterations. Compare contact normals, signed separation, collision outcomes, energy/momentum, penetration drift, and preserve **all** disagreement cases. Normalize coordinate-system and solver differences. Our SAP/brute parity and old/new solver self-comparison are not independent physics evidence.

**Improve solver only when a falsifying case proves the need:** add SAT edge witness feature IDs, stable contact selection, friction tangent basis, split impulse / positional correction as alternatives, and clearly distinguish a model defect from intended soft-contact trade-offs. Maintain opt-in switches and holdouts until consistent results are demonstrated.

## P1 — live renderer and interactive debugging

Build a native or WASM-powered **live GPU 3D viewport** with the C++ simulation loop, preferably a minimal OpenGL path or portable WebGPU. Expose real per-frame CPU physics time, broadphase AABBs, contact normals, depth, frame-step controls, and cross-engine comparison overlays. Current HTML viewer is a 2D Canvas projection of recorded JSONL, *not* real-time GPU rendering. Treat this as the next large visible portfolio improvement after numerical validation.

## P2 — real engine scale / dynamics

- Benchmark dense/sparse/collision-heavy families at 50–5,000 bodies with randomized repetitions, affinity, warm-up, full-step CPU and peak memory. Ensure continuous event correctness and no selectively favorable p99 results.
- Add spatial/temporal broadphase indexing, sleeping and contact islands, and specifically test wake-up races and determinism.
- Design a **general** CCD event scheduler for dynamic spheres and boxes (not just current downward sphere-plane TOI) with a clear negative-tunneling matrix.
- Implement distance joints and one articulated chain with measured constraint drift before broad avatar claims.
- Ensure Linux/Windows multi-compiler CI, sanitizers, reproducibility and a 60-second visual demo. No employer/customer code.

**Done criteria:** code + raw/evidence-gated tests + negative controls + platform build + a short explanation of where the algorithm fails. Avoid claiming production-grade realism or universal speedups.
