# Scotch-yoke reference parts (comparison mechanism)

A pin-in-slot Scotch yoke whose yoke follows x = R cos θ with R = 20 mm (stroke 40 mm) — the same slider motion as the tepee
linkage with r₂ = r₃ = 10 (r₁ = 2 r₂ cos θ₂ = 20 cos θ₂). It is included to compare the two mechanisms; it is **not** the mechanism
analysed in the ME 4504 report.

| Part | File | Notes |
|---|---|---|
| Frame | `step/scotch_yoke_frame.step`, `stl/…` | 200 × 120 × 8 mm base, Ø12.2 shaft bore, two guide blocks with Ø10.2 bores (axis 24 mm above the underside) |
| Crank | `step/scotch_yoke_crank.step`, `stl/…` | Ø64 × 10 mm disc, Ø12 bore, Ø8 pin at R = 20 mm |
| Yoke | `step/scotch_yoke_yoke.step`, `stl/…` | 64.6 × 70 × 8 mm plate with an 8.6 mm slot, two Ø10 × 80 mm rods |
| Assembly | `step/scotch_yoke_assembly.step` | crank angle 0 |

Generated with CadQuery by `build_scotch_yoke.py` (every dimension is a parameter). Over a full revolution the assembly has no interference
(`results/scotch_yoke_cad_motion_check.json`: 0.3 mm pin-to-slot gap, 0.1 mm rod-to-bore gap).
Dimensions for rebuilding it in SolidWorks are in [`docs/scotch_yoke_solidworks_guide.md`](../../docs/scotch_yoke_solidworks_guide.md).
