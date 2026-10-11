"""Parametric CAD of the tepee mechanism (isosceles slider-crank, r2 = r3) in Python with CadQuery.

Run from the repo root:  python cad/tepee/build_tepee.py
Writes STEP + STL of each part and of the assembly to cad/tepee/step and cad/tepee/stl, and
results/tepee_cad_key_dimensions.json.   Units: mm.

Kinematics (see matlab/ and python/): crank pivot O at the origin, crank pin P = R (cos t, sin t),
slider S = (2 R cos t, 0), coupler S -> P along the direction (180 deg - t).
The course report uses r2 = r3 = 10 (length units, units not stated consistently); here R = 25 mm,
i.e. 2.5 mm per report unit, giving a slider stroke of 4 R = 100 mm.

Layers (z, mm):  base 0-8 | crank 12-18 (boss 18-22) | coupler 22-28 | slider and guide plate 30-42.
The layering lets the coupler and slider pass over the crank pivot when theta = 90 / 270 deg.
"""
import json
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parents[2]
STEP, STL, RES = ROOT / "cad/tepee/step", ROOT / "cad/tepee/stl", ROOT / "results"
for d in (STEP, STL, RES):
    d.mkdir(parents=True, exist_ok=True)

# ---------------- parameters ----------------
R = 25.0                        # r2 = r3 (mm)
SCALE_MM_PER_UNIT = R / 10.0    # report units -> mm
LW, LT = 14.0, 6.0              # link width, thickness
PIN_D, HOLE_D = 6.0, 6.3        # pins and the holes they pass through (0.15 mm radial clearance)
POST_D, POST_HOLE = 8.0, 8.3
BOSS_D = 10.0
BASE_L, BASE_W, BASE_T = 240.0, 100.0, 8.0
Z_POST_TOP = 20.0
Z_CRANK, Z_BOSS_TOP, Z_PIN_TOP = 12.0, 22.0, 28.5
Z_COUP = 22.0
SL_L, SL_W, SL_H, SL_Z = 30.0, 16.0, 12.0, 30.0          # slider block
Z_SPIN_BOT = 21.0
SLOT_CLR = 0.3
GUIDE_HALF_SLOT, GUIDE_END, GUIDE_HALF_W = 70.0, 14.0, 20.0   # slot half length, end-wall thickness, half width
Z_GUIDE_TOP = SL_Z + SL_H

log = {}
def export(wp, name, **meta):
    cq.exporters.export(wp, str(STEP / f"{name}.step"))
    cq.exporters.export(wp, str(STL / f"{name}.stl"), tolerance=0.02, angularTolerance=0.15)
    v = wp.val(); bb = v.BoundingBox()
    log[name] = dict(volume_mm3=round(v.Volume(), 1), bbox_mm=[round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
                     valid=bool(v.isValid()), **meta)
    print(f"{name:16s} V = {v.Volume():9.1f} mm3   bbox = {log[name]['bbox_mm']}   valid = {log[name]['valid']}")

def stadium(z0, thick, length_between_centres, width):
    """Slot-shaped link, hole centres at (0,0) and (L,0), extruded from z0."""
    return (cq.Workplane("XY").workplane(offset=z0).center(length_between_centres / 2, 0)
            .slot2D(length_between_centres + width, width).extrude(thick))

def cyl(d, z0, z1, x=0.0, y=0.0):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(d / 2).extrude(z1 - z0)

# ---------------- base with pivot post ----------------
base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
base = base.union(cyl(POST_D, BASE_T, Z_POST_TOP))
for x in (-105, 105):
    for y in (-40, 40):
        base = base.cut(cyl(5.5, 0, BASE_T, x, y))
export(base, "tepee_base", plate=f"{BASE_L}x{BASE_W}x{BASE_T}", pivot_post_d=POST_D, post_top_z=Z_POST_TOP, mounting="4x d5.5 at (+-105, +-40)")

# ---------------- guide bridge (slotted plate on two end walls) ----------------
half_len = GUIDE_HALF_SLOT + GUIDE_END
bridge = cq.Workplane("XY").box(2 * half_len, 2 * GUIDE_HALF_W, Z_GUIDE_TOP - BASE_T, centered=(True, True, False)).translate((0, 0, BASE_T))
bridge = bridge.cut(cq.Workplane("XY").box(2 * GUIDE_HALF_SLOT, 2 * GUIDE_HALF_W, SL_Z - BASE_T, centered=(True, True, False)).translate((0, 0, BASE_T)))   # open underside
slot_w = SL_W + 2 * SLOT_CLR
bridge = bridge.cut(cq.Workplane("XY").box(2 * GUIDE_HALF_SLOT, slot_w, SL_H, centered=(True, True, False)).translate((0, 0, SL_Z)))                     # slider slot
export(bridge, "tepee_guide_bridge", outer=f"{2*half_len:.0f}x{2*GUIDE_HALF_W:.0f}", plate_z=f"{SL_Z}-{Z_GUIDE_TOP}", slot=f"{2*GUIDE_HALF_SLOT:.0f} x {slot_w:.1f}", underside_open_height=SL_Z - BASE_T)

# ---------------- crank (link 2): stadium + boss + pin ----------------
crank = stadium(Z_CRANK, LT, R, LW).cut(cyl(POST_HOLE, Z_CRANK, Z_CRANK + LT))
crank = crank.union(cyl(BOSS_D, Z_CRANK + LT, Z_BOSS_TOP, R, 0)).union(cyl(PIN_D, Z_BOSS_TOP, Z_PIN_TOP, R, 0))
export(crank, "tepee_crank", length_between_centres=R, width=LW, thickness=LT, post_hole_d=POST_HOLE, pin_d=PIN_D, boss_d=BOSS_D)

# ---------------- coupler (link 3): stadium with two holes, S end at the origin ----------------
coupler = stadium(Z_COUP, LT, R, LW).cut(cyl(HOLE_D, Z_COUP, Z_COUP + LT)).cut(cyl(HOLE_D, Z_COUP, Z_COUP + LT, R, 0))
export(coupler, "tepee_coupler", length_between_centres=R, width=LW, thickness=LT, hole_d=HOLE_D, z_range=f"{Z_COUP}-{Z_COUP+LT}")

# ---------------- slider: block + downward pin ----------------
slider = cq.Workplane("XY").box(SL_L, SL_W, SL_H, centered=(True, True, False)).translate((0, 0, SL_Z)).union(cyl(PIN_D, Z_SPIN_BOT, SL_Z))
export(slider, "tepee_slider", block=f"{SL_L}x{SL_W}x{SL_H}", pin_d=PIN_D, pin_z=f"{Z_SPIN_BOT}-{SL_Z}")

# ---------------- assembly at theta = 60 deg ----------------
import math
def poses(theta_deg):
    t = math.radians(theta_deg); sx = 2 * R * math.cos(t)
    return dict(crank=lambda w: w.rotate((0, 0, 0), (0, 0, 1), theta_deg),
                coupler=lambda w: w.rotate((0, 0, 0), (0, 0, 1), 180 - theta_deg).translate((sx, 0, 0)),
                slider=lambda w: w.translate((sx, 0, 0)))
if __name__ == "__main__":
    P = poses(60)
    asm = cq.Assembly(name="tepee")
    asm.add(base, name="base", color=cq.Color("gray")); asm.add(bridge, name="guide_bridge", color=cq.Color("lightgray"))
    asm.add(P["crank"](crank), name="crank", color=cq.Color("steelblue")); asm.add(P["coupler"](coupler), name="coupler", color=cq.Color("firebrick"))
    asm.add(P["slider"](slider), name="slider", color=cq.Color("seagreen"))
    asm.save(str(STEP / "tepee_assembly_theta60.step"))
    params = dict(R=R, SCALE_MM_PER_UNIT=SCALE_MM_PER_UNIT, STROKE=4 * R, LW=LW, LT=LT, PIN_D=PIN_D, HOLE_D=HOLE_D, POST_D=POST_D, POST_HOLE=POST_HOLE,
                  BOSS_D=BOSS_D, BASE_L=BASE_L, BASE_W=BASE_W, BASE_T=BASE_T, Z_POST_TOP=Z_POST_TOP, Z_CRANK=Z_CRANK, Z_BOSS_TOP=Z_BOSS_TOP,
                  Z_PIN_TOP=Z_PIN_TOP, Z_COUP=Z_COUP, SL_L=SL_L, SL_W=SL_W, SL_H=SL_H, SL_Z=SL_Z, Z_SPIN_BOT=Z_SPIN_BOT, SLOT_CLR=SLOT_CLR,
                  GUIDE_HALF_SLOT=GUIDE_HALF_SLOT, GUIDE_END=GUIDE_END, GUIDE_HALF_W=GUIDE_HALF_W)
    json.dump(dict(parameters=params, parts=log), open(RES / "tepee_cad_key_dimensions.json", "w"), indent=1)
    print("assembly saved")
