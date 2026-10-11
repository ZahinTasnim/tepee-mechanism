"""Parametric CAD of a Scotch-yoke mechanism, written in Python with CadQuery (OpenCASCADE).

Run from the repo root:  python cad/scotch_yoke_reference/build_scotch_yoke.py
Writes STEP + STL for each part and the assembly (crank angle 0) to cad/step and cad/stl,
and results/scotch_yoke_cad_key_dimensions.json (volume, bounding box, parameters of every part).
Units: mm.
"""
import json
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parents[2]
STEP, STL, RES = ROOT / "cad/scotch_yoke_reference/step", ROOT / "cad/scotch_yoke_reference/stl", ROOT / "results"
for d in (STEP, STL, RES):
    d.mkdir(parents=True, exist_ok=True)

# ---------------- design parameters ----------------
R_CRANK = 20.0          # crank radius -> stroke = 2R = 40 mm
PIN_D   = 8.0           # crank pin diameter
CLR     = 0.3           # pin-to-slot clearance per side
BASE_L, BASE_W, BASE_T = 200.0, 120.0, 8.0
SHAFT_D, SHAFT_BORE = 12.0, 12.2
DISC_D, DISC_T = 64.0, 10.0
PIN_LEN = 14.0
ROD_D, GUIDE_BORE = 10.0, 10.2
GUIDE_X, GUIDE_L, GUIDE_W, GUIDE_TOP = 72.0, 26.0, 30.0, 15.0   # block top = axis + GUIDE_TOP
AXIS_Z = 24.0           # height of the slide axis above the frame underside (rod bottom = 19 mm, 1 mm above the crank disc)
YOKE_T, YOKE_W, ROD_LEN = 8.0, 70.0, 80.0
Z_CRANK = BASE_T                    # crank disc sits on the frame
Z_YOKE = AXIS_Z - YOKE_T / 2        # yoke plate underside so that rod axis = AXIS_Z (19 mm)

log = {}

def export(wp, name, **meta):
    cq.exporters.export(wp, str(STEP / f"{name}.step"))
    cq.exporters.export(wp, str(STL / f"{name}.stl"), tolerance=0.02, angularTolerance=0.15)
    v = wp.val(); bb = v.BoundingBox()
    log[name] = dict(volume_mm3=round(v.Volume(), 1), bbox_mm=[round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
                     valid=bool(v.isValid()), **meta)
    print(f"{name:22s} V = {v.Volume():10.1f} mm3   bbox = {log[name]['bbox_mm']}   valid = {log[name]['valid']}")

# ---------------- frame ----------------
base = (cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
        .faces(">Z").workplane().hole(SHAFT_BORE)
        .faces(">Z").workplane().pushPoints([(-85, -50), (85, -50), (-85, 50), (85, 50)]).cboreHole(6.6, 11, 6.5))
frame = base
for sx in (-GUIDE_X, GUIDE_X):
    block = (cq.Workplane("XY").box(GUIDE_L, GUIDE_W, AXIS_Z + GUIDE_TOP - BASE_T, centered=(True, True, False))
             .translate((sx, 0, BASE_T)))               # stands on the frame, bore axis at AXIS_Z
    bore = cq.Workplane("YZ").center(0, AXIS_Z).circle(GUIDE_BORE / 2).extrude(GUIDE_L).translate((sx - GUIDE_L / 2, 0, 0))
    frame = frame.union(block.cut(bore))
export(frame, "scotch_yoke_frame", base=f"{BASE_L}x{BASE_W}x{BASE_T}", shaft_bore=SHAFT_BORE, guide_bore=GUIDE_BORE,
       slide_axis_height=AXIS_Z, mounting="4x M6 counterbored at (+-85, +-50)")

# ---------------- crank (disc + pin) ----------------
crank = (cq.Workplane("XY").circle(DISC_D / 2).extrude(DISC_T)
         .faces(">Z").workplane().hole(SHAFT_D)
         .faces(">Z").workplane().center(R_CRANK, 0).circle(PIN_D / 2).extrude(PIN_LEN))
export(crank, "scotch_yoke_crank", disc_d=DISC_D, thickness=DISC_T, shaft_bore=SHAFT_D, crank_radius=R_CRANK,
       pin_d=PIN_D, pin_length=PIN_LEN)

# ---------------- yoke (slotted plate + two guide rods) ----------------
SLOT_W = PIN_D + 2 * CLR
PLATE_L = 2 * R_CRANK + PIN_D + 2 * CLR + 16        # 64.6 mm
SLOT_L = YOKE_W - 16                                # overall slot length 54 mm (full-round ends)
yoke = (cq.Workplane("XY").rect(PLATE_L, YOKE_W).extrude(YOKE_T)
        .faces(">Z").workplane().slot2D(SLOT_L, SLOT_W, angle=90).cutThruAll())
rod_r = cq.Workplane("YZ").center(0, YOKE_T / 2).circle(ROD_D / 2).extrude(ROD_LEN).translate((PLATE_L / 2, 0, 0))
rod_l = cq.Workplane("YZ").center(0, YOKE_T / 2).circle(ROD_D / 2).extrude(-ROD_LEN).translate((-PLATE_L / 2, 0, 0))
yoke = yoke.union(rod_r).union(rod_l)
export(yoke, "scotch_yoke_yoke", slot_width=SLOT_W, slot_length=SLOT_L, plate=f"{PLATE_L}x{YOKE_W}x{YOKE_T}", rod_d=ROD_D,
       rod_length_each=ROD_LEN, stroke=2 * R_CRANK)

# ---------------- assembly (crank angle 0: pin at +x, yoke at x = +R) ----------------
asm = cq.Assembly(name="scotch_yoke")
asm.add(frame, name="frame", color=cq.Color("gray"))
asm.add(crank, name="crank", loc=cq.Location(cq.Vector(0, 0, Z_CRANK)), color=cq.Color("orange"))
asm.add(yoke, name="yoke", loc=cq.Location(cq.Vector(R_CRANK, 0, Z_YOKE)), color=cq.Color("steelblue"))
asm.save(str(STEP / "scotch_yoke_assembly.step"))

params = dict(R_CRANK=R_CRANK, STROKE=2 * R_CRANK, PIN_D=PIN_D, CLR=CLR, SLOT_W=SLOT_W, SLOT_L=SLOT_L, PLATE_L=PLATE_L,
              Z_CRANK=Z_CRANK, Z_YOKE=Z_YOKE, AXIS_Z=AXIS_Z, ROD_D=ROD_D, GUIDE_BORE=GUIDE_BORE, GUIDE_X=GUIDE_X, GUIDE_L=GUIDE_L,
              ROD_LEN=ROD_LEN, DISC_D=DISC_D, DISC_T=DISC_T, PIN_LEN=PIN_LEN)
json.dump(dict(parameters=params, parts=log), open(RES / "scotch_yoke_cad_key_dimensions.json", "w"), indent=1)
print("assembly saved")
