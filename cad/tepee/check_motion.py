"""Move the tepee CAD through a full crank revolution and check it.

At every crank angle the crank is rotated by theta, the coupler is placed with its S end at the slider
(2 R cos theta, 0) and rotated to direction (180 deg - theta), and the slider follows S.
Reported over the revolution:
  * interference volume of every pair of parts (must be 0 mm3)
  * smallest gap of every pair (0 = intended contact: bridge on base, crank boss under the coupler)
  * smallest gap between each pin and the coupler hole around it (design: 0.15 mm)

Usage (repo root):  python cad/tepee/check_motion.py     Writes results/tepee_cad_motion_check.json
"""
import itertools, json, math
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parents[2]
D = json.load(open(ROOT / "results/tepee_cad_key_dimensions.json")); P = D["parameters"]; R = P["R"]
imp = lambda n: cq.importers.importStep(str(ROOT / f"cad/tepee/step/{n}.step")).val()
base, bridge, crank, coupler, slider = (imp(n) for n in ("tepee_base", "tepee_guide_bridge", "tepee_crank", "tepee_coupler", "tepee_slider"))
pin_p = cq.Workplane("XY").workplane(offset=P["Z_BOSS_TOP"]).center(R, 0).circle(P["PIN_D"] / 2).extrude(P["Z_PIN_TOP"] - P["Z_BOSS_TOP"]).val()
pin_s = cq.Workplane("XY").workplane(offset=P["Z_SPIN_BOT"]).circle(P["PIN_D"] / 2).extrude(P["SL_Z"] - P["Z_SPIN_BOT"]).val()

names = ["base", "bridge", "crank", "coupler", "slider"]
worst_vol = {f"{a}-{b}": 0.0 for a, b in itertools.combinations(names, 2)}
min_gap = {k: 1e9 for k in worst_vol}
pin_gap = {"crank pin - coupler": 1e9, "slider pin - coupler": 1e9}
step = 15
for deg in range(0, 360, step):
    t = math.radians(deg); sx = 2 * R * math.cos(t)
    c = crank.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), deg)
    cp = coupler.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180 - deg).translate(cq.Vector(sx, 0, 0))
    s = slider.translate(cq.Vector(sx, 0, 0))
    parts = dict(base=base, bridge=bridge, crank=c, coupler=cp, slider=s)
    for a, b in itertools.combinations(names, 2):
        k = f"{a}-{b}"
        worst_vol[k] = max(worst_vol[k], parts[a].intersect(parts[b]).Volume())
        min_gap[k] = min(min_gap[k], parts[a].distance(parts[b]))
    pin_gap["crank pin - coupler"] = min(pin_gap["crank pin - coupler"], pin_p.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), deg).distance(cp))
    pin_gap["slider pin - coupler"] = min(pin_gap["slider pin - coupler"], pin_s.translate(cq.Vector(sx, 0, 0)).distance(cp))
    print(f"  {deg:3d} deg", flush=True)
out = dict(angles_checked_deg=list(range(0, 360, step)), max_interference_mm3={k: round(v, 4) for k, v in worst_vol.items()},
           min_gap_mm={k: round(v, 3) for k, v in min_gap.items()}, pin_to_hole_gap_mm={k: round(v, 3) for k, v in pin_gap.items()},
           slider_stroke_mm=4 * R)
json.dump(out, open(ROOT / "results/tepee_cad_motion_check.json", "w"), indent=1)
print(json.dumps({k: v for k, v in out.items() if k != "angles_checked_deg"}, indent=1))
