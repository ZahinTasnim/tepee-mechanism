"""Render the tepee CAD (figures/04_cad_parts_and_assembly.png) from cad/tepee/stl with a small numpy z-buffer rasteriser.

Usage (repo root):  python cad/tepee/render_cad.py
"""
import json, struct, math
from pathlib import Path
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
R = json.load(open(ROOT / "results/tepee_cad_key_dimensions.json"))["parameters"]["R"]

def read_stl(fn):
    d = open(fn, "rb").read()
    if d[:5] == b"solid" and b"facet" in d[:400]:
        v = [list(map(float, l.split()[1:])) for l in d.decode().splitlines() if l.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", d[80:84])[0]
    a = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)

def pose(T, deg=0.0, shift=(0, 0, 0)):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return T @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T + np.array(shift)

def render(parts, az=-60.0, el=38.0, W=800, H=560, ss=2, margin=0.07):
    T = np.concatenate([t for t, _ in parts]); col = np.concatenate([np.tile(np.array(matplotlib.colors.to_rgb(c)), (len(t), 1)) for t, c in parts])
    a, e = math.radians(az), math.radians(el)
    c_ = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    r_ = np.cross([0, 0, 1], c_); r_ /= np.linalg.norm(r_); u_ = np.cross(c_, r_)
    L = c_ + 0.55 * r_ + 0.7 * u_; L /= np.linalg.norm(L)
    nr = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nr /= np.linalg.norm(nr, axis=1, keepdims=True) + 1e-12
    nr *= np.where((nr @ c_) < 0, -1, 1)[:, None]
    shade = 0.32 + 0.68 * np.clip(nr @ L, 0, 1)
    X, Y, Z = T @ r_, T @ u_, T @ c_
    w, h = W * ss, H * ss
    scale = min((1 - 2 * margin) * w / np.ptp(X), (1 - 2 * margin) * h / np.ptp(Y))
    sx = (X - (X.max() + X.min()) / 2) * scale + w / 2; sy = h / 2 - (Y - (Y.max() + Y.min()) / 2) * scale
    img = np.ones((h, w, 3)); zb = np.full((h, w), -np.inf)
    for k in range(len(T)):
        x0, x1, x2 = sx[k]; y0, y1, y2 = sy[k]
        den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(den) < 1e-9: continue
        xa, xb = int(max(0, np.floor(min(x0, x1, x2)))), int(min(w - 1, np.ceil(max(x0, x1, x2))))
        ya, yb = int(max(0, np.floor(min(y0, y1, y2)))), int(min(h - 1, np.ceil(max(y0, y1, y2))))
        if xb < xa or yb < ya: continue
        gx, gy = np.meshgrid(np.arange(xa, xb + 1) + 0.5, np.arange(ya, yb + 1) + 0.5)
        l0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / den; l1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / den; l2 = 1 - l0 - l1
        m = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        if not m.any(): continue
        z = l0 * Z[k, 0] + l1 * Z[k, 1] + l2 * Z[k, 2]
        sub = zb[ya:yb + 1, xa:xb + 1]; win = m & (z > sub)
        sub[win] = z[win]; img[ya:yb + 1, xa:xb + 1][win] = col[k] * shade[k]
    return img.reshape(H, ss, W, ss, 3).mean(axis=(1, 3))

stl = {n: read_stl(ROOT / f"cad/tepee/stl/tepee_{n}.stl") for n in ("base", "guide_bridge", "crank", "coupler", "slider")}
COL = dict(base="#7f8c8d", guide_bridge="#b0b7bb", crank="#2c6fbb", coupler="#c0392b", slider="#2e8b57")
deg = 60.0; sx = 2 * R * math.cos(math.radians(deg))
asm = dict(base=stl["base"], guide_bridge=stl["guide_bridge"], crank=pose(stl["crank"], deg),
           coupler=pose(stl["coupler"], 180 - deg, (sx, 0, 0)), slider=pose(stl["slider"], 0, (sx, 0, 0)))
views = [([(stl["base"], COL["base"])], "Base with pivot post", (-60, 38)),
         ([(stl["guide_bridge"], COL["guide_bridge"])], "Guide bridge (slotted plate)", (-60, 38)),
         ([(stl["crank"], COL["crank"])], "Crank, link 2 (boss + pin)", (-60, 38)),
         ([(stl["coupler"], COL["coupler"])], "Coupler, link 3", (-60, 38)),
         ([(stl["slider"], COL["slider"])], "Slider (block + pin)", (-60, 38)),
         ([(asm[n], COL[n]) for n in ("crank", "coupler", "slider")], f"Linkage only at crank angle {deg:.0f}° (base and guide hidden)", (-60, 52))]
fig, axs = plt.subplots(2, 3, figsize=(12, 7.2))
for ax, (parts, ttl, (az, el)) in zip(axs.ravel(), views):
    ax.imshow(render(parts, az, el)); ax.set_title(ttl, fontsize=11); ax.axis("off")
fig.tight_layout(); fig.savefig(ROOT / "figures/04_cad_parts_and_assembly.png", dpi=140); plt.close(fig)
fig, ax = plt.subplots(figsize=(8.6, 5.2))
ax.imshow(render([(asm[n], COL[n]) for n in ("base", "guide_bridge", "crank", "coupler", "slider")], -52, 34, W=1100, H=660)); ax.axis("off")
ax.set_title("Tepee assembly, crank angle 60° (all parts)", fontsize=11); fig.tight_layout(); fig.savefig(ROOT / "figures/05_cad_assembly.png", dpi=140); plt.close(fig)
print("rendered")
