#!/usr/bin/env python3
"""Plug-only CAD fit, local around the pose predicted from the board. Offline, moves nothing.

  python3 ~/kinova/work/tools/fit_plug_local.py SCAN_DIR --out OUT_DIR [--part NAUO49] [--radius 0.020]

1. Prior: the board strip's yaw / centre / top height from every view (tools/board_yaw.py) place the CAD fixture (strip long edge 90 deg in the CAD, strip centre (0.254, 0)); the CAD plug lands at the predicted pose.
2. Input points per view: pixels that are sticker-teal (hue 80-105, S >= 70, V 35-140) or dark (V < 90), with depth (mixed depth-edge pixels removed), whose base-frame position is within --radius of the predicted plug axis
   horizontally and between the board top and the predicted plug top + 6 mm. The cable rising from the plug and everything far from it is excluded.
3. Fit: x, y, z, yaw of the CAD plug around the prior (trimmed mean distance from the points to the CAD surface, best 70 %, balanced over views, Nelder-Mead from several yaw starts).
4. Yaw ambiguity: score versus yaw (other parameters re-optimised); yaw spread = range of yaw whose score is within 1.2 x the best.
Gate numbers per view (within 2 mm, median, p95) as in method 2's supervised_pickup profile: >= 70 %, p95 <= 5 mm, yaw spread <= 15 deg. Overlays: mask pixels in magenta, fitted CAD samples in yellow.
"""
import argparse
import json
import pathlib
import sys

import cv2
import numpy as np
import trimesh
import yaml
from scipy.optimize import minimize
from scipy.spatial import cKDTree

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board_yaw as by  # noqa: E402

D = pathlib.Path.home() / "kinova/work/setup_cad"
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("scan", type=pathlib.Path)
ap.add_argument("--out", type=pathlib.Path, required=True)
ap.add_argument("--part", default="NAUO49")
ap.add_argument("--radius", type=float, default=0.020)
a = ap.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
views = sorted(a.scan.glob("view_*.npz"))


def rz(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1.0]])


# ---- 1. prior from the board
B = [by.board(v) for v in views]
yaws = np.array([b["yaw"] for b in B])
yaw_b = float(np.degrees(np.angle(np.exp(2j * np.radians(yaws)).mean())) / 2 % 180)
cen_b = np.mean([b["centre"] for b in B], axis=0)
zb = float(np.mean([b["z"] for b in B]))
parts = yaml.safe_load(open(D / "parts_index.yaml"))
strip = [p for p in parts if p["id"] == "NAUO43"][0]
plug_part = [p for p in parts if p["id"] == a.part][0]
strip_top = trimesh.load(D / "parts" / strip["file"]).bounds[1][2]
c0 = np.array([0.25, -0.05])
R0 = rz(yaw_b - 90)
t_xy = cen_b - (R0[:2, :2] @ (np.array(strip["centre_m"][:2]) - c0) + c0)
t = np.array([t_xy[0], t_xy[1], zb - strip_top])
mesh = trimesh.load(D / "parts" / plug_part["file"])
V0 = (mesh.vertices - np.r_[c0, 0]) @ R0.T + np.r_[c0, 0] + t
mesh_p = trimesh.Trimesh(V0, mesh.faces)
samples = mesh_p.sample(7000)
pc = np.r_[mesh_p.bounds.mean(axis=0)[:2], 0.0]
top_pred = mesh_p.bounds[1][2]
print("prior from the board: yaw %.2f deg (strip), centre x %.4f y %.4f, top face z %.4f | CAD plug %s predicted at x %.4f y %.4f, top z %.4f, bottom z %.4f" % (yaw_b, *cen_b, zb, a.part, pc[0], pc[1], top_pred, mesh_p.bounds[0][2]))

# ---- 2. plug points per view
data = []
for v in views:
    f = np.load(v)
    d, c, K, T = [f[k].astype(np.float32) if k == "depth" else f[k] for k in ("depth", "color", "K", "T_BC")]
    med = cv2.medianBlur(d, 5)
    ok = (d > 0.08) & (d < 0.8) & (np.abs(d - med) < 0.004)
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    teal = (hsv[..., 0] >= 80) & (hsv[..., 0] <= 105) & (hsv[..., 1] >= 70) & (hsv[..., 2] >= 35) & (hsv[..., 2] <= 140)
    cand = (teal | (hsv[..., 2] < 90)) & ok
    vv, uu = np.nonzero(cand)
    z = d[vv, uu]
    P = np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
    keep = (np.hypot(P[:, 0] - pc[0], P[:, 1] - pc[1]) <= a.radius) & (P[:, 2] >= zb + 0.003) & (P[:, 2] <= top_pred + 0.006)
    mask = np.zeros(d.shape, bool)
    mask[vv[keep], uu[keep]] = True
    pts = P[keep]
    if len(pts) > 1500:
        pts = pts[np.random.default_rng(0).choice(len(pts), 1500, replace=False)]
    Pm = np.full(d.shape + (3,), np.nan, np.float32)
    v2, u2 = np.nonzero(ok)
    z2 = d[v2, u2]
    Pm[v2, u2] = np.c_[(u2 - K[0, 2]) * z2 / K[0, 0], (v2 - K[1, 2]) * z2 / K[1, 1], z2] @ T[:3, :3].T + T[:3, 3]
    data.append(dict(name=v.stem, pts=pts, mask=mask, color=c, K=K, T=T, n_cand=int(cand.sum()), Pm=Pm))
    print("%s: %d candidate pixels, %d plug points kept (within %.0f mm of the predicted plug, z %.3f..%.3f)" % (v.stem, cand.sum(), keep.sum(), a.radius * 1e3, zb + 0.003, top_pred + 0.006))


# ---- 2b. face orientation from the surface normals of the plug pixels (independent of the CAD size)
def face_yaw(x, step=2):
    vv, uu = np.nonzero(x["mask"])
    sel = (vv >= step) & (vv < x["mask"].shape[0] - step) & (uu >= step) & (uu < x["mask"].shape[1] - step)
    vv, uu = vv[sel], uu[sel]
    Pm = x["Pm"]
    A = Pm[vv, uu + step] - Pm[vv, uu - step]
    Bv = Pm[vv + step, uu] - Pm[vv - step, uu]
    N = np.cross(A, Bv)
    n = np.linalg.norm(N, axis=1)
    good = np.isfinite(n) & (n > 0) & (np.abs(A).max(axis=1) < 0.006) & (np.abs(Bv).max(axis=1) < 0.006)
    N = N[good] / n[good, None]
    P = Pm[vv[good], uu[good]]
    flip = (N * (x["T"][:3, 3] - P)).sum(axis=1) < 0
    N[flip] *= -1
    vert = np.abs(N[:, 2]) < 0.35
    if vert.sum() < 15:
        return None, int(vert.sum()), int(len(N))
    ang = np.degrees(np.arctan2(N[vert, 1], N[vert, 0]))
    z = np.exp(1j * np.radians(4 * ang))
    return float(np.degrees(np.angle(z.mean())) / 4 % 90), int(vert.sum()), float(abs(z.mean())), int(len(N))


print("\nface orientation from surface normals (vertical faces only, angle mod 90 deg; the board strip direction is %.1f deg mod 90):" % (yaw_b % 90))
fy = []
for x in data:
    r = face_yaw(x)
    if r[0] is None:
        print("  %s: only %d vertical-face pixels of %d, no estimate" % (x["name"], r[1], r[2]))
    else:
        fy.append(r[0])
        print("  %s: face yaw %5.1f deg (concentration %.2f) from %d vertical-face pixels of %d" % (x["name"], r[0], r[2], r[1], r[3]))
if len(fy) >= 2:
    zz = np.exp(1j * np.radians(4 * np.array(fy)))
    print("  views agree: mean %.1f deg, circular spread (max deviation from the mean) %.1f deg" % (np.degrees(np.angle(zz.mean())) / 4 % 90, np.max(np.abs((np.array(fy) - np.degrees(np.angle(zz.mean())) / 4 + 45) % 90 - 45))))
    print("  difference to the board strip direction: %+.1f deg" % ((np.degrees(np.angle(zz.mean())) / 4 - yaw_b + 45) % 90 - 45))

# ---- 3. fit
def moved(theta):
    dx, dy, dz, dyaw = theta
    return (samples - pc) @ rz(dyaw).T + pc + np.array([dx, dy, dz])


def trimmed(dist):
    s = np.sort(dist)
    return float(s[:max(5, int(0.7 * len(s)))].mean())


def score(theta, tree=None):
    tr = cKDTree(moved(theta))
    return float(np.mean([trimmed(tr.query(x["pts"])[0]) for x in data]))


def fit_from(yaw0, fix_yaw=None):
    if fix_yaw is None:
        x0 = np.array([0, 0, 0, yaw0], float)
        sim = np.array([x0, x0 + [0.003, 0, 0, 0], x0 + [0, 0.003, 0, 0], x0 + [0, 0, 0.003, 0], x0 + [0, 0, 0, 5.0]])
        r = minimize(lambda th: score(th), x0, method="Nelder-Mead", options=dict(initial_simplex=sim, xatol=1e-4, fatol=1e-6, maxiter=400))
        return r.x, r.fun
    x0 = np.zeros(3)
    r = minimize(lambda th: score(np.r_[th, fix_yaw]), x0, method="Nelder-Mead", options=dict(initial_simplex=np.array([x0, x0 + [0.003, 0, 0], x0 + [0, 0.003, 0], x0 + [0, 0, 0.003]]), xatol=1e-4, fatol=1e-6, maxiter=300))
    return np.r_[r.x, fix_yaw], r.fun


best = min((fit_from(y0) for y0 in (-20, -10, 0, 10, 20)), key=lambda r: r[1])
theta, fun = best
prof = [(dy, fit_from(0, fix_yaw=dy)[1]) for dy in range(-60, 61, 6)]
good = [dy for dy, s in prof if s <= 1.2 * fun]
spread = max(good) - min(good)

# ---- 4. report + overlays
tr = cKDTree(moved(theta))
rows = []
for k, x in enumerate(data):
    dist = tr.query(x["pts"])[0]
    rows.append(dict(view=x["name"], points=len(x["pts"]), within_2mm=float(100 * np.mean(dist < 0.002)), median_mm=float(np.median(dist) * 1e3), p95_mm=float(np.percentile(dist, 95) * 1e3)))
    im = x["color"].copy()
    im[x["mask"]] = (255, 0, 255)
    M = moved(theta)
    cam = (M - x["T"][:3, 3]) @ x["T"][:3, :3]
    u = (x["K"][0, 0] * cam[:, 0] / cam[:, 2] + x["K"][0, 2]).astype(int)
    w = (x["K"][1, 1] * cam[:, 1] / cam[:, 2] + x["K"][1, 2]).astype(int)
    for uu, ww in zip(u, w):
        if 0 <= uu < im.shape[1] and 0 <= ww < im.shape[0]:
            im[ww, uu] = (0, 255, 255)
    cx, cy = int(np.median(u)), int(np.median(w))
    cv2.imwrite(str(a.out / (x["name"] + "_overlay.png")), cv2.resize(im[max(0, cy - 90):cy + 90, max(0, cx - 120):cx + 120], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
accepted = all(r["within_2mm"] >= 70 and r["p95_mm"] <= 5 for r in rows) and spread <= 15
yaw_fit = float((yaw_b - 90 + theta[3]) % 180)
res = dict(prior=dict(board_yaw=yaw_b, plug_centre=pc[:2].tolist(), top_z=float(top_pred)), shift_mm=(theta[:3] * 1e3).tolist(), yaw_correction_deg=float(theta[3]), plug_yaw_deg=yaw_fit,
           plug_centre_m=(pc + theta[:3]).tolist(), score_mm=fun * 1e3, yaw_spread_deg=float(spread), yaw_profile=[(int(dy), round(s * 1e3, 2)) for dy, s in prof], per_view=rows, accepted=bool(accepted))
(a.out / "fit_local.json").write_text(json.dumps(res, indent=1))
print("\nfit: shift from the board prior (%+.1f, %+.1f, %+.1f) mm, yaw correction %+.1f deg -> plug yaw %.1f deg, trimmed mean distance %.2f mm" % (*(theta[:3] * 1e3), theta[3], yaw_fit, fun * 1e3))
print("yaw profile (dyaw: score mm): %s" % " ".join("%d:%.2f" % (dy, s * 1e3) for dy, s in prof))
print("yaw spread (score within 1.2 x best): %.0f deg" % spread)
for r in rows:
    print("  %s: %4d points, within 2 mm %3.0f %%, median %.1f mm, p95 %.1f mm" % (r["view"], r["points"], r["within_2mm"], r["median_mm"], r["p95_mm"]))
print("GATES (>= 70 %% within 2 mm, p95 <= 5 mm, yaw spread <= 15 deg): %s" % ("ACCEPTED" if accepted else "rejected"))
tiles = [cv2.imread(str(a.out / (x["name"] + "_overlay.png"))) for x in data]
h = min(t.shape[0] for t in tiles)
cv2.imwrite(str(a.out / "overlays.png"), cv2.resize(np.hstack([t[:h, :360] for t in tiles]), None, fx=0.55, fy=0.55))
