#!/usr/bin/env python3
"""Plug pose from the segmented plug points, no CAD part needed. Offline, moves nothing.

  python3 ~/kinova/work/tools/plug_box.py SCAN_DIR --out OUT_DIR

Plug points as in fit_plug_local.py (sticker-teal or dark pixels near the board-predicted plug, below the plug top + 6 mm, cable excluded). All views are merged (balanced, equal points per view) and a
rotated rectangle is fitted to their plan view (min area of the 5th-95th percentile extents, yaw searched in 1 deg steps); height = 5th..95th percentile of z. Repeatability: the fit is repeated with each
view left out. Overlay per view: mask in magenta, fitted box in green (top) and blue (bottom).
Reported: centre x y z, footprint (long x short, mm), yaw of the long side (mod 180), leave-one-view-out spread of centre (mm) and yaw (deg), and the difference to the board strip direction.
"""
import argparse
import json
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import plug_points as pp  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("scan", type=pathlib.Path)
ap.add_argument("--out", type=pathlib.Path, required=True)
ap.add_argument("--radius", type=float, default=0.020)
ap.add_argument("--per-view", type=int, default=600)
a = ap.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
views = sorted(a.scan.glob("view_*.npz"))
pr = pp.prior(views)
data = pp.points(views, pr, a.radius)
rng = np.random.default_rng(1)


def merged(ix):
    return np.vstack([d["pts"][rng.choice(len(d["pts"]), min(a.per_view, len(d["pts"])), replace=False)] for i, d in enumerate(data) if i in ix])


def fit_box(P):
    xy, c = P[:, :2], np.median(P[:, :2], axis=0)
    best = None
    for th in range(0, 180):
        q = (xy - c) @ pp.rz(-th)[:2, :2].T
        lo, hi = np.percentile(q, 5, axis=0), np.percentile(q, 95, axis=0)
        area = np.prod(hi - lo)
        if best is None or area < best[0]:
            best = (area, th, lo, hi)
    _, th, lo, hi = best
    ext = hi - lo
    centre = c + pp.rz(th)[:2, :2] @ ((lo + hi) / 2)
    yaw = th if ext[0] >= ext[1] else th + 90
    return dict(centre=centre, yaw=float(yaw % 180), long=float(max(ext)), short=float(min(ext)), z0=float(np.percentile(P[:, 2], 5)), z1=float(np.percentile(P[:, 2], 95)), th=th, ext=ext)


for d in data:
    print("%s: %d plug points" % (d["name"], len(d["pts"])))
full = fit_box(merged(set(range(len(data)))))
loo = [fit_box(merged(set(range(len(data))) - {k})) for k in range(len(data))]
yaws = np.array([r["yaw"] for r in loo] + [full["yaw"]])
cen = np.array([r["centre"] for r in loo])
ydev = (yaws - full["yaw"] + 90) % 180 - 90
strip_dir = pr["board_yaw"] % 180
print("\nplug box from all %d views: centre x %.4f y %.4f m, z %.4f..%.4f, footprint %.1f x %.1f mm, long side yaw %.1f deg" % (len(data), *full["centre"], full["z0"], full["z1"], full["long"] * 1e3, full["short"] * 1e3, full["yaw"]))
print("board strip direction %.1f deg, so the plug long side is %+.1f deg from the strip's perpendicular (%.1f)" % (strip_dir, (full["yaw"] - (strip_dir - 90) + 90) % 180 - 90, (strip_dir - 90) % 180))
print("leave-one-view-out: centre spread %.1f mm (x %.1f, y %.1f), yaw deviations %s deg (max %.1f)" % (np.linalg.norm(cen.max(0) - cen.min(0)) * 1e3, np.ptp(cen[:, 0]) * 1e3, np.ptp(cen[:, 1]) * 1e3, np.round(ydev[:-1], 1).tolist(), np.abs(ydev[:-1]).max()))
# overlays
tiles = []
corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * (full["ext"] / 2)
base = full["centre"] + (pp.rz(full["th"])[:2, :2] @ corners.T).T if False else None
th = full["th"]
q0 = np.median(merged(set(range(len(data))))[:, :2], axis=0)
for dd in data:
    im = dd["color"].copy()
    im[dd["mask"]] = (255, 0, 255)
    cxy = full["centre"]
    ext = full["ext"]
    for z, col in ((full["z1"], (0, 255, 0)), (full["z0"], (255, 128, 0))):
        pts = []
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            xy = cxy + pp.rz(th)[:2, :2] @ np.array([sx * ext[0] / 2, sy * ext[1] / 2])
            P = np.r_[xy, z]
            cam = (P - dd["T"][:3, 3]) @ dd["T"][:3, :3]
            pts.append([dd["K"][0, 0] * cam[0] / cam[2] + dd["K"][0, 2], dd["K"][1, 1] * cam[1] / cam[2] + dd["K"][1, 2]])
        cv2.polylines(im, [np.array(pts, np.int32)], True, col, 1)
    c0 = np.array(pts).mean(axis=0).astype(int)
    tiles.append(cv2.resize(im[max(0, c0[1] - 70):c0[1] + 70, max(0, c0[0] - 90):c0[0] + 90], (480, 360), interpolation=cv2.INTER_NEAREST))
    cv2.putText(tiles[-1], dd["name"], (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
while len(tiles) % 3:
    tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(str(a.out / "box_overlays.png"), np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]))
(a.out / "plug_box.json").write_text(json.dumps(dict(centre=full["centre"].tolist(), z=[full["z0"], full["z1"]], long_mm=full["long"] * 1e3, short_mm=full["short"] * 1e3, yaw_deg=full["yaw"], strip_dir=strip_dir,
                                                    loo_centre_spread_mm=float(np.linalg.norm(cen.max(0) - cen.min(0)) * 1e3), loo_yaw_dev_deg=ydev[:-1].tolist()), indent=1))
