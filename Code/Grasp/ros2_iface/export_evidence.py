#!/usr/bin/env python3
"""Package everything from a colour-marker grasp run into its Media folder (offline, no ROS, no robot).
python3 ros2_iface/export_evidence.py RUN_DIR

Needs RUN_DIR/plan/<ts>/ (plan.json, frame.npz, marker_close_points.npy), RUN_DIR/scan_used.txt or RUN_DIR/scan/<ts>/, videos/camera.mp4, trajectory.csv.
Writes: scan/ copy, point_clouds/*.ply, perception/*.png, videos/grasp_annotated.mp4.
"""
import csv
import json
import pathlib
import shutil
import sys

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp.color_marker import marker_mask  # noqa: E402

RUN = pathlib.Path(sys.argv[1])


def pick(d, *names):  # runs saved before the 2026-11-07 rename use the old green_* file names
    return next(d / n for n in names if (d / n).exists())

cfg = load_config(str(ROOT / "config" / "grasp.yaml"), allow_unscanned=True)
G = cfg["color_marker"]
(RUN / "point_clouds").mkdir(exist_ok=True)
(RUN / "perception").mkdir(exist_ok=True)
plan_dir = sorted((RUN / "plan").glob("2*"))[-1]
plan = json.load(open(plan_dir / "plan.json"))
F = np.load(plan_dir / "frame.npz")
color, depth, K, T_BC = F["color"], F["depth"], F["K"], F["T_BC"]  # colour is BGR (saved by /grasp/sticker_grasp)

if (RUN / "scan_used.txt").exists():  # run reused a scan: copy it in so the package is self-contained
    src = pathlib.Path((RUN / "scan_used.txt").read_text().strip())
    if not (RUN / "scan" / src.name).exists():  # already inside the run folder: the original location may have been moved
        shutil.copytree(src, RUN / "scan" / src.name)
scan = sorted((RUN / "scan").glob("2*"))[-1]


def ply(path, xyz, rgb):
    a = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    a["x"], a["y"], a["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    a["r"], a["g"], a["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n" % len(xyz)).encode())
        f.write(a.tobytes())


def project(p):
    c = np.linalg.inv(T_BC) @ np.r_[p, 1]
    return int(round(K[0, 0] * c[0] / c[2] + K[0, 2])), int(round(K[1, 1] * c[1] / c[2] + K[1, 2]))


# ---- point clouds (PLY, base frame, metres) ----
fused = np.load(scan / "fused_cloud.npy")
h = np.clip((fused[:, 2] + 0.06) / 0.12, 0, 1)
cm = (plt.get_cmap("viridis")(h)[:, :3] * 255).astype(np.uint8)
ply(RUN / "point_clouds" / "1_fused_scan_all_points.ply", fused, cm)
marker = np.load(pick(scan, "marker_cloud.npy", "green_cloud.npy"))
ply(RUN / "point_clouds" / "2_fused_scan_marker_only.ply", marker, np.tile(np.array([0, 255, 0], np.uint8), (len(marker), 1)))
vv, uu = np.nonzero(np.isfinite(depth) & (depth > 0))
z = depth[vv, uu].astype(float)
pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z], 1)
pb = pc @ T_BC[:3, :3].T + T_BC[:3, 3]
ply(RUN / "point_clouds" / "3_close_view_single_frame_colour.ply", pb, color[vv, uu][:, ::-1])
gc = np.load(pick(plan_dir, "marker_close_points.npy", "green_close_points.npy"))
ply(RUN / "point_clouds" / "4_close_view_marker.ply", gc, np.tile(np.array([0, 255, 0], np.uint8), (len(gc), 1)))

# ---- perception overlay on the close view ----
img = color.copy()
m = marker_mask(color, G).astype(np.uint8)
cv2.drawContours(img, cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, (0, 255, 0), 2)
fp, c3, tcp, jaw, ow = plan["footprint"], np.array(plan["sticker_centroid_median_m"]), np.array(plan["tcp_m"]), np.array(plan["jaw_axis"]), plan["open_width_m"]
ctr, la, ja = np.array(fp["center_xy"]), np.array(fp["long_axis"]), np.array(fp["jaw_axis"])
corners = [ctr + sl * la * fp["length_m"] / 2 + sj * ja * fp["width_m"] / 2 for sl, sj in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
cv2.polylines(img, [np.array([project(np.r_[q, c3[2]]) for q in corners], np.int32)], True, (0, 165, 255), 2)  # housing footprint
for s in (1, -1):  # open finger positions
    cv2.circle(img, project(tcp + s * jaw * ow / 2), 7, (0, 0, 255), -1)
cv2.line(img, project(tcp + jaw * ow / 2), project(tcp - jaw * ow / 2), (0, 0, 255), 1)
cv2.drawMarker(img, project(tcp), (255, 0, 0), cv2.MARKER_STAR, 18, 2)
x0, y0 = project(c3)
cv2.imwrite(str(RUN / "perception" / "01_close_view_overlay.png"), img[max(0, y0 - 130):y0 + 130, max(0, x0 - 190):x0 + 190])
cv2.imwrite(str(RUN / "perception" / "01_close_view_overlay_full.png"), img)

# ---- marker mask in six scan views ----
tiles = []
for k in (0, 5, 6, 9, 10, 12):
    v = np.load(scan / ("view_%02d.npz" % k))
    t = v["color"].copy()
    cv2.drawContours(t, cv2.findContours(marker_mask(t, G).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, (0, 255, 0), 2)
    cv2.putText(t, "scan view %d" % k, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    tiles.append(cv2.resize(t, (424, 240)))
cv2.imwrite(str(RUN / "perception" / "02_marker_mask_in_scan_views.png"), np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])]))

# ---- single close frame, three projections ----
near = pb[np.linalg.norm(pb[:, :2] - tcp[:2], axis=1) < 0.05]
fig, ax = plt.subplots(1, 3, figsize=(16, 5))
for a, (i, j, t) in zip(ax, ((0, 1, "top x-y"), (0, 2, "side x-z"), (1, 2, "side y-z"))):
    a.scatter(near[:, i] * 1e3, near[:, j] * 1e3, s=1.5, c=near[:, 2] * 1e3, cmap="viridis")
    a.scatter(gc[:, i] * 1e3, gc[:, j] * 1e3, s=5, c="lime", label="marker sticker")
    a.plot(tcp[i] * 1e3, tcp[j] * 1e3, "r*", ms=14, label="TCP target")
    a.set_title("close view, single frame, " + t); a.set_xlabel("xyz"[i] + " (mm)"); a.set_ylabel("xyz"[j] + " (mm)"); a.set_aspect("equal"); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig(RUN / "perception" / "03_close_frame_point_cloud_3_views.png", dpi=95); plt.close(fig)

# ---- annotated video (measured values from trajectory.csv, no stage guesses) ----
rows = np.array([[float(x) if x not in ("", "nan") else np.nan for x in r] for r in list(csv.reader(open(RUN / "trajectory.csv")))[1:] if len(r) == 11])
cap = cv2.VideoCapture(str(RUN / "videos" / "camera.mp4"))
w, hgt, fps = int(cap.get(3)), int(cap.get(4)), cap.get(cv2.CAP_PROP_FPS)
out = cv2.VideoWriter(str(RUN / "videos" / "grasp_annotated.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, hgt))
i = 0
while True:
    ok, f = cap.read()
    if not ok:
        break
    r = rows[min(int(i / fps * 10), len(rows) - 1)]
    cv2.rectangle(f, (0, 0), (w, 52), (0, 0, 0), -1)
    cv2.putText(f, "t=%5.1fs  tool xyz mm (%.0f, %.0f, %.0f)  gripper %.2f rad (0 open, 0.81 closed)" % (i / fps, *(r[8:11] * 1e3), r[7]), (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    out.write(f)
    i += 1
out.release()
print("exported: %d points fused, %d marker points, close frame %d points; video frames %d" % (len(fused), len(marker), len(pb), i))
