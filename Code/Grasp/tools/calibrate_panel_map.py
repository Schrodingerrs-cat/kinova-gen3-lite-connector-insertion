"""Measure the plug's pose in the tag frame (the panel map entry of config/panel_map.yaml) from saved close frames in which the panel tag AND the plug's colour sticker
are visible (offline, no robot): the sticker gives the plug's x, y, yaw (plane normal, plug axis 3.75 mm behind the sticker), the tag gives the panel frame.
python3 tools/calibrate_panel_map.py --size-m 0.025 FRAME.npz [FRAME.npz ...] [--write]
FRAME.npz = plan/<ts>/frame.npz of any run (depth, color BGR, K, T_BC). Use frames with the panel in different places/rotations: a good map has a small spread."""
import argparse
import pathlib
import re
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from housing_grasp.color_marker import marker_points  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp.model_mode import detect_tag_pose, plug_pose_from_sticker, tag_plane_height  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("frames", nargs="+")
ap.add_argument("--size-m", type=float, required=True, help="side of the printed black square, measured with a ruler")
ap.add_argument("--write", action="store_true", help="write size and offset into config/panel_map.yaml")
args = ap.parse_args()
g = load_config(str(ROOT / "config" / "grasp.yaml"), allow_unscanned=True)["color_marker"]
tag_cfg = dict(dictionary="DICT_4X4_50", id=0, size_m=args.size_m)
rows = []
for f in args.frames:
    F = np.load(f)
    color, depth, K, T_BC = F["color"], F["depth"], F["K"], F["T_BC"]
    tag = detect_tag_pose(color, K, T_BC, tag_cfg)
    zd = tag_plane_height(depth, K, T_BC, tag["corners_px"])
    T = tag["T"].copy()
    if zd is not None:
        T[2, 3] = zd
    G = marker_points(depth, color, K, T_BC, g)
    G = G[np.linalg.norm(G[:, :2] - np.median(G[:, :2], 0), axis=1) < 0.022]
    st = plug_pose_from_sticker(G, T_BC[:3, 3])
    yaw_tag = np.arctan2(T[1, 0], T[0, 0])
    c, s = np.cos(yaw_tag), np.sin(yaw_tag)
    d = np.array([[c, s], [-s, c]]) @ (st["xy"] - T[:2, 3])                       # plug axis in the tag frame
    yaw_off = float(np.degrees((st["yaw"] - yaw_tag + np.pi / 2) % np.pi - np.pi / 2))   # plug is symmetric under 180 deg: keep (-90, 90]
    rows.append((d[0], d[1], yaw_off))
    print("%s: tag %.0f px, plug in the tag frame (%.1f, %.1f) mm, yaw offset %.1f deg (sticker plane %.0f deg from horizontal, %d points)" % (pathlib.Path(f).parent.parent.name, tag["side_px"], d[0] * 1e3, d[1] * 1e3, yaw_off, st["tilt_deg"], len(G)))
R = np.array(rows)
mean_yaw = float(np.degrees(np.angle(np.mean(np.exp(2j * np.radians(R[:, 2])))) / 2))   # circular mean on the doubled angle
spread_yaw = float(np.degrees(np.sqrt(np.mean(np.angle(np.exp(2j * (np.radians(R[:, 2]) - np.radians(mean_yaw)))) ** 2)) / 2))
mean_xy = R[:, :2].mean(0)
print("N = %d: offset (%.2f, %.2f) mm +- (%.2f, %.2f) mm, yaw %.1f deg +- %.1f deg" % (len(R), *(mean_xy * 1e3), *(R[:, :2].std(0) * 1e3), mean_yaw, spread_yaw))
if args.write:
    p = ROOT / "config" / "panel_map.yaml"
    t = p.read_text()
    t = re.sub(r"size_m: null", "size_m: %g" % args.size_m, t, count=1)
    t = re.sub(r"xy_m: null", "xy_m: [%.5f, %.5f]" % tuple(mean_xy), t, count=1)
    t = re.sub(r"yaw_deg: null", "yaw_deg: %.2f" % mean_yaw, t, count=1)
    p.write_text(t)
    print("written to", p)
