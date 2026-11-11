"""Offline check of scan.views_joints_deg (no ROS, no robot): forward kinematics from the robot URDF, then
(1) lowest point of the straight joint-space path home -> poses -> home, (2) can each pose see a given point (default: where the head was).
python3 tools/check_scan_poses.py [x y z of the head, m]"""
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
URDF = "/opt/ros/humble/share/kortex_description/robots/gen3_lite.urdf"
cfg = yaml.safe_load(open(ROOT / "config" / "grasp.yaml"))
head = np.array([float(a) for a in sys.argv[1:4]]) if len(sys.argv) >= 4 else np.array([0.26, -0.02, 0.005])


def rot(r, p, y):
    cx, sx, cy, sy, cz, sz = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])


def origin(j):
    o = j.find("origin")
    xyz, rpy = [float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy").split()]
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = rot(*rpy), xyz
    return M


joints = {j.get("name"): j for j in ET.parse(URDF).getroot().iter("joint") if j.find("origin") is not None}
arm = [origin(joints["joint_%d" % i]) for i in range(1, 7)]
tool = origin(joints["end_effector"]) @ origin(joints["tool_frame_joint"])


def fk(q_deg):
    M, pts = np.eye(4), []
    for O, q in zip(arm, np.radians(q_deg)):
        c, s = np.cos(q), np.sin(q)
        M = M @ O @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        pts.append(M[:3, 3])
    return M @ tool, pts


wrap = lambda q: (np.asarray(q, float) + 180) % 360 - 180
home, poses = wrap(cfg["scan"]["home_joints_deg"]), [wrap(q) for q in cfg["scan"]["views_joints_deg"]]
tcp, hand_eye = np.array(cfg["gripper"]["tcp_in_tool_xyz_m"]), cfg["camera"]
x, y, z, w = hand_eye["hand_eye_quat_xyzw"]
T_tc = np.eye(4)
T_tc[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
T_tc[:3, 3] = hand_eye["hand_eye_xyz_m"]
seq = [home] + poses + [home]
low = {"wrist (joint 5)": 9.0, "tool frame": 9.0, "fingertip": 9.0}
for a, b in zip(seq[:-1], seq[1:]):
    for s in np.linspace(0, 1, 100):
        T, pts = fk(a + (b - a) * s)
        for n, p in (("wrist (joint 5)", pts[4]), ("tool frame", T[:3, 3]), ("fingertip", (T @ np.r_[tcp, 1])[:3])):
            low[n] = min(low[n], p[2])
steps = [np.abs(b - a).max() for a, b in zip(seq[:-1], seq[1:])]
print("moves (largest joint, deg):", np.round(steps).astype(int).tolist(), "| total %.0f deg, about %.0f s at motion.joint_speed_deg_s" % (sum(steps), sum(max(2, s / cfg["motion"]["joint_speed_deg_s"]) for s in steps)))
print("lowest point on the path (mm above the base plane):", {k: round(v * 1e3) for k, v in low.items()}, "| table z %.0f mm" % (cfg["scan"]["table_z_m"] * 1e3))
for i, q in enumerate(poses, 1):
    T, _ = fk(q)
    Tbc = T @ T_tc
    c = np.linalg.inv(Tbc) @ np.r_[head, 1]
    u, v = 434.7 * c[0] / c[2] + 424.0, 434.7 * c[1] / c[2] + 240.0
    ok = c[2] > 0.07 and 0 <= u < 848 and 0 <= v < 480
    print("pose %d: tool (%.3f, %.3f, %.3f) m, camera axis %2.0f deg from straight down, point at pixel (%4.0f, %4.0f), %.2f m: %s" % (
        i, *T[:3, 3], np.degrees(np.arccos(np.clip(-Tbc[2, 2], -1, 1))), u, v, c[2], "in view" if ok else "NOT in view"))
