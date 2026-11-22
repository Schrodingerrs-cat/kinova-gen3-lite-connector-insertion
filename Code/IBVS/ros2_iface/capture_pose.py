#!/usr/bin/env python3
"""Save what the arm and camera see right now: color photo, depth, joint values (deg), gripper, visible markers. No robot motion.
  python3 ros2_iface/capture_pose.py --name hand_moved_1        -> Media/IBVS/poses/<name>_<time>/"""
import argparse
import json
import pathlib
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState

ROOT = pathlib.Path(__file__).resolve().parents[1]
JOINTS = [f"joint_{i}" for i in range(1, 7)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="pose")
    ap.add_argument("--config", type=pathlib.Path, default=ROOT / "config" / "marker_goal.yaml")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT.parents[1] / "Media" / "IBVS" / "poses")
    a = ap.parse_args()
    cfg = yaml.safe_load(a.config.read_text())
    configured = cfg["camera_topic_prefix"].rstrip("/")
    prefixes = [configured] + [p for p in ("/camera/camera",) if p != configured]      # the camera runs under one naming or the other
    rclpy.init()
    node, got = Node("capture_pose"), {}
    q = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    for prefix in prefixes:
        node.create_subscription(Image, prefix + "/color/image_raw", lambda m: got.__setitem__("color", m), q)
        node.create_subscription(Image, prefix + "/aligned_depth_to_color/image_raw", lambda m: got.__setitem__("depth", m), q)
        node.create_subscription(CameraInfo, prefix + "/color/camera_info", lambda m: got.__setitem__("info", m), q)
    node.create_subscription(JointState, "/joint_states", lambda m: got.__setitem__("joints", dict(zip(m.name, m.position))), 10)
    t0 = time.time()
    while not all(k in got for k in ("color", "depth", "info")) and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    for _ in range(10):                                   # joint values only if the arm driver happens to be running
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [k for k in ("joints", "color", "depth", "info") if k not in got]
    node.destroy_node()
    rclpy.shutdown()
    if "color" in missing:
        sys.exit("no camera image on %s: is the camera running?" % " or ".join(p + "/color/image_raw" for p in prefixes))
    folder = a.out / ("%s_%s" % (a.name, time.strftime("%Y%m%d_%H%M%S")))
    folder.mkdir(parents=True)
    joints = [float(np.degrees(got["joints"][j])) for j in JOINTS] if "joints" in got else None
    info = dict(name=a.name, joints_deg=None if joints is None else [round(v, 4) for v in joints],
                gripper_position=got.get("joints", {}).get("right_finger_bottom_joint"), missing=missing)
    if "color" in got:
        m = got["color"]
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        bgr = np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)
        cv2.imwrite(str(folder / "color.png"), bgr)
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["feature_tracking"]["aruco_dict"])),
                                           cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        info["markers_px"] = {} if ids is None else {int(i): dict(centre=np.round(c.reshape(4, 2).mean(0), 1).tolist(), side=round(float(np.linalg.norm(c.reshape(4, 2)[1] - c.reshape(4, 2)[0])), 1)) for i, c in zip(ids.ravel(), corners)}
    if "depth" in got:
        d = got["depth"]
        depth = np.frombuffer(d.data, np.uint16).reshape(d.height, d.step // 2)[:, :d.width]
        cv2.imwrite(str(folder / "depth_mm.png"), depth)
        valid = depth[depth > 0]
        info["depth_valid_fraction"] = round(float(valid.size / depth.size), 3)
        info["depth_median_mm"] = float(np.median(valid)) if valid.size else None
    if "info" in got:
        info["camera_K"] = list(got["info"].k)
    (folder / "pose.json").write_text(json.dumps(info, indent=1) + "\n")
    print("saved", folder)
    print("joints deg:", "not available (arm driver not running)" if joints is None else [round(v, 2) for v in joints], "| gripper", info["gripper_position"])
    print("markers visible:", info.get("markers_px", "no color image"))
    if joints is not None:
        print("as a post_grasp line:  - " + str([round(v, 4) for v in joints]))


if __name__ == "__main__":
    main()
