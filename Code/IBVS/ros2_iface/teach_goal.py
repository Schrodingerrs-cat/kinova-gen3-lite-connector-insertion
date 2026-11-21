#!/usr/bin/env python3
"""Teach the goal corners of one socket's marker. No robot motion.

Jog the arm (Code/Teleop) so the plug sits aligned at the pre-insertion pose, keep it still, then:
  python3 ros2_iface/teach_goal.py --socket audio
Writes config/taught_goals.yaml and a snapshot with the corners drawn.
"""
import argparse
import pathlib
import sys

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ibvs.markers import Tracker, median_corners, save_goal  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--socket", required=True)
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--config", type=pathlib.Path, default=ROOT / "config" / "marker_goal.yaml")
    ap.add_argument("--goals", type=pathlib.Path, default=ROOT / "config" / "taught_goals.yaml")
    a = ap.parse_args()
    cfg = yaml.safe_load(a.config.read_text())
    if a.socket not in cfg["sockets"]:
        sys.exit("--socket must be one of: " + ", ".join(cfg["sockets"]))
    marker_id, prefix = cfg["sockets"][a.socket], cfg["camera_topic_prefix"].rstrip("/")

    rclpy.init()
    node, got = Node("teach_goal"), {}
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, prefix + "/color/image_raw", lambda m: got.__setitem__("image", m), qos)
    node.create_subscription(CameraInfo, prefix + "/color/camera_info", lambda m: got.__setitem__("info", m), qos)
    while rclpy.ok() and len(got) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    info = got["info"]
    camera = dict(K=np.array(info.k).reshape(3, 3).tolist(), distortion=list(info.d))
    track = dict(cfg["feature_tracking"], ids=[marker_id], target_pixels={marker_id: [[0, 0], [1, 0], [1, 1], [0, 1]]})
    tracker, frames, last, last_image = Tracker(camera, track), [], None, None
    while rclpy.ok() and len(frames) < a.frames:
        rclpy.spin_once(node, timeout_sec=0.2)
        m = got["image"]
        if m.header.stamp == last:
            continue
        last = m.header.stamp
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        bgr = np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)
        px, valid = tracker.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if valid[0]:
            frames.append(px[0].copy())
            last_image = bgr
        else:
            frames.clear()   # must be seen in consecutive frames
    node.destroy_node()
    rclpy.shutdown()
    if len(frames) < a.frames:
        sys.exit("marker %d not seen in %d consecutive frames" % (marker_id, a.frames))
    corners = median_corners(frames)
    save_goal(a.goals, a.socket, marker_id, corners, camera["K"], np.std(frames, axis=0).max())
    cv2.polylines(last_image, [np.rint(corners).astype(np.int32).reshape(4, 1, 2)], True, (0, 255, 180), 2)
    snapshot = a.goals.parent / ("taught_goal_%s.png" % a.socket)
    cv2.imwrite(str(snapshot), last_image)
    print("taught %s (marker %d): corners %s px. Saved %s and %s" % (a.socket, marker_id, np.round(corners, 1).tolist(), a.goals, snapshot))


if __name__ == "__main__":
    main()
