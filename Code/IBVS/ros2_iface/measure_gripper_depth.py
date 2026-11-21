#!/usr/bin/env python3
"""Measure the camera-fixed gripper (and whatever it holds) so the servo can tell it from obstacles. No robot motion.
Close the gripper on the plug (or empty), keep hands and objects away from it, then:
  python3 ros2_iface/measure_gripper_depth.py --name closed_plug
Saves config/gripper_depth_<name>.npz only if a second burst of frames matches it (support >= 0.9)."""
import argparse
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
sys.path.insert(0, str(ROOT))
from ibvs.self_depth import SelfDepthTemplate  # noqa: E402


def burst(node, got, count):
    frames, last = [], None
    while len(frames) < count:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not all(k in got for k in "cdij"):
            continue
        c, d = got["c"], got["d"]
        if c.header.stamp == last:
            continue
        last = c.header.stamp
        time.sleep(0.25)
        img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.step)[:, :c.width * 3].reshape(c.height, c.width, 3)
        frames.append((np.ascontiguousarray(img[:, :, ::-1] if c.encoding == "rgb8" else img),
                       np.frombuffer(d.data, np.uint16).reshape(d.height, d.step // 2)[:, :d.width].astype(float) / 1000))
    return np.array([f[0] for f in frames]), np.array([f[1] for f in frames])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True)
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--near-m", type=float, default=.20, help="anything nearer than this in nearly every frame is gripper or held object")
    ap.add_argument("--config", type=pathlib.Path, default=ROOT / "config" / "marker_goal.yaml")
    a = ap.parse_args()
    prefix = yaml.safe_load(a.config.read_text())["camera_topic_prefix"].rstrip("/")
    rclpy.init()
    node, got = Node("measure_gripper_depth"), {}
    q = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, prefix + "/color/image_raw", lambda m: got.__setitem__("c", m), q)
    node.create_subscription(Image, prefix + "/aligned_depth_to_color/image_raw", lambda m: got.__setitem__("d", m), q)
    node.create_subscription(CameraInfo, prefix + "/color/camera_info", lambda m: got.__setitem__("i", m), q)
    node.create_subscription(JointState, "/joint_states", lambda m: got.__setitem__("j", dict(zip(m.name, m.position))), 10)
    color, depth = burst(node, got, a.frames)
    color2, depth2 = burst(node, got, 6)
    info, grip = got["i"], got["j"]["right_finger_bottom_joint"]
    K, D, frame_id = np.array(info.k).reshape(3, 3), np.array(info.d), info.header.frame_id
    node.destroy_node()
    rclpy.shutdown()

    near = ((depth > 0) & (depth < a.near_m)).mean(axis=0) >= .8
    near = cv2.morphologyEx(near.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(near)
    keep = np.isin(labels, [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 300]) & (depth[-1] > 0)
    if keep.sum() < 500:
        sys.exit("only %d gripper pixels found nearer than %.2f m: is the gripper in view?" % (keep.sum(), a.near_m))
    template = SelfDepthTemplate.calibrate(depth, keep, K, D, frame_id, float(grip), color=color[-1])
    support = []
    for d, c in zip(depth2, color2):
        try:
            support.append(template.exclude(d, K, D, frame_id, float(grip), color=c)[1]["support_fraction"])
        except ValueError:
            support.append(0.)
    print("gripper position %.3f, %d gripper pixels, support on a second burst: %s" % (grip, template.mask.sum(), np.round(support, 3).tolist()))
    if min(support) < .9:
        sys.exit("NOT saved: the second burst does not match (min support %.2f). Something moved or touched the gripper: keep hands and objects away and retry." % min(support))
    path = ROOT / "config" / ("gripper_depth_%s.npz" % a.name)
    template.save(path)
    print("saved", path)


if __name__ == "__main__":
    main()
