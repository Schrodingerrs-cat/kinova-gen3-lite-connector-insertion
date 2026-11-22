#!/usr/bin/env python3
"""Save one photo from the wrist camera. Nothing else."""
import pathlib
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

ROOT = pathlib.Path(__file__).resolve().parents[1]
configured = yaml.safe_load((ROOT / "config" / "marker_goal.yaml").read_text())["camera_topic_prefix"].rstrip("/")
prefixes = [configured] + [p for p in ("/camera/camera",) if p != configured]       # the camera runs under one naming or the other
rclpy.init()
node, got = Node("take_photo"), {}
for prefix in prefixes:
    node.create_subscription(Image, prefix + "/color/image_raw", lambda m: got.update(image=m), QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
deadline = time.time() + 10
while "image" not in got and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if "image" not in got:
    raise SystemExit("no image on %s: is the camera running?" % " or ".join(p + "/color/image_raw" for p in prefixes))
m = got["image"]
img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
path = ROOT.parents[1] / "Media" / "IBVS" / "photos" / time.strftime("photo_%Y%m%d_%H%M%S.png")
path.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(path), img[:, :, ::-1] if m.encoding == "rgb8" else img)
print(path)
