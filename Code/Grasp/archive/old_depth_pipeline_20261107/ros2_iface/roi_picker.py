#!/usr/bin/env python3
"""Drag a box around the cable head in the live color image; publishes it on /grasp/roi."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, RegionOfInterest

TOPIC = "/camera/camera/color/image_raw"


def main():
    rclpy.init()
    node = Node("roi_picker")
    frame = {}
    node.create_subscription(Image, TOPIC, lambda m: frame.__setitem__("m", m), qos_profile_sensor_data)
    pub = node.create_publisher(RegionOfInterest, "/grasp/roi", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    while "m" not in frame and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    m = frame["m"]
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    if m.encoding == "rgb8":
        img = img[:, :, ::-1]
    x, y, w, h = cv2.selectROI("drag a box on the connector head, Enter to confirm", np.ascontiguousarray(img))
    if w > 0 and h > 0:
        pub.publish(RegionOfInterest(x_offset=int(x), y_offset=int(y), width=int(w), height=int(h)))
        rclpy.spin_once(node, timeout_sec=0.5)
        print("ROI published:", x, y, w, h)


if __name__ == "__main__":
    main()
