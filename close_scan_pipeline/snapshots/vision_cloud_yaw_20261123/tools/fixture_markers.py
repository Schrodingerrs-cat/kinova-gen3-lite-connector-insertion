#!/usr/bin/env python3
"""Publishes the CAD fixture (setup_cad/parts/*.stl) as RViz markers on /fixture/markers in the base_link frame. Pose from setup_cad/fixture_pose.yaml, re-read every 2 s.
   python3 ~/kinova/work/tools/fixture_markers.py        (started by run_stack_method2.sh and open_rviz.sh; needs ROS sourced)
"""
import pathlib

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as Rot
from visualization_msgs.msg import Marker, MarkerArray

D = pathlib.Path.home() / "kinova/work/setup_cad"


class Fixture(Node):
    def __init__(self):
        super().__init__("fixture_markers")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/fixture/markers", qos)
        self.parts = yaml.safe_load(open(D / "parts_index.yaml"))
        self.create_timer(2.0, self.tick)
        self.tick()

    def tick(self):
        pose = yaml.safe_load(open(D / "fixture_pose.yaml"))
        c, t, yaw = np.array(pose["anchor_m"], float), np.array(pose["offset_m"], float), float(pose["yaw_deg"])
        R = Rot.from_euler("z", yaw, degrees=True)
        pos = c - R.apply(c) + t                                    # p_base = Rz (p_nominal - c) + c + t
        q = R.as_quat()
        out = MarkerArray()
        for k, p in enumerate(self.parts):
            mount = p["centre_m"][0] < 0.12                         # robot mount parts stay on the real arm base, only the cradle side is fitted
            m = Marker()
            m.header.frame_id, m.header.stamp = "base_link", self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "fixture", k, Marker.MESH_RESOURCE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = [0.0, 0.0, 0.0] if mount else [float(v) for v in pos]
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = [0.0, 0.0, 0.0, 1.0] if mount else [float(v) for v in q]
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = [float(v) for v in p["rgba"]]
            m.mesh_resource = "file://" + str(D / "parts" / p["file"])
            m.mesh_use_embedded_materials = False
            out.markers.append(m)
        self.pub.publish(out)


rclpy.init()
rclpy.spin(Fixture())
