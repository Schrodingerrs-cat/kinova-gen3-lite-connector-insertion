"""Open the gripper (fingers only, arm never commanded): python3 ros2_iface/open_gripper.py"""
import os
import sys

import rclpy
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm import Arm
from robot_ros import Robot

rclpy.init()
cfg = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'grasp.yaml')))
arm = Arm(Robot('grasp_open', background=True), None, None, 0.0, cfg['gripper']['close_cmd'])
assert arm.r.wait_ready(10.0) and arm.r.spin_until(lambda: arm.r.grip_pos_msr is not None, 5.0), 'no driver'
print('gripper before: %.3f rad' % arm.grip_pos())
print('gripper now:    %.3f rad (0 = open)' % arm.grip_to(0.0, 20.0, rate=0.3)[0])
rclpy.shutdown()
