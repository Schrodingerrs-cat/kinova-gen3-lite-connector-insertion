"""Joint move to scan.home_joints_deg (the only thing here that moves the arm): python3 ros2_iface/go_home.py"""
import os
import sys

import numpy as np
import rclpy
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from arm import Arm
from robot_ros import Robot

cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'grasp.yaml')))
home = (np.array(cfg['scan']['home_joints_deg']) + 180.0) % 360.0 - 180.0
rclpy.init()
arm = Arm(Robot('grasp_home', background=True), cfg['motion'], None, 0.0, cfg['gripper']['close_cmd'])
assert arm.r.wait_ready(10.0), 'no joint states / controller: driver up? same ros_env.sh?'
d = np.abs(home - arm.q_deg())
print('joints now (deg):', np.round(arm.q_deg(), 1), '\nhome      (deg):', np.round(home, 1), '\nlargest joint move: %.0f deg' % d.max(), flush=True)
arm.go_joints(home)
print('at home, max joint error %.2f deg' % np.abs(home - arm.q_deg()).max())
rclpy.shutdown()
