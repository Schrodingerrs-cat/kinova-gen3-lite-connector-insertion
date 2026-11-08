"""Read-only check of the ROS 2 stack (joint states, TF, action servers, camera, IK); never moves the arm."""
import os
import sys

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_ros import Robot

rclpy.init()
r = Robot("grasp_smoke")
print('joint states + arm/gripper action servers ready:', r.wait_ready(10.0))
if r.q is not None:
    print('joints (deg):', np.round(r.joints_deg(), 1))
    T = r.tool_matrix(); print('tool xyz (m):', np.round(T[:3, 3], 3))
try:
    color, depth, K = r.grab()
    print('camera:', color.shape, depth.shape, 'depth median %.3f m' % np.median(depth[depth > 0]), 'fx %.1f' % K[0, 0])
except Exception as e:
    print('camera:', e)
print('IK service up:', r.ik_srv.wait_for_service(timeout_sec=3.0), '| cartesian path service up:', r.cart_srv.wait_for_service(timeout_sec=3.0))
if r.q is not None and r.ik_srv.service_is_ready():
    q = r.ik(r.tool_matrix())
    print('IK of the current pose ->', None if q is None else np.round(q, 1), '(should match the joints above)')
rclpy.shutdown()
