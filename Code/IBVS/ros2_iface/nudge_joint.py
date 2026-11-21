#!/usr/bin/env python3
"""Move ONE joint by a few degrees (to start a servo run from a small known offset).
  python3 ros2_iface/nudge_joint.py --joint 1 --deg 3        --dry prints the move without sending it"""
import argparse

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 7)]

ap = argparse.ArgumentParser()
ap.add_argument("--joint", type=int, required=True, choices=range(1, 7))
ap.add_argument("--deg", type=float, required=True)
ap.add_argument("--dry", action="store_true")
a = ap.parse_args()
if abs(a.deg) > 10:
    raise SystemExit("nudges are limited to 10 deg")
rclpy.init()
node = Node("nudge_joint")
state = {}
node.create_subscription(JointState, "/joint_states", lambda m: state.update(zip(m.name, m.position)), 10)
while not all(j in state for j in JOINTS):
    rclpy.spin_once(node, timeout_sec=0.2)
q0 = np.array([state[j] for j in JOINTS])
q1 = q0.copy()
q1[a.joint - 1] += np.radians(a.deg)
T = max(3.0, abs(a.deg) / 4 + 1)
print("joint %d: %.2f -> %.2f deg over %.1f s" % (a.joint, np.degrees(q0[a.joint - 1]), np.degrees(q1[a.joint - 1]), T))
if not a.dry:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    for s in np.linspace(0, 1, 9)[1:]:       # quintic: zero velocity at both ends
        t = 0.1 + T * s
        p = JointTrajectoryPoint(positions=(q0 + (q1 - q0) * (10*s**3 - 15*s**4 + 6*s**5)).tolist(),
                                 velocities=((q1 - q0) * (30*s**2 - 60*s**3 + 30*s**4) / T).tolist())
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points.append(p)
    client = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
    assert client.wait_for_server(timeout_sec=5), "no trajectory action server"
    handle = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, handle)
    result = handle.result().get_result_async()
    rclpy.spin_until_future_complete(node, result)
    print("error_code", result.result().result.error_code, result.result().result.error_string)
rclpy.shutdown()
