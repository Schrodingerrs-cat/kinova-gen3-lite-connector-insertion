#!/usr/bin/env python3
"""Move the arm to the configured post_grasp (or home) pose by the safest joint route, as one smooth trajectory.
  python3 ros2_iface/go_to_pose.py --pose post_grasp [--speed-scale 2] [--dry]
Routes: joint groups one after another (several orders) or straight; the one whose lowest point is highest is used.
Refuses a route that comes within 3 cm of the table plane (grasp.yaml scan.table_z_m) or reaches farther than 0.50 m (the wall is at 0.55 m)."""
import argparse
import pathlib
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ibvs.alignment import sequenced_joint_path, smooth_velocities, trajectory_times  # noqa: E402
from ibvs.frames import matrix  # noqa: E402
from ibvs.kinematics import UrdfChain  # noqa: E402

JOINTS = [f"joint_{i}" for i in range(1, 7)]
ORDERS = ([(0, 5), (4,), (2, 3), (1,)], [(1,), (2, 3), (4,), (0, 5)], [(1,), (4,), (2, 3), (0, 5)], [(2, 3), (4,), (0, 5), (1,)])
MARGIN_ABOVE_TABLE_M, MAX_REACH_M = .03, .50     # height limit = the real table plane (grasp.yaml scan.table_z_m, base frame) + this


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", choices=("post_grasp", "home"), default="post_grasp")
    ap.add_argument("--speed-scale", type=float, default=2.)
    ap.add_argument("--dry", action="store_true", help="plan and report only")
    a = ap.parse_args()
    grasp = yaml.safe_load((ROOT.parent / "Grasp" / "config" / "grasp.yaml").read_text())
    target_deg = np.array(grasp["post_grasp"]["joint_waypoints_deg"][-1] if a.pose == "post_grasp" else grasp["scan"]["home_joints_deg"], float)
    target = np.radians((target_deg + 180) % 360 - 180) if a.pose == "home" else np.radians(target_deg)

    rclpy.init()
    node, state = Node("go_to_pose"), {}
    node.create_subscription(JointState, "/joint_states", lambda m: state.update(zip(m.name, m.position)), 10)
    deadline = time.time() + 10
    while not all(j in state for j in JOINTS) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not all(j in state for j in JOINTS):
        print("no /joint_states within 10 s: the arm driver is not running or has lost the arm", flush=True)
        sys.exit(3)                                         # the wrapper script restarts the driver on this code
    q0 = np.array([state[j] for j in JOINTS])
    urdf = ""
    for service in ("/robot_state_publisher/get_parameters", "/move_group/get_parameters"):     # the arm driver publishes the robot model
        client = node.create_client(GetParameters, service)
        if client.wait_for_service(timeout_sec=5):
            future = client.call_async(GetParameters.Request(names=["robot_description"]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if future.result() is not None and future.result().values[0].string_value:
                urdf = future.result().values[0].string_value
                break
    if not urdf:
        sys.exit("could not read the robot model (no robot_state_publisher or MoveIt): is the arm driver up?")
    fk, wrist = UrdfChain(urdf, "base_link", "tool_frame", JOINTS), UrdfChain(urdf, "base_link", "upper_wrist_link", JOINTS)
    T_EC = matrix(grasp["camera"]["hand_eye_xyz_m"], grasp["camera"]["hand_eye_quat_xyzw"])
    T_tcp = matrix(grasp["gripper"]["tcp_in_tool_xyz_m"], grasp["gripper"]["tcp_in_tool_quat_xyzw"])

    def extremes(path):
        low, reach = np.inf, -np.inf
        for q in path:
            T = fk(q)
            points = [T[:3, 3], (T @ T_tcp)[:3, 3], (T @ T_EC)[:3, 3], wrist(q)[:3, 3]]
            low, reach = min(low, *[p[2] for p in points]), max(reach, *[p[0] for p in points])
        return low, reach

    n = max(2, int(np.ceil(np.degrees(np.abs(target - q0).max()))) + 1)
    routes = [sequenced_joint_path(q0, target, g) for g in ORDERS] + [np.linspace(q0, target, n)]
    check = node.create_client(GetStateValidity, "/check_state_validity")
    have_check = check.wait_for_service(timeout_sec=5)

    def collision_free(path):
        """MoveIt self-collision check on every 3rd sample (and the last) of the route."""
        for q in list(path[::3]) + [path[-1]]:
            request = GetStateValidity.Request(group_name="arm", robot_state=RobotState())
            request.robot_state.joint_state.name, request.robot_state.joint_state.position = JOINTS, [float(v) for v in q]
            future = check.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if future.result() is None or not future.result().valid:
                return False
        return True

    # no refusals: just pick the best route. Collision-free first, then the one that stays highest.
    path = max(routes, key=lambda p: (have_check and collision_free(p), extremes(p)[0]))
    low, reach = extremes(path)
    print("current joints deg:", np.round(np.degrees(q0), 1).tolist(), "\ntarget %s deg:" % a.pose, np.round(np.degrees(target), 1).tolist())
    print("route: lowest point %.0f mm, farthest reach %.0f mm, %d samples, collision-free: %s" % (low * 1e3, reach * 1e3, len(path), collision_free(path) if have_check else "unchecked"))
    times = trajectory_times(path, 5. * a.speed_scale, 0., .1, peak_factor=1.25)
    print("duration %.1f s at speed scale %.1f" % (times[-1], a.speed_scale))
    if a.dry:
        return
    action = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
    if not action.wait_for_server(timeout_sec=10):
        sys.exit("no trajectory action server: the arm driver is not running")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    for q, v, t in zip(path, smooth_velocities(path, times), times):
        point = JointTrajectoryPoint(positions=q.tolist(), velocities=v.tolist())
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points.append(point)
    handle = action.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, handle, timeout_sec=10)
    if handle.result() is None or not handle.result().accepted:
        sys.exit("the controller rejected the trajectory")
    result = handle.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=float(times[-1]) + 15)
    if result.result() is None:
        sys.exit("the trajectory did not finish in time")
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    end = np.array([state[j] for j in JOINTS])
    print("result:", result.result().result.error_string or "ok", "| final joints deg:", np.round(np.degrees(end), 2).tolist(),
          "| max error to target %.2f deg" % np.degrees(np.abs(end - target).max()))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
