import sys
import numpy as np
import rclpy
from robot_ros import Robot
import json
from pathlib import Path
from scipy.spatial.transform import Rotation
import yaml

rclpy.init()
robot = Robot("test_descent")
try:
    if not robot.wait_ready(10):
        raise RuntimeError("Robot feedback or controllers unavailable")

    start = robot.tool_matrix()
    expected = np.array([0.2672, -0.0227, 0.0817])

    # Prevent repeated downward steps from a different starting position.
    if np.linalg.norm(start[:3, 3] - expected) > 0.003:
        raise RuntimeError("Not at the checked pregrasp position; stop")

    if robot.grip_pos_msr is None:
        raise RuntimeError("No gripper feedback")
    if abs(robot.grip_pos_msr - 0.6068) > 0.02:
        raise RuntimeError("Gripper opening changed; stop")

    root = Path(__file__).resolve().parents[1]
    plan_path = (
        root.parents[1] / "Media/Grasp/work/runs"
        / "20261115_065819/plan.json"
    )
    saved = json.loads(plan_path.read_text())
    candidate = saved["grasps"][1]
    if candidate["flip"] != -1:
        raise RuntimeError("Unexpected candidate")

    with open(root / "config/grasp.yaml") as stream:
        gripper = yaml.safe_load(stream)["gripper"]

    tool_tcp = np.eye(4)
    tool_tcp[:3, :3] = Rotation.from_quat(
        gripper["tcp_in_tool_quat_xyzw"]
    ).as_matrix()
    tool_tcp[:3, 3] = gripper["tcp_in_tool_xyz_m"]

    goal = np.array(candidate["T_BG"]) @ np.linalg.inv(tool_tcp)

    angle = Rotation.from_matrix(
        start[:3, :3].T @ goal[:3, :3]
    ).magnitude()
    if np.degrees(angle) > 2:
        raise RuntimeError("Target orientation differs from current pose")
    if np.linalg.norm(goal[:2, 3] - start[:2, 3]) > 0.005:
        raise RuntimeError("Lateral correction exceeds 5 mm")
    if not 0 < start[2, 3] - goal[2, 3] < 0.055:
        raise RuntimeError("Unexpected descent distance")

    path = robot.line_path(goal)
    if path is None or len(path) < 2:
        raise RuntimeError("No complete Cartesian path")

    joints = np.vstack([robot.q, np.asarray(path)])
    if not np.isfinite(joints).all():
        raise RuntimeError("Invalid joint values")
    if np.max(np.abs(np.diff(joints, axis=0))) > np.radians(5):
        raise RuntimeError("Joint jump detected")

    print("Start tool XYZ:", np.round(start[:3, 3], 4))
    print("Goal tool XYZ:", np.round(goal[:3, 3], 4))
    print("Path points:", len(path))

    if "--execute" in sys.argv:
        robot.send_line(path, 15.0)
        print("Measured tool XYZ:",
              np.round(robot.tool_matrix()[:3, 3], 4))
    else:
        print("PLAN ONLY: no movement commanded")
finally:
    robot.destroy_node()
    rclpy.shutdown()
