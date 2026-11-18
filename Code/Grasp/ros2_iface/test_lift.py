import sys
import numpy as np
import rclpy
from robot_ros import Robot

rclpy.init()
robot = Robot("test_descent")
try:
    if not robot.wait_ready(10):
        raise RuntimeError("Robot feedback or controllers unavailable")

    start = robot.tool_matrix()
    expected = np.array([0.2684, -0.0226, 0.0323])

    # Prevent repeated downward steps from a different starting position.
    if np.linalg.norm(start[:3, 3] - expected) > 0.003:
        raise RuntimeError("Not at the checked pregrasp position; stop")

    goal = start.copy()
    goal[2, 3] += 0.010

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
        robot.send_line(path, 5.0)
        print("Measured tool XYZ:",
              np.round(robot.tool_matrix()[:3, 3], 4))
    else:
        print("PLAN ONLY: no movement commanded")
finally:
    robot.destroy_node()
    rclpy.shutdown()
