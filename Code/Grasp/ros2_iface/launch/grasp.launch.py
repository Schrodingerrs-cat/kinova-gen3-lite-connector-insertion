"""ros2 launch ros2_iface/launch/grasp.launch.py [dry_run:=true] [rviz:=true] [candidate_search:=config|on|off] [grasp_mode:=head|cloud]

Grasp node + camera TF into the robot tree + RViz. Needs the arm driver, MoveIt (IK / line paths) and
realsense2_camera already running (run_ros.sh starts them), every terminal with `source ros_env.sh`.
"""
import pathlib

import numpy as np
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scipy.spatial.transform import Rotation as Rot

HERE = pathlib.Path(__file__).resolve().parents[1]
# RealSense: camera_link -> color optical frame is a fixed rotation published by the camera driver
Q_LINK_OPTICAL = [-0.5, 0.5, -0.5, 0.5]


def hand_eye_tf():
    """tool_frame -> camera_link, so the RealSense TF tree (and its live cloud) hangs off the robot."""
    cam = yaml.safe_load(open(HERE.parent / "config" / "grasp.yaml"))["camera"]
    X = np.eye(4); X[:3, :3] = Rot.from_quat(cam["hand_eye_quat_xyzw"]).as_matrix(); X[:3, 3] = cam["hand_eye_xyz_m"]
    L = np.eye(4); L[:3, :3] = Rot.from_quat(Q_LINK_OPTICAL).as_matrix()
    T = X @ np.linalg.inv(L)
    q = Rot.from_matrix(T[:3, :3]).as_quat()
    return [str(v) for v in (*T[:3, 3], *q)]


def generate_launch_description():
    x, y, z, qx, qy, qz, qw = hand_eye_tf()
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("candidate_search", default_value="config", description="config | on | off"),
        DeclareLaunchArgument("grasp_mode", default_value="head", description="mode of /grasp/pick: head | cloud"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="hand_eye_tf",
             arguments=["--x", x, "--y", y, "--z", z, "--qx", qx, "--qy", qy, "--qz", qz, "--qw", qw,
                        "--frame-id", "tool_frame", "--child-frame-id", "camera_link"]),
        # forced X11: RViz segfaults under Wayland on this machine. Window 1: robot + camera image, window 2: point clouds
        ExecuteProcess(cmd=["rviz2", "-d", str(HERE / "grasp.rviz")], additional_env={"QT_QPA_PLATFORM": "xcb"},
                       condition=IfCondition(LaunchConfiguration("rviz"))),
        ExecuteProcess(cmd=["rviz2", "-d", str(HERE / "grasp_clouds.rviz")], additional_env={"QT_QPA_PLATFORM": "xcb"},
                       condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(executable=str(HERE / "grasp_node.py"), name="grasp_node", output="screen",
             parameters=[{"dry_run": LaunchConfiguration("dry_run"), "candidate_search": LaunchConfiguration("candidate_search"), "grasp_mode": LaunchConfiguration("grasp_mode")}]),
    ])
