#!/usr/bin/env bash
# One command: arm link + driver + camera + MoveIt (IK / line paths only) + grasp node + RViz. Ctrl-C stops everything.
#   ./run_ros.sh                  dry run (checks every IK / line, nothing moves)
#   ./run_ros.sh dry_run:=false   real motion. Keep the e-stop within reach.
# Then in another terminal (source ros_env.sh first):  ros2 service call /grasp/<name> std_srvs/srv/Trigger
# Logs: logs/<name>.log. Check first: python3 ros2_iface/smoke_test.py
cd "$(dirname "$0")"
source ./ros_env.sh
mkdir -p logs
./arm_link.sh || exit 1

trap 'kill 0 2>/dev/null' EXIT INT TERM
wait_for() { for _ in $(seq 1 "$3"); do grep -q "$2" "logs/$1.log" 2>/dev/null && return 0; sleep 1; done; echo "!! $1: '$2' not seen in logs/$1.log"; return 1; }

ros2 daemon stop >/dev/null 2>&1
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 launch_rviz:=false > logs/driver.log 2>&1 &
wait_for driver "joint_trajectory_controller.*activated" 60 || exit 1
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true > logs/camera.log 2>&1 &
wait_for camera "Open profile" 30 || exit 1
# stock MoveIt launch, unpatched: its own controller name does not match the driver, and we do not use its execution
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10 launch_rviz:=false > logs/moveit.log 2>&1 &
wait_for moveit "You can start planning" 40 || exit 1
echo ">> arm, camera and MoveIt are up; starting the grasp node (dry_run default true)"
ros2 launch ros2_iface/launch/grasp.launch.py "$@"
