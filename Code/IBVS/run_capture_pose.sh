#!/usr/bin/env bash
# Photo + joint values of the current pose, no motion:  bash Code/IBVS/run_capture_pose.sh --name hand_moved_1
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/capture_pose.py "$@"
