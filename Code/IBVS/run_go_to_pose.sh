#!/usr/bin/env bash
# Move to the configured post_grasp (default) or home pose by the safest joint route:  bash Code/IBVS/run_go_to_pose.sh [--pose home] [--dry]
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/go_to_pose.py "$@"
