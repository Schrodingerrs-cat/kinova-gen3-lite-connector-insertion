#!/usr/bin/env bash
# Take the robot to the post-grasp pose (from grasp.yaml) and stop:  bash Code/IBVS/go_post_grasp.sh
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/go_to_pose.py --pose post_grasp
