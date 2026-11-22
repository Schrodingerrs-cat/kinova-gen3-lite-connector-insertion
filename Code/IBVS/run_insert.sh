#!/usr/bin/env bash
# Two-stage marker servo for one wire (after the grasp, from the post-grasp pose):  bash Code/IBVS/run_insert.sh aux|usb|power [--speed 0.05]
cd "$(dirname "$0")"
source ./ros_env.sh
w="$1"; shift
exec python3 ros2_iface/insert_servo.py --wire "$w" "$@"
