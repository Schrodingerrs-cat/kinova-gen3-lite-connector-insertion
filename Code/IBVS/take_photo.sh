#!/usr/bin/env bash
# Take one photo with the wrist camera:  bash Code/IBVS/take_photo.sh
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/take_photo.py
