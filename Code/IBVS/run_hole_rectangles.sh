#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/joint2_priority_servo.py --reference "$PWD/config/hole_rectangles.yaml" "$@"
