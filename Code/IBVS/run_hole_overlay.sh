#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/hole_overlay.py "$@"
