#!/usr/bin/env bash
# Teach a wire's goal from the real arm (no motion): put the arm by hand at the pose, then
#   bash Code/IBVS/teach_insert.sh usb insert     plug at the socket entrance
#   bash Code/IBVS/teach_insert.sh usb pre        standoff before it
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/teach_insert.py --wire "$1" --stage "$2"
