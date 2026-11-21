#!/usr/bin/env bash
# Close gripper -> home -> leveled post-grasp pose -> servo to the socket's marker -> move forward 15 cm along the wall normal.
#   bash Code/IBVS/run_end_to_end.sh --socket audio --show-image            preview, nothing moves (prints the predicted wall clearance)
#   bash Code/IBVS/run_end_to_end.sh --socket audio --show-image --live     real motion, e-stop in reach
# Extra args pass through, e.g. --speed-scale 1.5 (default 2.0), --advance-m 0.10, --allow-contact (full stroke even if the plug reaches the wall), --fast.
set -e
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/alignment_test.py --close-gripper --advance-m 0.15 --speed-scale 2.0 "$@"
