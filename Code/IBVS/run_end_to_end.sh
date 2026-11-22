#!/usr/bin/env bash
# Close gripper -> home -> leveled post-grasp pose -> then for EVERY marker in view, left to right: servo to it -> forward 10 cm along the wall normal -> back out.
#   bash Code/IBVS/run_end_to_end.sh --show-image            preview, nothing moves (prints the predicted wall clearance)
#   bash Code/IBVS/run_end_to_end.sh --show-image --live     real motion, e-stop in reach
# Nothing is taught. For a single marker use alignment_test.py/run_alignment.sh without --all-markers (optionally --marker-id N / --socket NAME). Extra args pass through, e.g. --speed-scale 1.5 (default 2.0), --advance-m 0.10, --allow-contact (full stroke even if the plug reaches the wall), --fast.
set -e
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/alignment_test.py --close-gripper --all-markers --advance-m 0.10 --speed-scale 2.0 --no-checks "$@"
