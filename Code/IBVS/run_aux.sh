#!/usr/bin/env bash
# AUX (audio) socket with the goal that was taught for it: close gripper -> home -> post-grasp -> servo to marker 11 -> forward 15 cm
#   -> second target audio_insertion (centre marker 11 as in Media/IBVS/poses/hand_moved_1_*/aux_insertion point.png).
#   bash Code/IBVS/run_aux.sh --show-image            preview, nothing moves
#   bash Code/IBVS/run_aux.sh --show-image --live     real motion, e-stop in reach (add --bringup if the driver is not running)
set -e
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 ros2_iface/alignment_test.py --socket audio --taught --close-gripper --advance-m 0.15 --then-goal audio_insertion --speed-scale 2.0 --no-checks "$@"
