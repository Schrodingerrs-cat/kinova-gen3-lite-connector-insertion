#!/usr/bin/env bash
# Jog the arm with the keyboard. Needs the arm driver + MoveIt running (Code/Grasp/run_ros.sh starts them).
#   ./run.sh          real motion (asks for Enter first)       ./run.sh --dry     checks only, nothing moves
cd "$(dirname "$0")"
source ./ros_env.sh
exec python3 teleop_keys.py "$@"
