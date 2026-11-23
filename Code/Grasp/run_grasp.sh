#!/bin/bash
# Short way to run the pipeline (the run folder is made for you):
#   ./run_grasp.sh strip usb_a aux            perception only
#   ./run_grasp.sh --grasp strip usb_a aux    also execute the grasp(s)
# first word after the optional --grasp = mode (head|cloud|model|cad|strip), the rest = connectors
cd "$(dirname "$0")" && source ros_env.sh
GRASP=""
[ "$1" = "--grasp" ] && GRASP="--grasp" && shift
MODE="$1"; shift
[ -z "$MODE" ] && { echo "usage: ./run_grasp.sh [--grasp] MODE [CONNECTOR ...]"; exit 1; }
CONN=(); [ $# -gt 0 ] && CONN=(--connector "$@")
exec python3 ros2_iface/run_pipeline.py "$PWD/../../Media/Grasp/run_$(date +%Y%m%d_%H%M%S)_$MODE" --mode "$MODE" "${CONN[@]}" $GRASP
