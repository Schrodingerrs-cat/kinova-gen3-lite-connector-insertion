#!/usr/bin/env bash
# End to end from ANY pose: home -> post-grasp -> look at the markers -> servo the wire's marker (pre-insertion, brief stop, insertion).
#   bash Code/IBVS/run_insert.sh aux|usb|power [--continuous] [--speed 0.05]
# Needs the arm driver, MoveIt and the grasp stack's camera up. Wire -> marker: aux 11, usb 12, power 10 (config/insertion_refs.yaml).
cd "$(dirname "$0")"
[[ "$1" =~ ^(aux|usb|power)$ ]] || { echo "usage: bash Code/IBVS/run_insert.sh aux|usb|power [--continuous] [--speed 0.05]"; exit 2; }
source ./ros_env.sh
w="$1"; shift
bash ./go_post_grasp.sh --pose home || exit $?          # safest route from wherever the arm is (restarts a stale driver if needed)
bash ./go_post_grasp.sh || exit $?                      # home -> leveled post-grasp pose
exec python3 ros2_iface/insert_servo.py --wire "$w" "$@"
