#!/usr/bin/env bash
# End to end from ANY pose: home -> post-grasp -> look at the markers -> servo the wire's marker (pre-insertion, brief stop, insertion).
#   bash Code/IBVS/run_insert.sh aux|usb|power [--continuous] [--speed 0.05]
#   bash Code/IBVS/run_insert.sh all           aux, then usb, then power; waits for Enter between wires (swap the plug, close the gripper on it)
# Needs the arm driver, MoveIt and the grasp stack's camera up. Wire -> marker: aux 11, usb 12, power 10 (config/insertion_refs.yaml).
cd "$(dirname "$0")"
[[ "$1" =~ ^(aux|usb|power|all)$ ]] || { echo "usage: bash Code/IBVS/run_insert.sh aux|usb|power|all [--continuous] [--speed 0.05]"; exit 2; }
source ./ros_env.sh
w="$1"; shift
set -o pipefail

run_one() {
  local folder="$(cd ../.. && pwd)/Media/IBVS/run_$(date +%Y%m%d_%H%M%S)_insert_$1"      # run folder: config, console log, csv, video, plots
  mkdir -p "$folder"
  {
    bash ./go_post_grasp.sh --pose home &&              # safest route from wherever the arm is (restarts a stale driver if needed)
    bash ./go_post_grasp.sh &&                          # home -> leveled post-grasp pose
    python3 ros2_iface/insert_servo.py --wire "$1" --out "$folder" "${@:2}"
  } 2>&1 | tee "$folder/controller.log"
}

if [ "$w" != all ]; then run_one "$w" "$@"; exit $?; fi
for wire in aux usb power; do
  [ "$wire" != aux ] && read -r -p ">> done with the last wire. Swap the plug, close the gripper on the $wire plug, then press Enter (Ctrl-C to stop): "
  run_one "$wire" "$@" || { echo ">> $wire stopped: not continuing with the next wire"; exit 1; }
done
echo ">> all three wires done"
