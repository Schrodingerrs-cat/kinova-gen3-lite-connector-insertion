#!/usr/bin/env bash
# Jazzy: one MoveIt launch owns the driver and controller manager.
set -eo pipefail
cd "$(dirname "$0")"
source ./ros_env.sh
mkdir -p logs/jazzy
./arm_link.sh

# Refuse duplicate hardware stacks; stop the old launch terminals first.
existing="$(timeout 8 ros2 node list --no-daemon 2>/dev/null || true)"
if printf '%s\n' "$existing" | grep -Eq '^/(controller_manager|move_group|camera/camera|grasp_node)$'; then
  echo 'A robot/camera stack is already running. Stop its launch terminals before using run_ros.sh.'
  exit 1
fi
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
wait_for() {
  local name="$1" pattern="$2" seconds="$3" pid="$4"
  for ((i=0; i<seconds; i++)); do
    if grep -q "$pattern" "logs/jazzy/$name.log" 2>/dev/null; then return 0; fi
    if ! kill -0 "$pid" 2>/dev/null; then tail -40 "logs/jazzy/$name.log"; return 1; fi
    sleep 1
  done
  echo "Startup timeout: see logs/jazzy/$name.log"; tail -40 "logs/jazzy/$name.log"; return 1
}
ros2 launch ros2_iface/launch/robot_jazzy.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=false launch_rviz:=false > logs/jazzy/moveit.log 2>&1 &
pids+=("$!")
wait_for moveit 'You can start planning' 60 "${pids[0]}"
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true enable_sync:=true > logs/jazzy/camera.log 2>&1 &
pids+=("$!")
wait_for camera 'RealSense Node Is Up' 45 "${pids[1]}"
echo 'Robot and camera are up. Grasp node defaults to dry_run:=true.'
ros2 launch ros2_iface/launch/grasp.launch.py "$@" &
pids+=("$!")
wait "${pids[2]}"
