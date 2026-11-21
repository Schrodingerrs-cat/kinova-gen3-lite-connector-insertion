#!/usr/bin/env bash
# Take the robot to the post-grasp pose (from grasp.yaml) and stop. Starts the arm driver if it is not running.
#   bash Code/IBVS/go_post_grasp.sh          (--dry: only print the route)
cd "$(dirname "$0")"
source ./ros_env.sh
if ! ros2 node list 2>/dev/null | grep -q controller_manager; then
  echo "arm driver is not running: starting it (this moves nothing)"
  bash ../Grasp/arm_link.sh || exit 1
  setsid nohup ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=false launch_rviz:=false \
    > "${TMPDIR:-/tmp}/ibvs_arm_driver.log" 2>&1 &
  for _ in $(seq 1 45); do ros2 node list 2>/dev/null | grep -q controller_manager && break; sleep 1; done
  ros2 node list 2>/dev/null | grep -q controller_manager || { echo "driver did not come up; see ${TMPDIR:-/tmp}/ibvs_arm_driver.log"; exit 1; }
  sleep 4
fi
exec python3 ros2_iface/go_to_pose.py --pose post_grasp "$@"
