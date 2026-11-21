#!/usr/bin/env bash
# Take the robot to the post-grasp pose (from grasp.yaml) and stop.   bash Code/IBVS/go_post_grasp.sh   (--dry: only print the route)
# If no joint values arrive (driver missing, or a stale driver after the arm was power-cycled) it stops the old driver, starts a fresh
# one (moves nothing) and tries once more.
cd "$(dirname "$0")"
source ./ros_env.sh
python3 ros2_iface/go_to_pose.py --pose post_grasp "$@"
rc=$?
[ $rc -eq 3 ] || exit $rc
echo "restarting the arm driver (the old one is stopped first; nothing moves)"
pkill -f kortex_bringup; pkill -f ros2_control_node; pkill -f robot_state_publisher; sleep 3
bash ../Grasp/arm_link.sh || exit 1
setsid nohup ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=false launch_rviz:=false \
  > "${TMPDIR:-/tmp}/ibvs_arm_driver.log" 2>&1 &
for _ in $(seq 1 45); do ros2 node list 2>/dev/null | grep -q controller_manager && break; sleep 1; done
sleep 5
exec python3 ros2_iface/go_to_pose.py --pose post_grasp "$@"
