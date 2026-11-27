#!/usr/bin/env bash
# Stop every ROS process that can hold the arm (driver, camera, MoveIt, RViz, grasp node, servo scripts). Then start ONE fresh set: cd Code/Grasp && ./run_ros.sh
for p in "[k]ortex_bringup" "[r]os2_control_node" "[r]obot_state_publisher" "[r]ealsense2_camera" "[m]ove_group" "[g]rasp_node" \
         "[r]viz2" "[i]nsert_servo" "[g]o_to_pose" "[r]un_ros.sh" "[g]en3_lite.launch" "[r]obot.launch" "[s]tatic_transform_publisher"; do
  pkill -f "$p"
done
sleep 3
pkill -9 -f "[k]ortex|[r]os2_control|[m]ove_group|[r]ealsense2_camera|[g]rasp_node|[i]nsert_servo|[g]o_to_pose" 2>/dev/null
sleep 1
ros2 daemon stop 2>/dev/null
if pgrep -af "kortex|ros2_control|move_group|realsense2_camera|grasp_node|insert_servo|go_to_pose" | grep -v pgrep; then echo "!! still running (above)"; else echo "all clear"; fi
