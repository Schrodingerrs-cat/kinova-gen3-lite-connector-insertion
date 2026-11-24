#!/usr/bin/env bash
# method 2's grasp node (worktree ~/kinova/ins/method2_src) on our Humble stack: arm link, driver, camera (enable_sync), MoveIt, method 2's node. Dry run by default. Ctrl-C stops all.
#   ~/kinova/work/tools/run_stack_method2.sh            dry_run:=true
MINE=$HOME/kinova/ins/repo/Code/Grasp
cd "$HOME/kinova/ins/method2_src/Code/Grasp" || exit 1
source "$MINE/ros_env.sh"
mkdir -p logs/humble
"$MINE/arm_link.sh" || exit 1

trap 'kill 0 2>/dev/null' EXIT INT TERM
wait_for() { for _ in $(seq 1 "$3"); do grep -q "$2" "logs/humble/$1.log" 2>/dev/null && return 0; sleep 1; done; echo "!! $1: '$2' not seen in logs/humble/$1.log"; return 1; }

ros2 daemon stop >/dev/null 2>&1
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 launch_rviz:=false > logs/humble/driver.log 2>&1 &
wait_for driver "joint_trajectory_controller.*activated" 60 || exit 1
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true enable_sync:=true > logs/humble/camera.log 2>&1 &
wait_for camera "Open profile" 30 || exit 1
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10 launch_rviz:=false > logs/humble/moveit.log 2>&1 &
wait_for moveit "You can start planning" 40 || exit 1
python3 "$HOME/kinova/work/tools/add_fixture_to_scene.py" > logs/humble/scene.log 2>&1 && echo ">> CAD fixture and table are in the MoveIt planning scene (arm collision only, no plugs)" || echo "!! fixture not added to the planning scene (see logs/humble/scene.log)"
echo ">> arm, camera and MoveIt are up; starting method 2's grasp node (dry_run default true)"
# our saved RViz scene (arm + CAD fixture + camera image) replaces the grasp node's own RViz windows; pass rviz:=false to start none
OURS=1; for a in "$@"; do [ "$a" = "rviz:=false" ] && OURS=0; done
if [ "$OURS" = 1 ]; then
  python3 "$HOME/kinova/work/tools/fixture_markers.py" > logs/humble/fixture.log 2>&1 &
  QT_QPA_PLATFORM=xcb rviz2 -d "$HOME/kinova/work/rviz/setup.rviz" > logs/humble/rviz.log 2>&1 &
fi
ros2 launch ros2_iface/launch/grasp.launch.py rviz:=false "$@"
