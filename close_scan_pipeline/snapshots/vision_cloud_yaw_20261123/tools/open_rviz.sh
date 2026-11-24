#!/usr/bin/env bash
# Opens RViz with the saved scene (arm + CAD fixture + camera image) on a running stack. Starts the fixture publisher if it is not running.
#   ~/kinova/work/tools/open_rviz.sh
source ~/kinova/ins/repo/Code/Grasp/ros_env.sh
pgrep -f "tools/fixture_markers.py" >/dev/null 2>&1 || python3 ~/kinova/work/tools/fixture_markers.py >/dev/null 2>&1 &
QT_QPA_PLATFORM=xcb exec rviz2 -d ~/kinova/work/rviz/setup.rviz
