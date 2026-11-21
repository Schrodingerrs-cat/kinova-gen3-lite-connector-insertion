#!/usr/bin/env bash
# Source in every terminal used for the alignment test.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  source /opt/ros/humble/setup.bash
fi
if [ -f /home/sri/kinova_ws/install/setup.bash ]; then
  source /home/sri/kinova_ws/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
