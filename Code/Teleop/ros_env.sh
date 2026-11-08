# Source in EVERY terminal that touches the robot (driver, camera, MoveIt, grasp node, rviz2, service calls). Same DDS settings everywhere,
# else nodes silently fail to see each other (HW7/docs/TROUBLESHOOTING.md 3.1, 3.2). After a change: restart all ROS terminals, `ros2 daemon stop`.
# Why these lines exist: docs/TROUBLESHOOTING.md sections 3.1 and 3.2.
#
#   source tools/ros_env.sh
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
