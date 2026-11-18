# Source this file in every terminal used by this checkout.
_connector_code_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_connector_workspace="$(cd -- "$_connector_code_dir/../.." && pwd)/kinova_ws"
source /opt/ros/jazzy/setup.bash || return 1
source "${KINOVA_WS:-$_connector_workspace}/install/setup.bash" || return 1
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
if [ -f "$_connector_code_dir/../.venv/bin/activate" ]; then
  source "$_connector_code_dir/../.venv/bin/activate"
fi
unset _connector_code_dir _connector_workspace
