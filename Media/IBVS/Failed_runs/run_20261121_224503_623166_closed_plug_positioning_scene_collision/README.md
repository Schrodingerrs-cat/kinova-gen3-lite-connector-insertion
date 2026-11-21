# Failed run: stopped before any motion (positioning scene collision with the held plug)

`run_end_to_end.sh --socket audio --show-image --live`, first run with the gripper closed on the plug (gripper_depth_closed_plug.npz, support 0.997). The gripper closed (position 0.835), then the run stopped while building the collision scene for the home move. The arm did not move.
