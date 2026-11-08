# Teleop: move the arm with the keyboard

Self-contained (own `arm.py`, `robot_ros.py`, `teleop.yaml`, `ros_env.sh`). Needs the Gen3 Lite driver and MoveIt running (IK service) and the same DDS settings in every terminal (`ros_env.sh`). The camera is NOT needed.

```bash
./run.sh --dry    # checks every key (IK, joint-jump, table floor), nothing moves
./run.sh          # real motion: press Enter, keep the e-stop in reach
```
Keys: `w/s a/d r/f` move the gripper tip +x/-x, +y/-y, +z/-z (base frame); `t/g y/h q/e` tilt it in place (pitch, roll, yaw); `1`-`6` then `[` `]` jog one joint;
`o` / `c` open / close the gripper by 10% of its range per press; `+` `-` step size (1-50 mm or deg); `0` or `H` go home; Ctrl-C quits.
Limits: one key may not move any joint more than 30 deg; targets below the table floor are refused.
Do not run it while another program is moving the arm. `arm.py` / `robot_ros.py` are copies of the ones in `Code/Grasp/ros2_iface/`.
