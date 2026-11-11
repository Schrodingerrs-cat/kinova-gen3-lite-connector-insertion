# Keyboard teleop

This is discrete pose jogging, not continuous velocity control. It uses **both Cartesian targets and joint control**, depending on the key:

- `w/s a/d r/f`: move the TCP +x/-x, +y/-y, +z/-z in the **robot base frame**, keeping its orientation fixed.
- `t/g y/h q/e`: rotate about base y, x, z respectively, keeping the TCP position fixed.
- `1`–`6`, then `[` / `]`: jog the selected joint directly, without IK. Selecting a joint does not change the Cartesian keys.

Cartesian keys ask MoveIt for an exact full-pose IK solution seeded at the current joints. The resulting motion interpolates **joint angles**; it does not guarantee a straight TCP path. A reachable position does not guarantee that the requested position plus orientation is solvable. No IK solution is not proof that a pose is physically unreachable.

Needs the Gen3 Lite driver and MoveIt `/compute_ik` service, with the same DDS settings in every terminal (`ros_env.sh`). No camera or Cartesian path service is needed for keyboard jogging.

```bash
./run.sh --dry    # solve Cartesian targets / print joint targets; no motion
./run.sh          # real motion: press Enter to start
```

`--dry` uses the current measured pose for each press; it does not simulate accumulated movement or validate controller acceptance of joint targets.

- `+` / `-`: change the step (1, 2, 5, 10, 20, 50 mm for translation; the same number in degrees for rotation/joints). Starts at 5.
- `o` / `c`: open / close by 10% of the gripper range.
- `p`: append measured joint angles and gripper position to `saved_poses.json`, including in dry mode.
- `0` / `H`: home via a joint move.
- `?`: display help. Ctrl-C quits.

Each press blocks until the command finishes. Keys queued during a move are discarded, so holding a key is not continuous jogging. Status identifies Cartesian or joint commands, the step units, and the selected joint.

IK rejections now include MoveIt's error name/code. `NO_IK_SOLUTION` means the solver did not find the requested full pose; try a smaller step or explicitly jog a joint to change configuration. `TIMED_OUT` means the solver exhausted its budget. A service-response timeout indicates a communication/service issue. `EXECUTION FAILED` reports a controller failure after target selection. Small joint moves are sent rather than silently dropped by the previous 0.3-degree threshold.

No teleop table floor or joint-jump limit is enabled, and IK collision checking is disabled. Keep the e-stop in reach and do not run another motion program simultaneously. These local `arm.py` / `robot_ros.py` files originated as copies from `Code/Grasp/ros2_iface/`; teleop fixes are local.

Offline regression checks (ROS Python packages required; no robot commands):

```bash
source ros_env.sh
python3 -m unittest discover -s . -p 'test_*.py'
```
