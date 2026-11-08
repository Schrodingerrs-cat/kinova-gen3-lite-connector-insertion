# IBVS + bounded search (Task A)

Implements `plan/Task_A_IBVS_Search_PLAN.md` work packages A2-A6 in software. Hardware: Kinova Gen3 Lite, wrist D405, ArUco markers beside the socket.

## Layout
```
ibvs/                 pure NumPy/OpenCV, no ROS (all unit-tested)
  config.py           refuses null values; contact=True also refuses UNCALIBRATED and unset safety flags
  frames.py           pose matrix, twist adjoint
  markers.py          ArUco corners -> normalized points; validity = size + frame-to-frame jump
  depth.py            per-corner depth with validity, fallback = median of valid corners, none valid = abort
  control.py          interaction matrix, damped pseudoinverse, 3-DoF law, speed cap + slew limit, convergence counter
  search.py           bounded candidate list (nominal first, Chebyshev ring, then prior-sigma distance)
  guards.py           servo stop checks (stale camera/joints, workspace box, timeout, marker loss), axial step progress
  verify.py           INSERTED_VERIFIED / NOT_INSERTED / UNKNOWN_SENSOR / FAULT from the independent sensor, dwell + staleness
  state_machine.py    the plan's state machine for B0 / A1 / A2 over injected primitives
  logger.py           CSV with the plan's data columns
ros2_iface/ibvs_node.py   ROS 2 wiring: camera, Twist publisher, controller switch, guarded axial steps, taught pick
config/ibvs.yaml     every value null / false until measured
tests/                python3 -m pytest tests -q
```

## Method (what is and is not IBVS)
- Features: ArUco corners, undistorted to normalized coordinates; error `e = s - s*`, `s*` captured at the safe pre-contact goal.
- Law: `u = -lambda (L A)^+ W e` with `A` selecting lateral x_S, y_S and yaw about z_S (axes measured in the camera frame, `ibvs.socket_axes_in_C`). Other DoFs are not commanded.
- Depth Z only enters `L` (**hybrid** IBVS, not pure). Standoff and roll/pitch come from the taught geometry.
- Search offsets are applied after `servo_align` returns to `s*`, retracted, in the socket frame, rotating about the camera origin like the servo yaw. The plug-tip pivot (`T_EP`) is not modelled.
- Insertion is short position-controlled steps along z_S; each step checks robot follow-through (not force). No force-guided search.
- Success is only the independent sensor (`verification.topic`, std_msgs/Bool). Pose or depth never counts.

## Run
```bash
source ../Grasp/ros_env.sh            # same DDS settings as every other terminal; driver + camera up (../Grasp/run_ros.sh)
python3 ros2_iface/ibvs_node.py servo                 # dry run: logs the commands it would send, publishes nothing
python3 ros2_iface/ibvs_node.py servo --live          # non-contact servo (needs servo-section config)
python3 ros2_iface/ibvs_node.py cycle A2 --live       # full cycle; needs status CALIBRATED, sensor tests passed, supervisor flags
```
Output goes to `runs/<ts>/trial.csv`. Copy real runs, with video, into `Media/IBVS/` per `Media/RULES.md`.

## Status (2026-11-07)
**Not run on the arm.** 22 synthetic tests pass (closed-loop convergence with correct signs, interaction matrix vs finite differences, guards, state machine, config refusal, ArUco detection on rendered markers). These are logic checks, not hardware evidence; per `CLAUDE.md` the task is NOT DONE until the real arm shows the closed loop and the sensor verifies seating.

Blocking items (all `null` in `config/ibvs.yaml`): K / distortion / `T_EC` (HW7 hand-eye in `../Grasp/config/grasp.yaml` is an unverified estimate), marker layout and `s*`, `socket_axes_in_C`, twist topic + controller names + command frame, speed/travel/search limits, waypoints, seating sensor and its positive/negative tests, e-stop/supervisor.

Assumptions to verify on the arm before trusting them:
- `picknik_twist_controller` takes `geometry_msgs/Twist` (header confirmed); topic name, frame, and watchdog on stop are NOT confirmed.
- Controller switching uses `start_controllers`/`stop_controllers` of `/controller_manager/switch_controller`.
- Aligned depth is 16UC1 millimetres; the node reads the color/depth/camera_info topics hardcoded in `../Grasp/ros2_iface/robot_ros.py`.
- Taught pick uses joint waypoints; the grasp-held cue is gripper position only, to be validated by the 20-pick repeatability set.
