# IBVS

Image-based visual servoing for the Kinova Gen3 Lite and the wrist-mounted RealSense D405. One **ArUco marker per socket**, placed above it. The camera tracks the marker's four corners, compares them with the **taught** goal corners for that socket, and the controller makes small joint corrections until the corners match. Because the goal is taught with the plug held aligned at the pre-insertion pose, matching the marker image means the plug is aligned with the socket.

The current system tests free-space alignment with the gripper only (no insertion, no seating check yet). Grasping is handled separately in [`../Grasp`](../Grasp/README.md). Tilt of the tool against the wall is not controlled separately yet (the simple approach is tried first; see "Known limits").

## Code overview

| File or folder | Purpose |
| --- | --- |
| `run_teach_goal.sh` | Saves the goal corners of one socket's marker (no robot motion). |
| `run_alignment.sh` | Runs startup → home → saved post-grasp pose → IBVS. |
| `run_joint2_priority.sh` | Runs IBVS from the current robot pose. |
| `ros2_iface/teach_goal.py` | Camera-only goal teaching. |
| `ros2_iface/alignment_test.py` | ROS startup, camera feedback, planning, robot motion, and recording. |
| `ros2_iface/joint2_priority_servo.py` | The alignment loop: chooses corrections and checks convergence. |
| `ibvs/markers.py` | ArUco corner tracking, marker depth, goal averaging and saving. |
| `ibvs/` (rest) | Control calculations, collision-scene checks, kinematics, display helpers. |
| `config/marker_goal.yaml` | Marker dictionary, marker id per socket, tolerances, motion settings. |
| `config/taught_goals.yaml` | Taught goal corners per socket (written by the teach tool). |
| `config/alignment.yaml` | Base controller and motion settings. |
| `tests/` | Offline tests for control mathematics and component logic. |
| `archive/aruco_state_machine/` | Earlier ArUco state machine, guards, search and seating logic (not used by these runners). |

Home/post-grasp poses and camera-to-tool calibration come from [`../Grasp/config/grasp.yaml`](../Grasp/config/grasp.yaml) (the hand-eye there is the old one: redo it after the camera change). `ros2_iface/ibvs_node.py` and the state-machine modules in `ibvs/` are the older ArUco design; the runners below do not use them.

## Marker setup and teaching

1. Print `markers/aruco_markers.pdf` (page 1: ids 10/11/12, 30 mm black square; page 2: hand-eye marker id 0, 100 mm; regenerate with `markers/make_markers.py`; print at 100 %). These are DICT_4X4_50 markers with the ids from `sockets:` in `config/marker_goal.yaml` (`usb_a` 10, `audio` 11, `ac` 12; id 0 is the hand-eye calibration marker). Stick each flat, in the socket's plane, above its socket, and do not move it afterwards.
2. For each socket, jog the arm (Code/Teleop) until the plug is aligned at the pre-insertion pose, keep it still, then:

```bash
bash Code/IBVS/run_teach_goal.sh --socket audio
```

It needs the marker in 30 consecutive frames with corner std below 0.5 px, then writes `config/taught_goals.yaml` and `config/taught_goal_audio.png`. Re-teach after any camera, mount or marker change: the runner refuses a goal taught with different intrinsics.

## How to test

Run these commands from the repository root. Hardware tests require the arm, D405, ROS 2, and the installed Kinova/MoveIt workspace.

```bash
cd <repo root>   # the insertion_task folder
source Code/IBVS/ros_env.sh
python3 -m pytest Code/IBVS/tests -q      # offline logic checks, no robot
```

Check the system without moving the arm (feedback, marker, depth, virtual reach/collision preview; a good result reports `PREVIEW`):

```bash
bash Code/IBVS/run_joint2_priority.sh --socket audio --bringup --show-image --wait-target-s 30
```

Run IBVS directly from the current pose (e-stop in reach):

```bash
bash Code/IBVS/run_joint2_priority.sh --socket audio --bringup --live --show-image --wait-target-s 30
```

Or home → saved post-grasp pose → IBVS (moving to the post-grasp pose does not grasp):

```bash
bash Code/IBVS/run_alignment.sh --socket audio --bringup --live --show-image
```

`--bringup` starts missing ROS services, `--live` enables robot motion, `--show-image` opens the camera display. Run one motion program at a time. **Q/Esc** in the camera window or **Ctrl+C** stops the runner.

The measured `config/gripper_depth*.npz` references identify the visible fingers by color and depth; matching finger pixels and farther returns occluded by them are excluded from collision snapshots. Closer returns or changed appearance stay obstacles. A different camera setup, gripper opening, or poor calibration match blocks the run.

Depth is the mean measured depth inside the marker (flat paper, one depth for all four corners). A missing marker or too little valid depth causes a tracking hold.

## End to end from home (what the first hardware run did, plus the home move)

Stack up first (arm driver, MoveIt `allow_trajectory_execution:=false`, camera `eih_cam`), gripper open and empty, e-stop in reach. From the `insertion_task` folder:

```bash
source Code/IBVS/ros_env.sh
bash Code/IBVS/run_alignment.sh --socket audio --show-image                 # preview: no motion
bash Code/IBVS/run_alignment.sh --socket audio --show-image --live          # home -> leveled post-grasp pose -> servo to marker 11
```

The post-grasp pose (`Code/Grasp/config/grasp.yaml`, `post_grasp`) was leveled on the wall with the camera (evidence: `Media/IBVS/run_*_leveling_post_grasp/`) and the goal in `config/taught_goals.yaml` was taught there, so the arm arrives almost aligned and the run may end `ALREADY_ALIGNED` with 0 steps. To exercise the servo, offset the arm first (the first hardware run used joint 1 by +3 deg) and run from the current pose:

```bash
python3 Code/IBVS/ros2_iface/nudge_joint.py --joint 1 --deg 3
bash Code/IBVS/run_joint2_priority.sh --socket audio --show-image --wait-target-s 30 --live
```

First hardware run: `Media/IBVS/shape_ibvs_20261121_222939_249959` (CONVERGED, 8 steps, 12.2 s). The home -> post-grasp move through `run_alignment.sh` has not been run with markers yet.

## Archive and speed changes (2026-11-20)

- `archive/aruco_state_machine/`: the earlier ArUco design (state machine B0/A1/A2, guards, bounded search, seating check, 22 tests). Not used by the runners here.
- Speed work on the marker servo (offline tests only, **not yet run on the arm**; use `loop_timing.jsonl` from a supervised run to see the effect):
  - `capture()` no longer writes `latest.png`, `latest_features.png` and a compressed `last_observation.npz` every frame (about 120 ms per frame measured on a saved 848x480 frame). The first observation is still saved once, the last one at shutdown.
  - Each step uses a straight joint path from the current joints (`Experiment.direct_path`) instead of a MoveIt planning call. Same step limit, same geometric limits, same collision service on the samples.
  - Collision samples per step use `collision_segment_deg` (default stays 0.25 in the default profile; 1.0 in `--fast`). The stationary preflight checks collisions every `preflight_check_every` simulated steps (default 5) and always at the final state.
  - Per-step joint caps are now config (`approach_limits_deg`, `fine_limits_deg`); defaults are the old fixed values.
  - `loop_timing.jsonl` now splits `choose_s`, `check_s`, `move_s`, `observation_s`.
- `--fast` (opt-in, UNVALIDATED): bigger steps (6 mm, 1.5 deg), joint/linear/angular speed 20 deg/s, 30 mm/s, 10 deg/s, 0.15 s minimum move, 0.05 s settling, gain 0.6, collision samples every 1 deg, no step-rate cap. The 3-degree guard on joints 1,3-6 and the 5.5-degree step guard are unchanged. First run supervised with the e-stop in reach, in preview first (`run_joint2_priority.sh --socket NAME` without `--live`).

## Expected result

The controller first adjusts image center, size and angle, then aligns the four corners to the taught goal. Success requires **every corner within `tolerance_px` (5 px) for five consecutive fresh observations**. At about 18 cm standoff with the current intrinsics, 1 px is about 0.4 mm, so 5 px is about 2 mm: compare that with the connector clearance before an insertion attempt.

The final status is `CONVERGED`, or `ALREADY_ALIGNED` if no correction was needed. Each run prints its evidence directory under [`Media/IBVS`](../../Media/IBVS/README.md). Check `summary.json` for the result, `trial.csv` for measured errors, `loop_timing.jsonl` for step timing, and `ibvs_correction.mp4` for the recording.

## Known limits

- One marker sees lateral position, distance and in-plane roll well, but tilt of the tool against the wall looks like a sideways shift. Not handled yet; if alignment looks right but the plug is off, this is the first suspect.
- Marker ids, the dictionary and the camera topic prefix (`/eih_cam`, taken from the earlier rectangle config) are defaults to confirm on the real setup.
- Nothing here has run on the arm with a marker yet.
