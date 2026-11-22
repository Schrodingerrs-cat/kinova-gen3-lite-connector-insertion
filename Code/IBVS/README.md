# IBVS

Image-based visual servoing for the Kinova Gen3 Lite and the wrist-mounted RealSense D405 against ArUco markers on the wall. Nothing is taught or remembered: at the post-grasp pose the camera detects every marker in view, **chooses one live** (the one closest to the image centre) and builds the goal from the live image (that marker centred, upright, at the size it has when chosen). The controller then makes small joint corrections until the four marker corners match the goal, and the arm moves forward along the measured wall normal (`--advance-m`).

The current system tests free-space alignment with the gripper only (no insertion, no seating check yet). Grasping is handled separately in [`../Grasp`](../Grasp/README.md). Tilt of the tool against the wall is not controlled separately yet (the simple approach is tried first; see "Known limits").

## Code overview

| File or folder | Purpose |
| --- | --- |
| `run_end_to_end.sh` | Close gripper → home → post-grasp pose → choose a marker live → servo → forward move. |
| `run_alignment.sh` | Runs startup → home → saved post-grasp pose → IBVS. |
| `run_joint2_priority.sh` | Runs IBVS from the current robot pose. |
| `ros2_iface/teach_goal.py` | Legacy goal teaching (used only by the harness check; the servo does not use it). |
| `ros2_iface/alignment_test.py` | ROS startup, camera feedback, planning, robot motion, and recording. |
| `ros2_iface/joint2_priority_servo.py` | The alignment loop: chooses corrections and checks convergence. |
| `ibvs/markers.py` | `AutoMarkerTracker` (live choice and goal), marker depth. |
| `ibvs/` (rest) | Control calculations, collision-scene checks, kinematics, display helpers. |
| `config/marker_goal.yaml` | Marker dictionary, optional socket-to-marker table (only to force a marker), tolerances, motion settings. |
| `config/alignment.yaml` | Base controller and motion settings. |
| `tests/` | Offline tests for control mathematics and component logic. |
| `archive/aruco_state_machine/` | Earlier ArUco state machine, guards, search and seating logic (not used by these runners). |

Home/post-grasp poses and camera-to-tool calibration come from [`../Grasp/config/grasp.yaml`](../Grasp/config/grasp.yaml) (the hand-eye there is the old one: redo it after the camera change). `ros2_iface/ibvs_node.py` and the state-machine modules in `ibvs/` are the older ArUco design; the runners below do not use them.

## Which script to run

Every command from the `insertion_task` folder. The arm driver, MoveIt and the camera (`eih_cam`) must be up first; the quickest way is any run script with `--bringup` (it starts what is missing; start one set only).

| What | Command | Moves the arm? |
| --- | --- | --- |
| Whole job: close gripper → home → post-grasp → every marker (servo, 8 cm forward, back out) | `bash Code/IBVS/run_end_to_end.sh --show-image --live --bringup` (drop `--live` for a preview) | yes |
| Go to the configured post-grasp pose (safest route from wherever the arm is) | `bash Code/IBVS/run_go_to_pose.sh` (`--pose home`, `--dry` plan only) | yes |
| Photo + depth + joint values (degrees) + visible markers of the current pose | `bash Code/IBVS/run_capture_pose.sh --name my_pose` → `Media/IBVS/poses/` | no |
| AUX (audio) socket with the goal taught for it: close gripper → home → post-grasp → marker 11 → forward 8 cm → second target `audio_insertion` (centre marker 11 as in the insertion-pose photo; `--then-goal NAME` for another saved goal; shifted by `then_goal_offset_px` in `config/marker_goal.yaml`, now 5 px down) | `bash Code/IBVS/run_aux.sh --show-image --live --bringup` (preview without `--live`) | yes |
| One marker only, from the current pose | `bash Code/IBVS/run_joint2_priority.sh --show-image --live` (`--marker-id N`) | yes |
| Home → post-grasp → one marker | `bash Code/IBVS/run_alignment.sh --show-image --live --bringup` | yes |
| Print the markers | `markers/aruco_markers.pdf` (regenerate: `python3 Code/IBVS/markers/make_markers.py`) | no |
| Offline tests | `python3 -m pytest Code/IBVS/tests -q` | no |

Useful options on the run scripts: `--speed-scale 1..3`, `--advance-m 0.08`, `--no-checks`, `--allow-contact`, `--marker-id N`, `--all-markers`. Gripper by hand: `ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.81, max_effort: 30.0}}"` (0.0 opens).

## Marker setup

Print `markers/aruco_markers.pdf` (page 1: ids 10/11/12, 30 mm black square; page 2: hand-eye marker id 0, 100 mm; regenerate with `markers/make_markers.py`; print at 100 %) and stick each marker flat on the wall above its socket. They are DICT_4X4_50 markers. No teaching step: the run picks a visible marker and builds its goal itself. `--marker-id N` or `--socket NAME` (usb_a 12, audio 11, ac 10) only force which marker is chosen. The goal puts the chosen marker at the image centre plus `goal_offset_px` (config, now 5 px down), so the marker must be placed so that centring it lines the plug up with its socket.

## How to test

Run these commands from the repository root. Hardware tests require the arm, D405, ROS 2, and the installed Kinova/MoveIt workspace.

```bash
cd <repo root>   # the insertion_task folder
source Code/IBVS/ros_env.sh
python3 -m pytest Code/IBVS/tests -q      # offline logic checks, no robot
```

Check the system without moving the arm (feedback, marker, depth, virtual reach/collision preview; a good result reports `PREVIEW`):

```bash
bash Code/IBVS/run_joint2_priority.sh --bringup --show-image --wait-target-s 30
```

Run IBVS directly from the current pose (e-stop in reach):

```bash
bash Code/IBVS/run_joint2_priority.sh --bringup --live --show-image --wait-target-s 30
```

Or home → saved post-grasp pose → IBVS (moving to the post-grasp pose does not grasp):

```bash
bash Code/IBVS/run_alignment.sh --bringup --live --show-image
```

`--bringup` starts missing ROS services, `--live` enables robot motion, `--show-image` opens the camera display. Run one motion program at a time. **Q/Esc** in the camera window or **Ctrl+C** stops the runner.

The gripper and whatever it holds are not scene: everything nearer than 18 cm to the camera (plus a 12 px halo) is dropped from the collision scene. No stored gripper reference is used any more (`ibvs/self_depth.py` and the `config/gripper_depth*.npz` files remain but the runners ignore them).

Depth is the mean measured depth inside the marker (flat paper, one depth for all four corners). A missing marker or too little valid depth causes a tracking hold.

## End to end from home

Stack up first (arm driver, MoveIt `allow_trajectory_execution:=false`, camera `eih_cam`), e-stop in reach. From the `insertion_task` folder:

```bash
source Code/IBVS/ros_env.sh
bash Code/IBVS/run_end_to_end.sh --show-image          # preview: nothing moves, prints the predicted wall clearance
bash Code/IBVS/run_end_to_end.sh --show-image --live   # close gripper -> home -> post-grasp -> for EVERY marker in view, left to right: servo -> forward 8 cm -> back out
```

`run_end_to_end.sh` uses `--all-markers`: it looks at which markers are visible at the post-grasp pose (left to right), then for each one servos to it (goal built live), moves forward, and backs out along the same path to the standoff before the next marker. It stops at the first marker that does not converge or whose forward move fails, and the preview only looks at the first marker. Without `--all-markers` (e.g. `run_alignment.sh`) it does one marker, chosen live (or forced with `--marker-id`/`--socket`). `--speed-scale` also scales the servo step size, so the lateral move between markers is not dozens of 2 mm steps.

Forward move: along the wall normal measured from depth, as straight Cartesian trajectories (MoveIt's Cartesian path service, orientation held, a joint waypoint every 5 mm, one flowing trajectory per 5 cm chunk); each chunk must be followed by the arm, and a joint jump over 5.5 deg between 5 mm points (singularity) stops the move before it starts. What the gripper holds is everything nearer than 18 cm to the camera (live), and the move stops 1.5 cm short of the wall unless `--allow-contact` is given (full stroke, for the insertion push). `--advance-m 0` turns it off.

Runs: `Media/IBVS/shape_ibvs_20261121_222939_249959` (first servo, taught-goal version, CONVERGED). The live-choice version and the home → post-grasp → forward sequence have not run on the arm yet.

## Forcing a marker, and speed

| option | marker id |
| --- | --- |
| (none) | chosen live: the visible marker closest to the image centre |
| `--socket usb_a` | 12 |
| `--socket audio` | 11 |
| `--socket ac` | 10 |
| `--marker-id N` | N |

`--speed-scale S` (1 to 3, `run_end_to_end.sh` uses 2.0) multiplies the joint, linear, angular and advance speed limits and shortens the minimum move and settle times. Positioning (home, post-grasp) and the forward move now flow through their waypoints (cubic, no stop at every MoveIt waypoint) and are timed with a 1.25 peak factor instead of the 1.875 of a stopping quintic; alignment steps still stop at their end. At 5 deg/s the home-to-post-grasp swing (about 210 deg on joint 5) was timed at about 80 s; at scale 2 the same move is timed at about 26 s (computed from the timing formula, not measured on the arm). Not yet run on the arm.

## Archive and speed changes (2026-11-20)

- `archive/aruco_state_machine/`: the earlier ArUco design (state machine B0/A1/A2, guards, bounded search, seating check, 22 tests). Not used by the runners here.
- Speed work on the marker servo (offline tests only, **not yet run on the arm**; use `loop_timing.jsonl` from a supervised run to see the effect):
  - `capture()` no longer writes `latest.png`, `latest_features.png` and a compressed `last_observation.npz` every frame (about 120 ms per frame measured on a saved 848x480 frame). The first observation is still saved once, the last one at shutdown.
  - Each step uses a straight joint path from the current joints (`Experiment.direct_path`) instead of a MoveIt planning call. Same step limit, same geometric limits, same collision service on the samples.
  - Collision samples per step use `collision_segment_deg` (default stays 0.25 in the default profile; 1.0 in `--fast`). The stationary preflight checks collisions every `preflight_check_every` simulated steps (default 5) and always at the final state.
  - Per-step joint caps are now config (`approach_limits_deg`, `fine_limits_deg`); defaults are the old fixed values.
  - `loop_timing.jsonl` now splits `choose_s`, `check_s`, `move_s`, `observation_s`.
- `--fast` (opt-in, UNVALIDATED): bigger steps (6 mm, 1.5 deg), joint/linear/angular speed 20 deg/s, 30 mm/s, 10 deg/s, 0.15 s minimum move, 0.05 s settling, gain 0.6, collision samples every 1 deg, no step-rate cap. The 3-degree guard on joints 1,3-6 and the 5.5-degree step guard are unchanged. First run supervised with the e-stop in reach, in preview first (`run_joint2_priority.sh` without `--live`).

## Expected result

The controller first adjusts image center, size and angle, then aligns the four corners to the live goal. Success requires **every corner within `tolerance_px` (5 px) for five consecutive fresh observations**. At about 18 cm standoff with the current intrinsics, 1 px is about 0.4 mm, so 5 px is about 2 mm: compare that with the connector clearance before an insertion attempt.

The final status is `CONVERGED`, or `ALREADY_ALIGNED` if no correction was needed. Each run prints its evidence directory under [`Media/IBVS`](../../Media/IBVS/README.md). Check `summary.json` for the result, `trial.csv` for measured errors, `loop_timing.jsonl` for step timing, and `ibvs_correction.mp4` for the recording.

## Known limits

- The goal is only "marker centred, upright, same size": it does not know where the socket is, so the marker placement decides whether the plug is aligned. One marker sees lateral position, distance and in-plane roll well, but tilt of the tool against the wall looks like a sideways shift. Not handled yet; if alignment looks right but the plug is off, this is the first suspect.
- The marker dictionary and the camera topic prefix (`/eih_cam`, taken from the earlier rectangle config) are defaults to confirm on the real setup.
- Nothing here has run on the arm with a marker yet.
