# IBVS




  Do this, in order:
  1. Physical check: make sure the arm is powered, the e-stop is released, and the
     USB cable to the PC is seated. If the arm's light shows a fault, clear it in the
     Kinova web app or power-cycle the arm. Wait about a minute for it to boot.
  2. Stop everything stale:
  cd ~/Documents/GitHub/VBRM_insertion/insertion_task
  bash Code/IBVS/kill_all.sh
  3. Bring the link and driver back up. run_ros.sh runs arm_link.sh, which will ask
     for your sudo password:
  cd ~/Documents/GitHub/VBRM_insertion/insertion_task/Code/Grasp
  ./run_ros.sh
  Wait for ">> arm, camera and MoveIt are up", and make sure there is no "driver ...
  not seen" line before it.
  4. Confirm it answers before moving anything, in a second terminal:
  ping -c 2 192.168.1.10
  source ~/Documents/GitHub/VBRM_insertion/insertion_task/Code/IBVS/ros_env.sh
  ros2 topic echo /joint_states --once
  5. Then run the servo:
  bash Code/IBVS/run_insert.sh aux --speed 0.03

  
Image-based visual servoing for the Kinova Gen3 Lite and the wrist-mounted RealSense D405 against ArUco markers on the wall. Nothing is taught or remembered: at the post-grasp pose the camera detects every marker in view, **chooses one live** (the one closest to the image centre) and builds the goal from the live image (that marker centred, upright, at the size it has when chosen). The controller then makes small joint corrections until the four marker corners match the goal, and the arm moves forward along the measured wall normal (`--advance-m`).

The current system tests free-space alignment with the gripper only (no insertion, no seating check yet). Grasping is handled separately in [`../Grasp`](../Grasp/README.md). Tilt of the tool against the wall is not controlled separately yet (the simple approach is tried first; see "Known limits").

## Run this: insertion servo (current pipeline, smooth Cartesian, per wire)

This is the pipeline to use. Everything under "Old pipeline" below (run_end_to_end.sh, run_aux.sh, run_joint2_priority.sh, run_alignment.sh) is the older joint step-and-observe servo: slow, one small joint step at a time, and it matches marker rotation (`align_orientation`), which the new one does not. After the grasp, from the post-grasp pose, it servos the wire's marker to the pre-insertion image, stops 1 s, then servos to the insertion image (plug at the socket entrance). The robot only translates (orientation held), no camera tilting. Marker per wire: aux = 11, usb = 12, power = 10. Reference pixels per wire: `config/insertion_refs.yaml` (`nudge_px: [x, y]` shifts the insertion target in the image to fine-tune).

Terminal by terminal (every terminal: `cd .../insertion_task && source Code/IBVS/ros_env.sh`, one arm driver only):

1. Terminal 1, the arm and MoveIt (the grasp stack's bringup, the same one the grasp pipeline uses; wait until the controllers are up).
2. Terminal 2, the camera as the grasp stack starts it (`/camera/camera/...` with aligned depth; this script reads these topics).
3. Terminal 3, one command does everything from any arm pose: home, post-grasp, look at the markers, servo the wire's marker (pre-insertion, 1 s stop, insertion). It does not grasp; run it after the grasp, with the plug held.

| Mode | Command |
| --- | --- |
| Continuous (default): re-plans on every camera frame while moving, slows into the goal, never stops to look | `bash Code/IBVS/run_insert.sh aux` (or `usb`, `power`, `all`) |
| Stepwise: quintic chunks of at most 4 cm, each ending at rest, marker re-checked between chunks | `bash Code/IBVS/run_insert.sh aux --stepwise` |
| All three wires one after another (aux, usb, power; waits for Enter between wires so you can swap the plug; stops if one fails) | `bash Code/IBVS/run_insert.sh all` |
| Sideways-first gate during the insertion approach (mm, default 15; 0 = off): forward motion only starts once the sideways error is below this, so the plug is lined up before it goes toward the wall | `--align-mm 15` |
| Back-off after reaching the socket entrance (m, default 0.15; 0 = stay): straight back out along the camera axis before the arm goes home | `--back-off-m 0.15` |
| Slower / faster (m/s, default 0.05) | `--speed 0.03` |
| Chunk length in default mode (m, default 0.04) | `--step-m 0.02` |
| Finish tolerance (mm, default 2) and stop at pre-insertion (s, default 1) | `--tol-mm 1.5 --pause 2` |

Every run is recorded in `Media/IBVS/run_<time>_insert_<wire>/` (layout of `Media/RULES.md`): `config.json` (exactly what was run: command, arguments, wire references and nudge, hand-eye, start joints, git commit), `controller.log` (console), `trajectory.csv` (every observation: marker pixels, offsets, commanded step, tool pose, joints), `metrics.json`, `screenshots/`, `videos/servo.mp4` (green marker, red goal) and `plots/` (offset to goal, distance to go, marker size, marker path in the image, tool position, joints, commanded step). Redraw the plots: `python3 Code/IBVS/ros2_iface/run_record.py <run folder>`. If a run fails, add `failure_analysis.md` and move the folder to `Media/IBVS/Failed_runs/`.

Contact: there is no force sensor, so the servo stops (and holds) when the arm does not follow a command, i.e. it moved less than half of a final move, or under 12% of a cruise piece. Motion is a quintic time law along each straight line, started at the speed the arm has, so replacing a trajectory does not jerk.

If MoveIt has no straight path for a step, the arm goes home by the safe route and runs the same line from there. If the marker is lost it stops (after 6 frames in a row in the default mode). Do not edit files while it runs. 

Camera: `insert_servo.py` reads the grasp stack's `/camera/camera/...` topics (check with `ros2 topic list | grep image_raw`). The old scripts expect `/eih_cam/...`.

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

## Old pipeline: which script to run (slow, joint step-and-observe, kept for reference)

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

## Old pipeline: end to end from home

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
