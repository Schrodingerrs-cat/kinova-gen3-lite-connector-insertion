# IBVS

This folder contains image-based visual servoing for the Kinova Gen3 Lite and wrist-mounted RealSense D405. The camera detects a rectangle, compares its corners with a desired image target, and the controller makes small joint corrections until the corners align.

The current system tests free-space alignment with an **open, empty gripper**. Grasping is handled separately in [`../Grasp`](../Grasp/README.md).

## Code overview

| File or folder | Purpose |
| --- | --- |
| `run_alignment.sh` | Runs startup → home → saved post-grasp pose → IBVS. |
| `run_joint2_priority.sh` | Runs IBVS from the current robot pose. |
| `ros2_iface/alignment_test.py` | Handles ROS startup, camera feedback, planning, robot motion, and recording. |
| `ros2_iface/joint2_priority_servo.py` | Runs the alignment loop, chooses corrections, and checks convergence. |
| `ibvs/` | Rectangle tracking, control calculations, depth processing, collision-scene checks, and display helpers. |
| `config/alignment.yaml` | Base controller and motion settings. |
| `config/center_rectangle.yaml` | Desired rectangle, tracking settings, and alignment tolerances. |
| `tests/` | Offline tests for control mathematics and component logic. |

Home/post-grasp poses and camera-to-tool calibration come from [`../Grasp/config/grasp.yaml`](../Grasp/config/grasp.yaml). The older ArUco insertion/search implementation remains in `ros2_iface/ibvs_node.py`; the commands below use the current rectangle-alignment system.

## How to test

Run these commands from the repository root. Hardware tests require the arm, D405, ROS 2, and the installed Kinova/MoveIt workspace.

```bash
cd /home/sri/kinova-gen3-lite-connector-insertion-master
source Code/IBVS/ros_env.sh
```

### 1. Run the offline tests

```bash
python3 -m pytest Code/IBVS/tests -q
```

These tests check component logic without moving the robot. Use the next steps to test the integrated system.

### 2. Check the system without moving the arm

```bash
bash Code/IBVS/run_joint2_priority.sh --bringup --show-image --wait-target-s 30
```

This checks feedback, rectangle detection, depth, and a virtual reach/collision preview at the current pose. It sends no motion trajectory. A successful preview reports `PREVIEW`.

### 3. Run home → post-grasp → IBVS

```bash
bash Code/IBVS/run_alignment.sh --bringup --live --show-image
```

This moves to the saved home and post-grasp poses, then performs visual alignment. Moving to the post-grasp pose does not perform a grasp.

### 4. Run IBVS directly from the current pose

Use this instead of step 3 when positioning is already complete:

```bash
bash Code/IBVS/run_joint2_priority.sh --bringup --live --show-image --wait-target-s 30
```

`--bringup` starts missing ROS services, `--live` enables robot motion, and `--show-image` opens the camera display. The shell scripts source the ROS environment automatically. Run one motion program at a time. Press **Q/Esc** in the camera window or **Ctrl+C** to stop the runner; background ROS services may remain running.

## Acrylic sheet: choose one of three hole rectangles

The new mode uses **four hole centers** as its features. Rectangles **1, 2, 3** are numbered **left to right when acquired**, with each group ordered top-left → top-right → bottom-right → bottom-left. The labels stay fixed while tracking.

First check the camera-only overlay:

```bash
bash Code/IBVS/run_hole_overlay.sh --bringup --rectangle 1
```

Press **1/2/3** to choose a rectangle in this preview. All three groups are outlined; the selected group is highlighted. Automatic acquisition needs all 12 holes visible in three side-by-side groups. If grouping is unclear, use:

```bash
bash Code/IBVS/run_hole_overlay.sh --bringup --calibrate
```

Click four holes from each rectangle, completing one group before the next. Group order does not matter: the software numbers them left to right. **S** saves the initial groups in `config/hole_rectangles.yaml`; **R** resets and right-click undoes a point. Repeat this setup if the starting camera pose or sheet position changes. Quit the overlay before starting an IBVS runner.

Then preview or run IBVS from the current pose, choosing **1**, **2**, or **3**:

```bash
# Stationary preview
bash Code/IBVS/run_hole_rectangles.sh --bringup --show-image --rectangle 2 --wait-target-s 30
# Live IBVS
bash Code/IBVS/run_hole_rectangles.sh --bringup --show-image --rectangle 2 --wait-target-s 30 --live
```

The live runner keeps that selection for the run. `ibvs/hole_rectangles.py` detects and tracks hole centers; `ros2_iface/hole_overlay.py` handles preview/setup. Edit `config/hole_rectangles.yaml` for hole brightness, diameter limits, the default selection, and desired image corners. Set `seed_rectangles_image_px: null` to return to automatic acquisition. The desired image target is now a centered 100×100 px square, selected after collision previews. It can be changed independently of the measured holes.

Click setup also saves color templates for the 12 holes, including pale holes. Keep the generated template file with its YAML configuration. To return to automatic acquisition, remove `template_file` as well as clearing the seed points.

The measured `config/gripper_depth*.npz` references identify the visible fingers by color and depth. The best matching independently measured reference is used for the current pose. Matching finger pixels and farther returns occluded by those fingers are excluded from collision snapshots. Closer returns or changed appearance remain obstacles; occluded areas remain unobserved. A different camera setup, gripper opening, or poor calibration match blocks the run. Collision checks still include the entire robot.

Depth is fitted around the selected hole rims, excluding the open holes. Check that measured depth corresponds to the acrylic sheet: transparent acrylic can return background depth. Missing hole features or insufficient depth cause a tracking hold.

## Expected result

For the smoother trial, use `bash Code/IBVS/run_hole_rectangles.sh --bringup --home-first --live --show-image --rectangle 1 --smooth --control-hz 2`. This adds image-angle correction, removes intermediate stops on checked straight servo paths, and reduces repeated settling. The requested 2 Hz is a maximum step rate; `loop_timing.jsonl` and `summary.json` report the rate actually achieved. The trial uses 8°/s joint positioning, 8 mm/s translation, and 3°/s camera rotation, with the existing step and collision limits.

The original mode targets a **200×200 px square**; the calibrated three-rectangle mode targets **100×100 px**, both in an 848×480 image. The controller first adjusts image center and size, then aligns the four corners. Success requires **every corner within 5 px for five consecutive fresh observations**.

The final status is `CONVERGED`, or `ALREADY_ALIGNED` if no correction was needed. Each run prints its evidence directory under [`Media/IBVS`](../../Media/IBVS/README.md). Check `summary.json` for the result, `trial.csv` for measured errors, and `ibvs_correction.mp4` for the recording.
