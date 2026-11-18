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

## Expected result

The target is a **200×200 px square in an 848×480 image**. The controller first adjusts image center and size, then aligns the four corners. Success requires **every corner within 5 px for five consecutive fresh observations**.

The final status is `CONVERGED`, or `ALREADY_ALIGNED` if no correction was needed. Each run prints its evidence directory under [`Media/IBVS`](../../Media/IBVS/README.md). Check `summary.json` for the result, `trial.csv` for measured errors, and `ibvs_correction.mp4` for the recording.
