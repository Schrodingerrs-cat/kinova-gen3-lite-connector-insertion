# Multiview CAD fitting and physical USB-A pickup

This branch contains the ROS 2 Jazzy setup, calibration diagnostics, CAD fitting tools, and supervised Cartesian pickup workflow developed on the Gen3 Lite with a wrist-mounted D405.

## Geometry and fitting

`tools/fit_cad_multiview.py` loads `CAD_files/USB_A_Plug.glb`, samples 40,000 mesh surface points, crops the model, and converts it to the connector coordinate convention in millimeters. Geometry has fixed scale. It segments housing/sticker depth in each image ROI, filters depth edges, and transforms points into robot-base coordinates. The nearby-view pipeline uses raw calibrated camera poses without table ICP.

Each view reserves 20% of points for same-view pixel validation and contributes at most 400 training points. `tools/fit_cad_cloud.py:register` tries 24 yaw starts, performs trimmed nearest-neighbor rigid alignment, then refines six pose parameters using robust least squares. One pose is fitted jointly across all views. The objective measures observed-to-model surface agreement, so hidden surfaces do not require observations. Partial surfaces can leave pose ambiguous.

The fitted `T_base_plug` transforms precomputed CAD grasp candidates into robot coordinates. The colored sticker locates the search region. The batch fitter does not use the ArUco tag to determine connector pose.

## Setup and entry points

Follow [SETUP_JAZZY.md](../../SETUP_JAZZY.md). In `Code/Grasp`, source `ros_env.sh` and start `./run_ros.sh dry_run:=true rviz:=false`.

The full nearby-view runner is `ros2_iface/run_fresh_nearby_pickup.py`. Without flags it prints help and does not move. `--plan-scan --sideways` checks the four camera route legs without moving. `--execute --sideways` moves, captures three fresh views, returns home, fits the CAD, and requests pickup only if its checks pass. The default fit profile is `strict`. The demonstrated run explicitly selected `--fit-profile supervised_pickup`, an experimental profile rather than an accuracy certification.

Stored camera poses, calibration, dimensions, and workspace settings belong to the original lab setup. Revalidate them on any changed setup. The runner requires project home and an empty fully open gripper. Robot movement is only requested by explicit execution flags. No release or insertion is included.

## Demonstration and limits

The November 15 demonstration used perception run `1794789916872077379` and motion run `1794789954005073051`. Three views contributed 1,426 segmented points. Held-out-pixel median/p95 surface errors were 1.15/2.57, 1.25/2.90, and 0.61/3.94 mm. The experimental profile requires at least 70% within 2 mm, p95 at most 5 mm, and yaw spread at most 15 degrees. The strict profile requires 80% and 10 degrees and would reject that demonstration.

The supervised sequence approached from 100 mm standoff, closed the gripper, and commanded a 5 mm test lift plus 50 mm extraction. Physical holding was visually confirmed. This is one documented demonstration, not a measured success rate or insertion result. Surface residuals do not establish absolute pose accuracy. Cartesian checks do not establish scene/cable collision clearance. Gripper encoder readings did not independently verify holding or contact force.

Figures and captions are in `presentation_assets/2026-11-15_successful_pickup/`. Raw scans, runtime logs, local diagnostic captures, and backup files are excluded from version control. The older saved procedure in `procedure_records/` describes an earlier staged test.

## Offline checks

From repository root:

```bash
MPLCONFIGDIR=/tmp/mpl .venv/bin/python Code/Grasp/tools/test_fit_cad_cloud.py
.venv/bin/python Code/Grasp/tools/test_cad_execution.py
.venv/bin/python -m pytest Code/Calibration/tests/test_capture_quality.py -q
```
