# Run 20261103_173928: plug attempt 1 (FAILED)
- **Date:** 2026-11-03 | **Controller:** analytic housing grasp (Task A grasp front end) + MoveIt Cartesian pick | **Platform:** REAL Kinova Gen3 Lite + D405 (eye-in-hand), no simulation
- **Object:** black 2-prong US power plug (unknown dimensions to the code) on a wooden table
- **Code version:** no git repo; `Code/Grasp/` as of this date. The plots here re-analyse the saved frame with the CURRENT pipeline (cleaned cloud); the attempt itself ran the PRE-cleaning pipeline.
- **Calibration:** HW7 hand-eye and 0.035 m tool tip offset (same robot/camera; not re-verified). Finger size and gripper position->width table are PLACEHOLDERS (collision check / open width only).
- **Video:** none recorded (UNKNOWN timing; only frames below).

## Objective / hypothesis
Pick the plug housing from an ROI with the dimension-free pipeline; hypothesis: planned jaws centred on the body close on the housing.

## Result: `FAILED`
MoveIt planned and executed pre-grasp, straight descent, close and lift without errors. The gripper closed to 0.800 (fully closed, empty threshold 0.790): verifier says nothing was held. The wrist view afterwards shows the plug no longer where it was (displaced by the fingers); the planned centre projects ~10-15 mm beyond the plug body.
See `failure_analysis.md`.

## Evidence
- `screenshots/01_planning_view_with_roi.png`, `02_wrist_view_after_attempt1.png`, `03_planned_grasp_point_projected_into_after_view.png`
- `plots/01..04` (raw vs cleaned cloud, segmentation + grasp, cross-section at grasp, housing profile)
- `pointclouds/` (`frame.npz` raw depth+colour+K+camera pose, `scene_cloud.npy`/`housing_cloud.npy` as saved by the node), `metrics.json`, `metrics_pipeline.json`, `config.json`, `controller.log`

## Does not prove
That the pipeline's housing estimate was wrong (the estimate is stable, see `../../run_20261103_roi_size_invariance_us_plug/`); only that the executed pick did not hold the housing.
