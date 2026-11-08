# run_20261107_140835_green_marker_pipeline_grasp  (FAILED at grasp planning; no grasp attempted)

- 2026-11-07 14:08, real arm + D405 (USB 2.1). Objective: scan -> merge/ICP -> table RANSAC/DBSCAN -> green marker -> close view -> housing grasp plan -> gates -> grasp (`ros2_iface/run_green_pipeline.py --grasp`).
- Status: **FAILED** (stopped by the gates before any grasp motion: `STOPPED_BEFORE_GRASP: plan failed`). Arm only moved for the scan and the view over the target.
- Stages: scan OK (14/15 views, view 8 skipped by IK; 2,199,045 fused pts, 6,642 green pts, ICP median 5.2 mm / max 15.7 mm); coarse green OK (table tilt 4.5 deg, 107,621 inliers, 143 above-table clusters, green clusters [6322, 30, 24, 18, 15], target (257.0, -25.0, 4.0) mm; the 2 earlier scans of this head gave (256.8, -25.3) and the same x,y within 1 mm); view OK; close-view plan FAILED.
- `retry_roi_x4/` = a second plan from the same view with the ROI grown 4x (not run through the orchestrator, services called by hand).
- Evidence: `metrics.json`, `controller.log`, `grasp_node.log`, `scan/`, `plan/` (first plan), `retry_roi_x4/`, `plots/`, `screenshots/`, `videos/camera.mp4`, `trajectory.csv`, `failure_analysis.md`.
