# run_20261107_140509_green_marker_pipeline_grasp  (FAILED, software bug, no motion beyond the scan)

- Date: 2026-11-07 14:05. Objective: end-to-end green-marker pipeline incl. grasp (`ros2_iface/run_green_pipeline.py --grasp`). Status: **FAILED** at stage 2 (`/grasp/green`).
- Done before the failure: home check, 15-view colour scan (145 s), merge + ICP: 2,274,440 fused points, 7,122 green points, ICP corrections median 6.0 mm, max 15.6 mm (`scan/`, `metrics.json`).
- Not reached: coarse green stage, view over the target, close-view plan, gates, grasp. The arm never left its scan poses; it returned to the start pose after the scan.
- Evidence: `metrics.json`, `controller.log`, `grasp_node.log` (contains the traceback), `videos/camera.mp4`, `trajectory.csv`, `scan/`, `screenshots/`, `failure_analysis.md`.
