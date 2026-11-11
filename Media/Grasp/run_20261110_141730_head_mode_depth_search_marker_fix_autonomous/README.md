# run_20261110_141730_head_mode_depth_search_marker_fix_autonomous  (real arm, head mode, SUCCESS n=1)

## A. Identification
- 2026-11-10, same hardware; the head was re-seated at a different angle. Run by the assistant without intervention at the user's request: `run_pipeline.py RUN --grasp --mode head`, stack `./run_ros.sh dry_run:=false rviz:=false`.
- First run on the arm with four changes made earlier the same day (all of them tested offline, 21 tests):
  1. **Depth dimension in the grid search** (`color_marker.extra_depth_m` 3..7 mm below the usual depth, `finger_clearance_m` 3 mm, `w_depth` 0.3): each geometry is tried at every depth; the finger boxes (reaching 3 mm below the tip) may not hit scene points.
  2. **Gripper order fix**: the jaw opening is set at the pre-grasp pose, just before the descent (before: at the start of execution, before the long move to the pre-grasp).
  3. **`min_head_points` 100** (was 200; one earlier run stopped at 136) and `min_marker_points` 40.
  4. **Marker selection**: a marker cluster whose xy extent is larger than 30 mm is not the sticker; in the close view only the blob nearest to the scan target is used. Reason: the previous attempt (`Failed_runs/run_20261110_140745_head_mode_no_sticker_on_head_teal_strip_on_panel`) matched a teal strip along the panel edge instead of the sticker (the strip is a 1568 px blob; the sticker 219 px; the user pointed out the head still had its green sticker, which is right: the sticker was detected, the strip was chosen as the target).

## B. Results (`controller.log`, `metrics.json`, 124.0 s total)
| stage | s | result |
|---|---|---|
| home | 5.0 | at home (47 deg away) |
| scan | 85.5 | 7 views, none skipped, 1,194,129 fused points, 3,726 marker points, ICP median 9.1 mm / max 31.1 mm |
| colour target | 3.0 | (256.0, -19.0, 2.0) mm; marker clusters [3478, 166, 20] |
| close view | 5.6 | camera 180 mm above |
| head grasp | 9.0 | head cloud 930 points, 102 marker points with depth; grid 229 x yaw 36 x depth 5 = 41,220 candidates, 9,159 valid (31,750 jaw centre off the gap middle, 20 too few points, 291 finger-collision); best yaw 40 deg, 2.1 mm from the sticker, gap 11.8 mm, opening 35.8 mm, depth +7 mm, TCP z -2.7 mm (sticker median height 12.3 mm in this frame), antipodal 0.79, contact 46 points |
| execute | 14.4 | held (wrist camera: yes; finger check: no); depth between the tips 0.117 m, table 0.253 m |
Outcome: `GRASP_REPORTED_HELD`.

## C. Checks done on this run
- **Gripper order (from `trajectory.csv`)**: gripper fully open until t = 114.6 s, i.e. after the arm reached the pre-grasp pose (execution started at 109.5 s); it then narrowed to the opening (0.41 -> 0.60 rad), the tool descended (z 92 -> 32 mm), the gripper closed to 0.81 rad at the bottom, then the lift. In the head-mode run before the fix (`run_20261110_135341...`) it left fully open at t = 106.0 s, right at the start of execution.
- **Candidate 0 had no usable IK** ("pre-grasp: no near-current IK"): measured afterwards with the same pose and seed: its IK solution is a wrist-flipped configuration (joint 4 +177 deg, joint 5 +86 deg, over the 120 deg limit); the jaw-flipped candidate 1 needs 47 deg on joint 6 at most and was executed. Same two walls (parallel gripper).
- `plots/plan_07_depth_levels_side_view.png`: fingertips at every depth tried against the head and the panel (the panel is 10 mm or more below the tips here, so the deepest level, +7 mm, was allowed and preferred).

## D. Evidence
`videos/camera.mp4`, `videos/grasp_annotated.mp4`, `trajectory.csv`, `screenshots/`, `perception/`, `plots/plan_01..07` and scan plots, `point_clouds/*.ply`, `scan/`, `plan/`. mp4, ply, npz and npy are not in git.

## E. Limits
n = 1; held = wrist camera + the end image, not tugged; the finger check said "no" (known false negative); only 102 marker points had depth in the close view; the 3D sticker height in the close frame (12.3 mm) is 10 mm above the scan's marker height (2.0 mm): the known sticker depth ramp; the jaw direction (yaw 40 deg) differs from earlier runs because the head was re-seated at another angle. The user has not reported which faces the pads touched in this run. Straight-down approach.
