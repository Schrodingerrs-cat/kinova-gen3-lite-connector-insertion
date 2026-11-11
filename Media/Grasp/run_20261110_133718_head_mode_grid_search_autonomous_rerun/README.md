# run_20261110_133718_head_mode_grid_search_autonomous_rerun  (real arm, SUCCESS n=1)

## A. Identification
- 2026-11-10, Kinova Gen3 Lite + wrist D405, USB-A cable head (black housing, teal sticker on the top face) on the white panel. Run by the assistant without intervention at the user's request: `run_pipeline.py RUN --grasp --mode head` from home to grasp. Stack: `./run_ros.sh dry_run:=false rviz:=false`.
- Objective: first execution of the image mode (`--mode head`, grid search, `Code/Grasp/PIPELINE.md` 5.2) on the arm.
- Changes vs the earlier head-mode attempt the same day (`Failed_runs/run_20261110_133346_head_mode_close_view_99_marker_points`): close-view depth = median of 5 frames (`scan.close_frames`), `color_marker.min_marker_points` 80 (was a fixed 100; the failed attempt had 99). Scan: 7 views, 5 tilted.

## B. Results (`controller.log`, `metrics.json`, 126.6 s total)
| stage | s | result |
|---|---|---|
| home | 4.4 | at home (43 deg away) |
| scan | 85.1 | 7 views, none skipped, 1,156,032 fused points, 3,728 marker points, ICP corrections median 10.8 mm / max 30.6 mm |
| colour target | 3.2 | (263.0, -19.0, 4.0) mm, one marker cluster of 3,643 points |
| close view | 5.4 | camera 180 mm above the target |
| head grasp | 6.2 | head cloud 847 points, marker points with depth 100; grid 200 x yaw 36 = 7200 candidates, 1512 valid (5445 jaw centre off the gap middle, 227 finger-collision, 16 too few points); best yaw 155 deg, 3.6 mm from the sticker, gap 13.0 mm, opening 37.0 mm, TCP z -0.3 mm, antipodal 0.83, contact 47 points |
| execute | 20.8 | held (wrist camera: yes; finger check: no); depth between the tips 0.145 m, table 0.255 m |
Outcome: `GRASP_REPORTED_HELD`.

## C. Evidence
`videos/camera.mp4`, `videos/grasp_annotated.mp4`, `trajectory.csv`, `screenshots/`, `perception/` (close-view overlay with the head cloud and open finger positions), `plots/` (`plan_01..06` grid-search figures, scan plots), `point_clouds/*.ply`, `scan/` (7 views + fused clouds), `plan/` (close frame, head cloud, plan.json). mp4, ply, npz and npy are not in git (see `../README.md`).

## D. Limits (do not over-read this run)
- n = 1. "Held" is the wrist-camera check plus one end image; the connector was not tugged or carried to a second place. The finger-position check said "no", the known thin-object false negative.
- Only 100 marker points had depth in the close view (237 sticker pixels): sparse depth on the glossy sticker; the threshold 80 was lowered by the user's decision after the 99-point failure.
- Straight-down approach: the pads touch the thin side walls (jaw across the 13 mm direction).
- Finger-collision test is on in this mode (227 candidates removed); optimizer limits and the sticker-mode checks are as in `config_grasp.yaml`.
- Sticker mode and head mode were not compared on the same head position.

## E. User observation (2026-11-10, logged as told; not measured or checked by the assistant)
- The user saw that in this head-mode run the gripper grasped the head from the **wider faces**, and called it good. This is a person's observation of the real grasp, not a result of the video, the plan numbers or a measurement.
- What the numbers say, for comparison only: the best candidate closes across a 13.0 mm gap (opening 37.0 mm), i.e. across the head's short direction, so the pads sit on the two longer walls; the jaw is yaw 155 deg (-0.91, 0.42). The sticker-mode run the same day (`run_20261110_132457_end_to_end_autonomous`) closed across 12.9 mm with jaw (-0.30, 0.95), 52 deg different, and its target also differed by 4 mm in x, 3 mm in y, so the head was probably not in the same place for the two runs.
- "Wider" here is the user's word. The earlier write-up used "wide faces" for the sticker (top) and bottom faces, which a straight-down approach cannot touch; the user means the larger side walls, as far as the assistant can tell. To be confirmed by the user, or from `videos/grasp_annotated.mp4` and `screenshots/05_after_grasp.png`.
