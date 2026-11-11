# run_20261110_134757_cloud_mode_gate40_autonomous  (real arm, cloud mode, SUCCESS n=1)

## A. Identification
- 2026-11-10, same hardware and head position as the head-mode run (`run_20261110_133718...`). Run by the assistant without intervention at the user's request: `run_pipeline.py RUN --grasp --mode cloud`, stack `./run_ros.sh dry_run:=false rviz:=false`.
- Objective: first execution of the cloud mode (marker region only, candidate search) on the arm.
- Change vs the two failed attempts the same day (`Failed_runs/run_20261110_134241_cloud_mode_close_view_58_marker_points`, `Failed_runs/run_20261110_134505_cloud_mode_close_view_63_marker_points`): `color_marker.min_marker_points` lowered from 80 to 40 at the user's request (the close-view sticker depth gave 58 and 63 points; only 64 here).

## B. Results (`controller.log`, `metrics.json`, 112.6 s total)
| stage | s | result |
|---|---|---|
| home | 4.4 | at home (43 deg away) |
| scan | 85.3 | 7 views (5 tilted), none skipped, 1,162,683 fused points, 3,760 marker points, ICP median 10.2 mm / max 26.6 mm |
| colour target | 3.6 | (263.0, -20.0, 3.0) mm |
| close view | 5.4 | camera 180 mm above |
| cloud grasp | 0.8 | 64 marker points with depth; sticker rectangle from the image 8.5 x 4.0 mm; search ON, faces `edge_prior`, extents from the sticker; 18 candidates; best: jaw along the long edge, yaw 0, offset 0, jaw (0.05, 1.00), pads on faces 4.0 mm wide (the sticker's), opening 32.5 mm, TCP (263.6, -21.9, -2.3) mm |
| execute | 11.6 | held (wrist camera: yes; finger check: no); depth between the tips 0.122 m, table 0.249 m |
Outcome: `GRASP_REPORTED_HELD`.

## C. Evidence
`videos/camera.mp4`, `videos/grasp_annotated.mp4`, `trajectory.csv`, `screenshots/` (`99_end.png` shows the connector between the fingertips and the empty slot on the panel), `perception/` (overlay with the two open finger positions), `plots/plan_01_marker_cloud_candidates_topview.png` and the scan plots, `point_clouds/*.ply`, `scan/`, `plan/`. mp4, ply, npz and npy are not in git.

## D. Limits
- n = 1. "Held" = wrist-camera check + the end image; not tugged, not carried to a second place. The finger check said "no" (known thin-object false negative).
- The sticker rectangle (8.5 x 4.0 mm) is smaller than the sticker seen on 2026-11-07 (12.5 x 9.5 mm): less of it is visible now. The opening (32.5 mm) comes from that rectangle plus 2 x 12 mm and is wider than the head (about 13 x 25 mm), so the jaw closes on the head, but the sizes the search used are the sticker's, not the head's.
- `jaw_edge: long` is a convention (the jaw closes along the sticker's long edge); which faces the pads touched is not checked here: the user has not reported it for this run.
- The gate was lowered after two failures; the close-view sticker depth fluctuates between 58 and 100 points per capture.
- Straight-down approach.
