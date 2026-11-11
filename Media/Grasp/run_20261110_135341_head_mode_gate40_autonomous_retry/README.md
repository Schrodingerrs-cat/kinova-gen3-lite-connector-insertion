# run_20261110_135341_head_mode_gate40_autonomous_retry  (real arm, head mode, SUCCESS n=1)

## A. Identification
- 2026-11-10, same hardware and head position as the other runs of the day. Run by the assistant without intervention at the user's request: `run_pipeline.py RUN --grasp --mode head`, stack `./run_ros.sh dry_run:=false rviz:=false`, `color_marker.min_marker_points` 40.
- This is the second head-mode execution on the arm (first: `run_20261110_133718...`). It followed a failed attempt with the same settings (`Failed_runs/run_20261110_135145_head_mode_head_cloud_136_points`: head cloud 136 points, minimum 200; the same pipeline gave 847 and 803 points in the two runs that passed).

## B. Results (`controller.log`, `metrics.json`, 125.5 s total)
| stage | s | result |
|---|---|---|
| home | 4.4 | at home (43 deg away) |
| scan | 85.1 | 7 views, none skipped, 1,158,917 fused points, 3,733 marker points, ICP median 10.0 mm / max 25.6 mm |
| colour target | 3.2 | (263.0, -20.0, 3.0) mm |
| close view | 5.4 | camera 180 mm above |
| head grasp | 6.4 | head cloud 803 points, 61 marker points with depth; grid 187 x yaw 36 = 6732 candidates, 1707 valid (5010 jaw centre off the gap middle, 12 too few points, 3 no contact, 0 finger collision); best yaw 170 deg, 4.5 mm from the sticker, gap 12.1 mm, opening 36.1 mm, TCP z -2.0 mm, antipodal 0.90, contact 50 points |
| execute | 19.4 | held (wrist camera: yes; finger check: no); depth between the tips 0.144 m, table 0.252 m |
Outcome: `GRASP_REPORTED_HELD`.

## C. Evidence
`videos/camera.mp4`, `videos/grasp_annotated.mp4`, `trajectory.csv`, `screenshots/` (`99_end.png`: the connector between the fingertips, the slot on the panel empty), `perception/`, `plots/plan_01..06` (grid-search figures) and scan plots, `point_clouds/*.ply`, `scan/`, `plan/`. mp4, ply, npz and npy are not in git.

## D. Limits
n = 1; held = wrist camera + one image, not tugged; the finger check said "no" (known false negative); only 61 marker points had depth in the close view (gate 40, lowered by the user); the head cloud size varies a lot between captures (136 to 847 points); straight-down approach. The user has not reported which faces the pads touched in this run.
