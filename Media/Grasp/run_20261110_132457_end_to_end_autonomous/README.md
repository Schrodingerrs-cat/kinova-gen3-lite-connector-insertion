# run_20261110_132457_end_to_end_autonomous  (real arm, sticker mode, SUCCESS n=1)

## A. Identification
- 2026-11-10, same hardware and head as `run_20261110_133718...`. Run by the assistant without intervention: `run_pipeline.py RUN --grasp` (default sticker mode), 15-view scan, `dry_run:=false rviz:=false`. First full scan after the scan-save fix (`grasp_node.py` line 267).

## B. Results (`metrics.json`, 171.4 s total)
home 4.4 s; scan 143.8 s (15 views, 2,290,259 fused points, 9,149 marker points, ICP median 4.5 mm / max 16.7 mm); colour target (259.0, -22.0, 4.0) mm; close view; sticker grasp: sticker median (259.3, -24.2, 8.3) mm from 682 points, footprint 25.2 x 12.9 mm, jaw axis (-0.30, 0.95), opening 36.9 mm, TCP z 0.3 mm; execute 11.6 s: held (wrist camera: yes, depth between the tips 0.120 m vs table 0.261 m; finger check: no).
Outcome: `GRASP_REPORTED_HELD`. The end image shows the connector between the fingertips.

## C. Evidence
As in the head-mode run: video, annotated video, trajectory, screenshots, perception overlays, plots, PLY clouds, scan, plan.

## D. Limits
n = 1 for this head position, held = wrist camera + one image, not tugged; sticker mode has no candidate search; straight-down approach (thin walls). An earlier perception-only run the same day with the head placed differently produced a bad plan (sticker depth 52 mm off, TCP below the table) and was stopped before any grasp: `Failed_runs/run_20261110_131913_perception_only_bad_sticker_depth`.

## E. User observation (2026-11-10, logged as told; not measured or checked by the assistant)
- The user saw that in this sticker-mode run the grasp was from the **narrower faces**. In the head-mode run the same day (`run_20261110_133718_head_mode_grid_search_autonomous_rerun`) the user saw the **wider faces**. These are a person's observations of the real grasps, not results of the videos or measurements.
- For comparison only: this run's plan closed across 12.9 mm with jaw (-0.30, 0.95) (footprint 25.2 x 12.9 mm from the image outline), and the head-mode run across 13.0 mm with jaw (-0.91, 0.42). Both gaps are the short direction, so the plan numbers do not explain a difference in faces; the jaw yaw differs by 52 deg and the head position by about 4 mm, so the head was probably placed differently. Not resolved. To check: `videos/grasp_annotated.mp4` and `screenshots/05_after_grasp.png` in both runs.
