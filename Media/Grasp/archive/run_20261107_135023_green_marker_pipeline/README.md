# run_20261107_135023_green_marker_pipeline (scan + offline analysis only)

## A. Identification
- Run ID: run_20261107_135023_green_marker_pipeline, 2026-11-07 13:50-14:02. Real Kinova Gen3 Lite + wrist D405 (USB 2.1).
- Objective: first scan WITH colour (15 views), merged + ICP, green marker cloud, table RANSAC, DBSCAN of the green points.
- Hypothesis: the merged multi-view cloud is accurate enough to find the green marker AND to compute its plane normal / axes for a grasp.
- Status: **PARTIAL_SUCCESS** (scan, merge, colour segmentation worked; hypothesis about orientation REJECTED). No grasp attempted.
- Code version: not under git (UNKNOWN).

## B. Implementation
- 15 views from `config_grasp.yaml` (`scan.views`, 7 straight down at 3 yaws/positions, 8 tilted 25-45 deg), tool orientation of the home pose, cloud voxel 0.8 mm (green: 0.5 mm), trimmed ICP between views, green = HSV H 80-105, S>=140, V>=60 (measured on the head: H 93, S 219, V 138).
- Table: RANSAC (3 mm) on points within 30 mm of `scan.table_z_m`; object = 3-80 mm above that plane; DBSCAN eps 6 mm (object), 3 mm (green).
- Hand-eye = HW7's (`config_grasp.yaml: camera`). Scene: black connector head with ONE teal sticker on its top face, lying on a white board; the handwritten label strip (also teal ink) was still on the table.

## C. Results (`scan/`, `plots/`)
- 15/15 views, 2,265,314 fused points, 8,167 green points, ICP corrections median 5.1 mm, max 15.0 mm.
- RANSAC table plane: 106,935 inliers, **tilt 4.34 deg** on a flat table (leftover hand-eye error, same as HW7 docs).
- Green clusters above the table: one big (6,411 pts at base (257, -25) mm = the head) and four tiny ones (15-46 pts, 3-6 mm high, x~120 mm = label ink).
- Merged green cluster size 38 x 23 x 17 mm, long axis (0.26, 0.04, -0.96) = nearly VERTICAL, patch normal nearly horizontal. The sticker is flat on a top face, so this geometry is wrong.

## D. Evidence
- `plots/01_table_removal.png`, `plots/02_clusters_topview.png`, `plots/03_green_patch_frame.png`, `scan/` (view_XX.npz with depth + colour, fused_cloud.npy, green_cloud.npy, fusion.json, green_analysis not written in this run), `videos/camera.mp4` (full recording, 10 fps), `trajectory.csv` (joints, gripper, TCP, 10 Hz), `controller.log` n/a (manual run), `screenshots/`.

## E. Observations / conclusions
- Verified: colour segmentation finds the sticker in every view; table RANSAC + DBSCAN isolate the head from the label ink.
- Verified: the stitched cloud has stacked "table" layers (side views z ~ -50, -15 mm) and a 41 mm tall slanted head: the views disagree by more than the head height. Cause (suspected, not isolated): hand-eye rotation error amplified by distance (HW7 measured 0.9-6.6 deg per-view table tilt).
- Consequence: only the green CENTROID (x, y) of the merged scan is used as a coarse target; normals/axes/grasp come from a single close view.
- Not verified: how large the merged-scan centroid error really is (the earlier 'find' target missed the plug by ~37 mm from the far view).
- Next: close-view green ROI + housing plan (`run_green_pipeline.py`).
