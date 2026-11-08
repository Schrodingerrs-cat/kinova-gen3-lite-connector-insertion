# How the connector was perceived and grasped (run_20261107_142037_green_sticker_grasp)

All numbers are from this run's data (`plan/*/plan.json`, `metrics.json`, `controller.log`). Images in `perception/`, clouds in `point_clouds/`.

## 0. The scene
Black connector head (USB-A, housing about 25 x 13 mm seen from above) lying in a slot of a white panel, one teal-green sticker on its top face, cable leaving towards the robot base. Wood table about 30 mm below the panel top.

## 1. Scan (reused: `scan/20261107_140836`)
14 camera poses over the workspace (7 straight down at different positions/yaws, 7 tilted 25-45 deg from the sides), each saved with depth + colour (`scan/*/view_XX.npz`). One of the 15 planned views (view 8) had no IK solution this time. `perception/02_green_mask_in_scan_views.png`: the green mask (HSV H 80-105, S>=140, V>=60) on six of the views; the sticker is found in all of them.

## 2. Merge, ICP, voxel, table removal, clustering (`scan/*/plots/`, `point_clouds/1_*.ply`, `2_*.ply`)
- Per-view cloud, edge filter (3 mm depth jumps), voxel 0.8 mm (green 0.5 mm), outlier removal; trimmed ICP between views: corrections median 5.2 mm, max 15.7 mm; 2,199,045 fused points, 6,642 green points (green gets the same ICP corrections).
- RANSAC table plane: 107,621 inliers, tilt 4.5 deg (hand-eye error; the table is flat).
- Above-table points -> DBSCAN: 143 clusters; green points above the table -> DBSCAN: 5 clusters (6,322 / 30 / 24 / 18 / 15 points); the big one is the head.
- Result: coarse target = green centroid (257.0, -25.0, 4.0) mm. The merged cloud smears the head (views disagree by 5-15 mm) so ONLY this centroid is used (`scan/*/plots/03_green_patch_frame.png` shows the nonsense orientation of the merged patch).

## 3. Close view (single frame, no registration error) (`perception/01_*`, `03_*`, `point_clouds/3_*.ply`, `4_*.ply`)
Camera moved to 180 mm over the target. In that one frame:
- green pixels -> 458 sticker points, median (256.6, -28.9, 7.6) mm (median, because the glossy head gives depth streaks that would pull a mean);
- housing footprint = dark + green pixels connected to the sticker within 22 mm, back-projected at the sticker depth: 24.7 x 13.4 mm; long axis along the cable, jaw axis across the body (-0.21, 0.98);
- jaw opening = 13.4 + 2 x 12 mm margin = 37.4 mm; TCP = sticker median z - 8 mm (chosen constant) = (256.6, -28.9, -0.4) mm; approach straight down.
`01_close_view_overlay.png`: green contour, orange footprint, red dots = open fingers, blue star = TCP. `03_close_frame_point_cloud_3_views.png`: the single-frame cloud; note the layers (table, panel) and the depth streak above the head, which is why the housing is NOT taken from depth.

## 4. Execution (`videos/camera.mp4`, `videos/grasp_annotated.mp4`, `trajectory.csv`)
Candidate flip +1 had no near IK, flip -1 was used. Open fingers, pre-grasp 60 mm above, straight descent, close, straight lift 80 mm. Recorded tool point lowest at (256.5, -28.8, 34.2) mm (TCP about -0.8 mm). Gripper closed to 0.812 rad.

## 5. Verdict
Node (finger reading): "closed on nothing" - a known false negative for thin objects (HW7 docs 3.7). Video/end image: connector between the fingertips, lifted out of the panel. User confirmed it worked. The node now also runs a wrist-camera hold check after the lift (`/grasp/hold_check`); it was NOT yet tested live because the camera dropped off USB right after this run.
