# run_20261119_214552_orbit_AUX1
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [311, 432, 274, 220, 427] (>= 100), board [(151, 52), (152, 51), (151, 52)] mm, board yaw spread 0.80 deg (<= 2) |
| plug centre repeatable leaving a view out | ok | largest shift 2.2 mm (<= 3) |

Plug centre x 0.2566 y 0.0245 m; in the board frame +1.4 mm along the strip, -2.2 mm across it.

## Grasp
Board top -0.0125 m, plug top 0.0112 m (plug stands 24 mm). Grasp tcp z -0.0065 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6174931228637696} |
| 20 mm above the grasp point | {'tcp': [0.2557, 0.0243, 0.013]} |
| at grasp pose | {'tcp': [0.256, 0.0244, -0.0069]} |
| closed | {'position': 0.8119125274658204, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0019} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.114, 'table_depth': 0.251, 'position': 0.8100536407470703} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
