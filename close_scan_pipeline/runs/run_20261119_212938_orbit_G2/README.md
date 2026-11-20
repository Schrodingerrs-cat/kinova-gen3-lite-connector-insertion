# run_20261119_212938_orbit_G2
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [541, 1192, 807, 906, 1260] (>= 200), board [(151, 52), (152, 51), (151, 51)] mm, board yaw spread 0.53 deg (<= 2) |
| wide-face yaw agrees left vs right and with the strip | ok | left face 91.1 deg (382 px), right face 93.8 deg (769 px), difference 2.7 (<= 5); mean 92.9 deg, -1.8 from the strip (<= 10) |
| plug centre repeatable leaving a view out | ok | largest shift 2.6 mm (<= 3) |

Plug centre x 0.2619 y -0.0241 m; in the board frame -48.2 mm along the strip, -1.0 mm across it.

## Grasp
Board top -0.0137 m, plug top 0.0329 m (plug stands 47 mm). Grasp tcp z -0.0017 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6174931228637696} |
| 20 mm above the grasp point | {'tcp': [0.2609, -0.024, 0.0178]} |
| at grasp pose | {'tcp': [0.2609, -0.0241, -0.0023]} |
| closed | {'position': 0.8120556518554689, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0021} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.109, 'table_depth': 0.256, 'position': 0.8099773818969727} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Operator observation
Operator confirmed by eye that the plug was picked up out of the board. Camera rule said HELD.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
