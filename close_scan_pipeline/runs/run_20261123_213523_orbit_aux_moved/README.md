# run_20261123_213523_orbit_aux_moved
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [404, 506, 507, 324, 425] (>= 100), board outline [(150, 52), (150, 52), (150, 51)] mm (long and short side agree within 8 mm), board yaw spread 0.76 deg (<= 2) |
| plug centre repeatable leaving a view out | ok | largest shift 1.9 mm (<= 3) |

Plug centre x 0.2569 y 0.0656 m; in the board frame +47.8 mm along the strip, -3.0 mm across it.

## Grasp
Board top -0.0121 m, plug top 0.0104 m (plug stands 23 mm). Grasp tcp z -0.0041 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6177878997802735} |
| 20 mm above the grasp point | {'tcp': [0.2562, 0.0654, 0.0155]} |
| at grasp pose | {'tcp': [0.2562, 0.0654, -0.0045]} |
| closed | {'position': 0.8127506332397462, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0026} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.117, 'table_depth': 0.253, 'position': 0.8101392929077149} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
