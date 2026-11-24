# run_20261123_212231_orbit_aux_search
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [308, 602, 444, 404, 671] (>= 100), board outline [(151, 52), (151, 52), (150, 52)] mm (long and short side agree within 8 mm), board yaw spread 1.07 deg (<= 2) |
| plug centre repeatable leaving a view out | ok | largest shift 1.5 mm (<= 3) |

Plug centre x 0.2589 y 0.0133 m; in the board frame +0.7 mm along the strip, -1.6 mm across it.

## Grasp
Board top -0.0121 m, plug top 0.0104 m (plug stands 22 mm). Grasp tcp z -0.0041 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6177615119934082} |
| 20 mm above the grasp point | {'tcp': [0.2581, 0.0132, 0.0154]} |
| at grasp pose | {'tcp': [0.2584, 0.0132, -0.0044]} |
| closed | {'position': 0.8127590995788575, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0026} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.115, 'table_depth': 0.253, 'position': 0.8101656806945802} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
