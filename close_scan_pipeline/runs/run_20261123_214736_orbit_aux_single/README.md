# run_20261123_214736_orbit_aux_single
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points in every view | ok | plug points per view [362, 466, 343, 573, 791] (>= 100); board yaw spread 0.89 deg is reported only, the board orientation is not a condition |
| plug centre repeatable leaving a view out | ok | largest shift 2.5 mm (<= 3) |

Plug centre x 0.2571 y -0.0322 m; in the board frame -49.2 mm along the strip, -3.4 mm across it.

## Grasp
Board top -0.0127 m, plug top 0.0100 m (plug stands 23 mm). Grasp tcp z -0.0047 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6176890228271484} |
| 20 mm above the grasp point | {'tcp': [0.2558, -0.0323, 0.0149]} |
| at grasp pose | {'tcp': [0.2564, -0.0323, -0.005]} |
| closed | {'position': 0.81261782913208, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0025} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.11, 'table_depth': 0.23, 'position': 0.8100818824768067} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
