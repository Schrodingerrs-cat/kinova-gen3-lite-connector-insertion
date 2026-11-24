# run_20261123_215232_orbit_series1_2_aux
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [388, 525, 455, 368, 512] (>= 100), board outline [(150, 52), (152, 51), (151, 51)] mm (long and short side agree within 8 mm), board yaw spread 0.98 deg (<= 2) |
| plug centre repeatable leaving a view out | ok | largest shift 2.4 mm (<= 3) |

Plug centre x 0.2567 y -0.0320 m; in the board frame -49.3 mm along the strip, -3.6 mm across it.

## Grasp
Board top -0.0122 m, plug top 0.0111 m (plug stands 23 mm). Grasp tcp z -0.0042 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6177831413269044} |
| 20 mm above the grasp point | {'tcp': [0.2562, -0.0319, 0.0156]} |
| at grasp pose | {'tcp': [0.2563, -0.0319, -0.0045]} |
| closed | {'position': 0.8125368736267091, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0027} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.111, 'table_depth': 0.253, 'position': 0.8098163360595704} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
