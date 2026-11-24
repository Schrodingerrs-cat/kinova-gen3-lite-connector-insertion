# run_20261123_220417_orbit_aux_retry_1_aux
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points in every view | ok | plug points per view [373, 468, 410, 532, 748] (>= 100); board yaw spread 0.63 deg is reported only, the board orientation is not a condition |
| plug centre repeatable leaving a view out | ok | largest shift 2.1 mm (<= 3) |

Plug centre x 0.2575 y -0.0303 m; in the board frame -47.5 mm along the strip, -2.6 mm across it.

## Grasp
Board top -0.0124 m, plug top 0.0096 m (plug stands 22 mm). Grasp tcp z -0.0044 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6178123718261719} |
| 20 mm above the grasp point | {'tcp': [0.257, -0.0303, 0.0154]} |
| at grasp pose | {'tcp': [0.2573, -0.0303, -0.0045]} |
| closed | {'position': 0.8125538063049317, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0028} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.118, 'table_depth': 0.251, 'position': 0.8097165321350098} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
