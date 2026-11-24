# run_20261123_220221_orbit_series2_3_power
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points in every view | ok | plug points per view [692, 962, 804, 752, 995] (>= 200); board yaw spread 0.91 deg is reported only, the board orientation is not a condition |
| plug centre repeatable leaving a view out | ok | largest shift 3.9 mm (<= 6) |

Plug centre x 0.2592 y 0.0208 m; in the board frame +1.2 mm along the strip, -0.4 mm across it.

## Grasp
Board top -0.0096 m, plug top 0.0173 m (plug stands 27 mm). Grasp tcp z 0.0004 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.5102163562774659} |
| 20 mm above the grasp point | {'tcp': [0.2586, 0.0208, 0.0201]} |
| at grasp pose | {'tcp': [0.2587, 0.0208, 0.0002]} |
| closed | {'position': 0.8131263656616211, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.003} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.116, 'table_depth': 0.254, 'position': 0.8101167366027834} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
