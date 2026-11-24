# run_20261123_220010_orbit_series2_1_usb
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points in every view | ok | plug points per view [345, 807, 598, 496, 756] (>= 200); board yaw spread 1.09 deg is reported only, the board orientation is not a condition |
| plug centre repeatable leaving a view out | ok | largest shift 2.9 mm (<= 3) |
| plug has a clearly wider side | ok | pixels facing the dominant direction 91 deg: 730, facing the perpendicular: 33, ratio 22.12 (>= 1.3, and >= 300 pixels) |
| wide-face yaw: both opposite faces seen and agreeing | ok | face on one side 87.9 deg (365 px), face on the other side 95.3 deg (405 px), difference 7.4 (<= 12); mean 91.8 deg |

Plug centre x 0.2546 y 0.0723 m; in the board frame +50.0 mm along the strip, -0.3 mm across it.

## Grasp
Board top -0.0108 m, plug top 0.0202 m (plug stands 31 mm). Grasp tcp z 0.0012 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6177916694641113} |
| 20 mm above the grasp point | {'tcp': [0.2539, 0.0721, 0.0209]} |
| at grasp pose | {'tcp': [0.2542, 0.0722, 0.001]} |
| closed | {'position': 0.8129003700256348, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0027} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.113, 'table_depth': 0.256, 'position': 0.810203377532959} |
| held for 5 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
