# run_20261119_220300_orbit_POW1
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [699, 995, 726, 889, 1210] (>= 200), board [(152, 52), (152, 53), (152, 52)] mm, board yaw spread 0.62 deg (<= 2) |
| plug centre repeatable leaving a view out | ok | largest shift 4.5 mm (<= 6) |

Plug centre x 0.2563 y 0.0782 m; in the board frame +51.4 mm along the strip, +0.2 mm across it.

## Grasp
Board top -0.0143 m, plug top 0.0153 m (plug stands 30 mm). Grasp tcp z -0.0043 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.510194696044922} |
| 20 mm above the grasp point | {'tcp': [0.256, 0.0781, 0.0154]} |
| at grasp pose | {'tcp': [0.2555, 0.0778, -0.0046]} |
| closed | {'position': 0.8132261695861817, 'contact': False, 'gap_mm': 0.4, 'note': 'finger reading does not decide'} |
| 5 mm lift | {'finger_change': 0.0033} |
| lifted 80 mm, camera check | {'held': True, 'depth_between_tips': 0.111, 'table_depth': 0.254, 'position': 0.8099434547424317} |
| held for 8 s, then jaws opened where the arm is (no put-back) | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Operator observation
Power plug (thick black cable, teal body). CAD size did not match the real part (CAD 26.5 mm square, real teal body ~15 x 21 mm, top 30 mm above the board), so the body was found from bright teal pixels only. Camera rule HELD (0.111 m between the tips, table 0.254 m). Held 8 s, then the jaws were opened where the arm was (no put-back, as the operator asked). Images show the pin slots empty where the plug sat. Physical hold: operator to confirm.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
- `07_hold_check.png`
