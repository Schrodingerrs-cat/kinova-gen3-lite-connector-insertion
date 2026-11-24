# run_20261123_215151_orbit_series1_1_usb
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points in every view | ok | plug points per view [317, 828, 606, 473, 750] (>= 200); board yaw spread 1.24 deg is reported only, the board orientation is not a condition |
| plug centre repeatable leaving a view out | ok | largest shift 2.9 mm (<= 3) |
| plug has a clearly wider side | ok | pixels facing the dominant direction 91 deg: 771, facing the perpendicular: 25, ratio 30.84 (>= 1.3, and >= 300 pixels) |
| wide-face yaw: both opposite faces seen and agreeing | ok | face on one side 87.9 deg (388 px), face on the other side 95.3 deg (426 px), difference 7.4 (<= 12); mean 91.7 deg |

Plug centre x 0.2545 y 0.0723 m; in the board frame +49.9 mm along the strip, -0.2 mm across it.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
