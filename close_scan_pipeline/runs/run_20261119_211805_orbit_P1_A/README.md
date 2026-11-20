# run_20261119_211805_orbit_P1_A
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [504, 1142, 719, 861, 1214] (>= 200), board [(151, 51), (151, 51), (151, 51)] mm, board yaw spread 0.47 deg (<= 2) |
| wide-face yaw agrees left vs right and with the strip | ok | left face 92.3 deg (341 px), right face 92.6 deg (731 px), difference 0.3 (<= 5); mean 92.5 deg, -2.1 from the strip (<= 10) |
| plug centre repeatable leaving a view out | ok | largest shift 2.8 mm (<= 3) |

Plug centre x 0.2618 y -0.0240 m; in the board frame -48.2 mm along the strip, -1.1 mm across it.

## Operator observation
Scan and stage 1 check only, no grasp.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
