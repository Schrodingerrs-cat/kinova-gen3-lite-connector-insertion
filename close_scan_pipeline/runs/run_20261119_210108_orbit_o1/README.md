# run_20261119_210108_orbit_o1
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [363, 1178, 775, 547, 1146] (>= 200), board [(150, 52), (151, 51), (151, 51)] mm, board yaw spread 1.15 deg (<= 2) |
| wide-face yaw agrees left vs right and with the strip | ok | left face 93.3 deg (396 px), right face 94.1 deg (700 px), difference 0.8 (<= 5); mean 93.8 deg, -1.0 from the strip (<= 10) |
| plug centre repeatable leaving a view out | ok | largest shift 2.3 mm (<= 3) |

Plug centre x 0.2624 y -0.0262 m; in the board frame -49.0 mm along the strip, -0.8 mm across it.

## Operator observation
First close scan 0.228 m above the board, scan and stage 1 check only, no grasp.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
