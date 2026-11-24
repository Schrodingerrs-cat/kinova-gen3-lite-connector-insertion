# run_20261123_211640_orbit_aux_vision
Stage 1 (close scan) **FAIL**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | FAIL | plug points per view [167, 60, 102, 224, 350] (>= 100), board outline [(151, 51), (150, 52), (151, 52)] mm (long and short side agree within 8 mm), board yaw spread 0.33 deg (<= 2) |
| plug centre repeatable leaving a view out | ok | largest shift 2.1 mm (<= 3) |

Plug centre x 0.2607 y 0.0112 m; in the board frame -1.4 mm along the strip, -0.2 mm across it.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `04_plug_points_top_view.png`
