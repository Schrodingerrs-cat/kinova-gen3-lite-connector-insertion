# run_20261119_212527_orbit_G1
Stage 1 (close scan) **PASS**. 5 views.
| gate | result | detail |
|---|---|---|
| plug points and board in every view | ok | plug points per view [488, 1123, 738, 855, 1225] (>= 200), board [(151, 51), (151, 51), (150, 51)] mm, board yaw spread 0.86 deg (<= 2) |
| wide-face yaw agrees left vs right and with the strip | ok | left face 90.6 deg (347 px), right face 92.6 deg (754 px), difference 2.0 (<= 5); mean 91.9 deg, -2.8 from the strip (<= 10) |
| plug centre repeatable leaving a view out | ok | largest shift 2.8 mm (<= 3) |

Plug centre x 0.2618 y -0.0238 m; in the board frame -47.9 mm along the strip, -0.9 mm across it.

## Grasp
Board top -0.0125 m, plug top 0.0335 m (plug stands 46 mm). Grasp tcp z -0.0005 m.

| stage | detail |
|---|---|
| at pre-grasp | {} |
| jaws open | {'position': 0.6174940498352052} |
| 20 mm above the grasp point | {'tcp': [0.2608, -0.0238, 0.0191]} |
| at grasp pose | {'tcp': [0.2611, -0.0238, -0.0008]} |
| closed | {'position': 0.8120274101257325, 'contact': False, 'gap_mm': 0.4, 'held': False} |
| not held: opened and lifted clear | {} |

The software result is the wrist-camera rule only; physical hold is judged by the operator.

## Operator observation
Operator saw the jaws close on the plug, then open slightly and lift with nothing held. Cause: the old script judged 'not held' from the finger reading (0.812, gap 0.4 mm), which cannot see thin parts, and opened the jaws before lifting. Fixed before G2 (camera decides after the lift). Not a missed grasp.

## Plots (plots/)
- `01_views_plug_overlay.png`
- `02_board_yaw_per_view.png`
- `03_plug_wide_face_yaw.png`
- `04_plug_points_top_view.png`
- `05_grasp_depth.png`
- `06_grasp_sequence.png`
