# Testing every mode on the robot, terminal by terminal

Status of each mode before you start: `sticker` ran on the arm once (n = 1). `head` (grid search) and `cloud` are offline only, never executed on the arm.
The e-stop stays in reach for every real-motion step. Settings in `config/grasp.yaml` are read when the node starts, so after editing one, stop Terminal 1 (Ctrl-C) and start it again.

## Every terminal starts with
```bash
cd ~/Documents/GitHub/VBRM_insertion/insertion_task/Code/Grasp
source ros_env.sh
```

## Terminal 1: the stack (leave it running)
```bash
./run_ros.sh                    # dry run: every IK / line is only checked, nothing moves
./run_ros.sh dry_run:=false     # real motion
```
Brings up the arm link, driver, D405, MoveIt, the grasp node and the two RViz windows (`grasp.rviz`, `grasp_clouds.rviz`). Add `rviz:=false` for none. Logs: `logs/*.log`.

## Terminal 2: live camera
```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```
## Terminal 3 (only if you closed RViz)
```bash
QT_QPA_PLATFORM=xcb rviz2 -d ros2_iface/grasp.rviz
QT_QPA_PLATFORM=xcb rviz2 -d ros2_iface/grasp_clouds.rviz
```

## Terminal 4: run a test
First, always: `python3 ros2_iface/smoke_test.py` (joints, TF, action servers, camera, IK; never moves).

`RUN` below is a run folder. Use a new one per run:
```bash
RUN=$PWD/../../Media/Grasp/run_$(date +%Y%m%d_%H%M%S)_mytest
```
Add `--skip-scan` to reuse the last scan (saves about 2.5 minutes) once one good scan exists. A run without `--grasp` does everything except move to the grasp.

### Step 0: scan and rough target only (no grasp)
```bash
python3 ros2_iface/run_pipeline.py $RUN
```
Check in RViz window 2 that the fused scan, the marker cloud and the cleaned scene look right, and that the camera ends over the sticker.

### Step 1: compute the grasp in each mode, without moving to it
Needs Terminal 1 up and the camera over the sticker (Step 0 leaves it there).
```bash
python3 ros2_iface/run_pipeline.py $RUN --skip-scan --mode sticker
python3 ros2_iface/run_pipeline.py $RUN --skip-scan --mode head
python3 ros2_iface/run_pipeline.py $RUN --skip-scan --mode cloud
```
Each writes plots into `$RUN/plots/`. The head mode writes the six grid-search figures (normals, candidates, grasp points, counts, cost per yaw). Compare the three tool points and jaw axes in `$RUN/controller.log` before you let any of them move the arm.

### Step 2: real grasp per mode (Terminal 1 with `dry_run:=false`)
```bash
python3 ros2_iface/run_pipeline.py $RUN --skip-scan --mode sticker --grasp
python3 ros2_iface/run_pipeline.py $RUN --skip-scan --mode head --grasp
python3 ros2_iface/run_pipeline.py $RUN --skip-scan --mode cloud --grasp
```
After each real run: `python3 ros2_iface/export_evidence.py $RUN` (point clouds, overlays, annotated video). Record the outcome in `Media/Grasp/README.md`, and keep failures in `Media/Grasp/Failed_runs/`.

## Combinations to test
Settings are in the `color_marker:` block of `config/grasp.yaml`. Change one, restart Terminal 1, then rerun Step 1 (and Step 2 if the plan looks right).

| Mode | Setting | Values to try | What it changes |
|---|---|---|---|
| sticker | none | | one grasp plus the jaw flip, no search |
| head | none | | grid search, always on, no switch |
| cloud | `candidate_search` | `true`, `false` | search of about 30 candidates, or one closed-form grasp |
| cloud | `jaw_edge` | `long`, `short` | jaw along the sticker's long or short edge (90 degrees apart) |
| cloud | `contact_faces` | `edge_prior`, `larger`, `smaller` | rule for which wall pair the pads touch |
| cloud | `extent_source` | `sticker`, `footprint` | face sizes from the sticker rectangle or the head outline |

Useful cloud combinations: `candidate_search` true and false with the defaults; `contact_faces: larger` with `extent_source: sticker` and with `footprint`; `jaw_edge: short`. `candidate_search` can also be given at launch for the cloud mode: `./run_ros.sh dry_run:=false candidate_search:=off`.

## Other things to test
- **Different marker colour:** change `h`, `s_min`, `v_min` in `color_marker:` (a hue range that wraps, such as red, is written `h: [170, 10]`), restart, then Step 0.
- **One-call pick:** `./run_ros.sh dry_run:=false grasp_mode:=head`, then `ros2 service call /grasp/pick std_srvs/srv/Trigger`. Not run yet.
- **Hold check after a lift:** `ros2 service call /grasp/hold_check std_srvs/srv/Trigger`. Not run yet on a held object.
- **Individual services:** `ros2 service call /grasp/<name> std_srvs/srv/Trigger` with `home, scan, color_target, view, sticker_grasp, head_grasp, cloud_grasp, allow_grasp, execute, hold_check`. `execute` only checks the motion until `allow_grasp` has been called and the node runs with `dry_run:=false`.
- **Gripper and home:** `python3 ros2_iface/go_home.py`, `python3 ros2_iface/open_gripper.py`.
- **Keyboard jog:** `Code/Teleop/run.sh` (never while the pipeline is moving the arm).

## Known limits while testing
- The approach is straight down in every mode, so the pads can only touch the thin side walls.
- The grasp is computed from one top-down view; the walls are assumed vertical.
- Finger collision and optimizer limits are off by decision. Nothing stops a grasp next to the panel or the cable.
- If the D405 drops off USB the driver crashes: replug it and restart Terminal 1.
