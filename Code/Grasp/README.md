# Grasp pipeline (Task A): colour marker on the cable head -> grasp

Every stage, switch, parameter and limit is described in [`PIPELINE.md`](PIPELINE.md). Terminal-by-terminal commands to test every mode and combination: [`TESTING.md`](TESTING.md).

A teal-marker sticker on the TOP face of the connector head marks the grasp. No connector dimensions are configured: the housing width is
measured from the colour image around the sticker. The earlier depth-based housing planner and optimizer are archived in
[`archive/old_depth_pipeline_20261107/`](archive/old_depth_pipeline_20261107/README.md).

## Layout
```
housing_grasp/        pure NumPy/SciPy library, no ROS
  cloud.py            depth -> cloud: edge filter, voxel grid, outlier removal
  scan.py             fuse views from predefined poses into one dense cloud (ICP; reports calibration consistency)
  color_marker.py            marker mask, marker point cloud, table RANSAC, DBSCAN, marker clusters, housing footprint from the colour image
  color_marker_report.py     plots of the colour-marker stages
  gripper.py          gripper self-scan (finger size, jaw gap vs position) and grasp_held()
  config.py           loads config/grasp.yaml, refuses unmeasured (null) values
ros2_iface/           grasp_node.py (services), arm.py (motion), robot_ros.py (HW7 hardware layer), run_pipeline.py (runner + data collection),
                      export_evidence.py, record.py, smoke_test.py, go_home.py, open_gripper.py, launch/, grasp.rviz, grasp_clouds.rviz
config/grasp.yaml     all parameters; sources of measured values are commented
tests/                pytest, synthetic data (geometry only, not hardware evidence)
archive/              the old depth-based pipeline (snapshot)
logs/                 runtime logs of the stack (not in git)
(node scratch output goes to ../../Media/Grasp/work/{scans,runs}, not in git; the record of a run is its folder in ../../Media/Grasp/)
```

## Run on the robot (ROS 2 Humble, e-stop in reach)
Every terminal starts with the same two lines (identical DDS settings everywhere, mixed settings = silent failures):
```bash
cd ~/Documents/GitHub/VBRM_insertion/insertion_task/Code/Grasp
source ros_env.sh
```
**Terminal 1: everything (stack + views).** Ctrl-C stops it all.
```bash
./run_ros.sh dry_run:=false          # arm link check (sudo only if the USB link is not up), arm driver, D405 camera (point cloud on), MoveIt, grasp node, and TWO RViz windows
./run_ros.sh                          # same but dry run: every motion is only checked, nothing moves
./run_ros.sh dry_run:=false grasp_mode:=head   # optional: mode of /grasp/pick: sticker|head|cloud (cloud mode also takes candidate_search:=on|off|config)
```
The two RViz windows: `grasp.rviz` (robot model + camera image display) and `grasp_clouds.rviz` (fused scan, marker cloud, cleaned scene, segmented head, grasp markers). Add `rviz:=false` for no RViz.

**Terminal 2: live camera view.**
```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
# depth instead: /camera/camera/aligned_depth_to_color/image_raw   (or pick a topic in the drop-down)
```
**Terminal 3 (only if you closed them): the RViz windows on their own** (QT_QPA_PLATFORM=xcb because RViz crashes under Wayland on this machine).
```bash
QT_QPA_PLATFORM=xcb rviz2 -d ros2_iface/grasp.rviz           # robot + camera image
QT_QPA_PLATFORM=xcb rviz2 -d ros2_iface/grasp_clouds.rviz    # point clouds and grasp markers
```
**Terminal 4: run the pipeline** (needs terminal 1 up and the arm near home). With full data collection (video, joint trajectory, logs, scan, plots into `Media/Grasp/<run>/`):
```bash
python3 ros2_iface/smoke_test.py            # first: joints, TF, action servers, camera, IK (never moves)
python3 ros2_iface/run_pipeline.py $PWD/../../Media/Grasp/run_$(date +%Y%m%d_%H%M%S)_mytest --grasp
python3 ros2_iface/run_pipeline.py RUN_DIR --mode head|cloud       # other grasp modes (default: sticker)
python3 ros2_iface/run_pipeline.py RUN_DIR                         # no --grasp: perception only, computes the grasp, never moves to it
python3 ros2_iface/run_pipeline.py RUN_DIR --grasp --skip-scan     # reuse the last scan (saves ~2.5 min)
python3 ros2_iface/export_evidence.py RUN_DIR                            # afterwards: point clouds (PLY), perception overlays, annotated video
```
The runner does: home -> 15-view colour scan -> merge/ICP/voxel 0.8 mm -> table RANSAC + DBSCAN -> colour marker (coarse target) -> camera over it -> grasp computation -> [with --grasp] `allow_grasp` + execute (open, descend, close, lift, hold check).
Without the runner, the same steps are services: `ros2 service call /grasp/<name> std_srvs/srv/Trigger` with `home, scan, color_target, view, sticker_grasp | head_grasp | cloud_grasp, allow_grasp, execute, hold_check`, or all at once with `pick`.
`execute` only checks the motion until `/grasp/allow_grasp` has been called AND the node runs with `dry_run:=false`. Finger-collision and optimizer limits are OFF in `config/grasp.yaml` by user decision (2026-11-07): nothing stops a grasp next to an obstacle.
Other tools: keyboard jog of the arm `Code/Teleop/run.sh` (do not run it while the pipeline moves the arm); calibration scans / hand-eye / intrinsics `Code/Calibration/`; `python3 ros2_iface/go_home.py` joint move to home; `python3 ros2_iface/open_gripper.py`.

## Three grasp modes (same scan, same coarse target, same execute; choose with `run_pipeline.py --mode ...` or `grasp_mode:=...` for `/grasp/pick`)
- **sticker** (default, the one that worked on the arm): close view -> marker sticker pixels lifted to 3D, MEDIAN = grasp point; the housing outline (dark + marker pixels connected to the sticker) is back-projected at the sticker depth -> jaw axis across the body, width. One grasp point, two candidates (jaw flipped), no search.
- **head** = the IMAGE method (`--mode head`, `/grasp/head_grasp`): the same segmentation, but ONLY the head pixels are lifted to a head cloud with their OWN depth, then a GRID SEARCH (no on/off switch) generates thousands of candidates from that cloud: jaw centres every 1 mm x yaw every 5 deg, pad window and gap from the cloud, finger-collision test against the non-head scene points, cost from outline normals, width evenness, centring, contact and gap width. Details: `PIPELINE.md` section 5.2.
- **cloud** (`--mode cloud`, `/grasp/cloud_grasp`; candidate search on/off with `color_marker.candidate_search` or `-p candidate_search:=on|off`; faces with `color_marker.contact_faces`, sizes from `color_marker.extent_source`): COLOUR MARKER REGION ONLY, the black body is not used. Position and height = median of the marker 3D points (stray specks removed); sticker axes and size = min-area rectangle of the marker blob IN THE IMAGE (the 3D sticker points are distorted on the glossy head: plane tilted 73 deg although the sticker faces up). Candidate SEARCH: jaw along the sticker's long or short edge x yaw offsets (-10/0/+10 deg) x 5 pad positions x 2 flips, opening = sticker extent along the jaw + 2 x 12 mm, TCP 8 mm below the sticker; cost = edge prior + centring + yaw; ranked, `/grasp/execute` takes the first with a feasible IK chain.
  Limit: marker data alone cannot say which sticker edge the jaws should close along. `color_marker.jaw_edge: long` is a CONVENTION (the sticker's long edge lies across the head, true on the current head); the black footprint is what measures it in the other modes. If a head is stickered the other way round, set `jaw_edge: short` (the other edge is also in the candidate list, ranked lower).
Why the pads touch the narrow side faces: the approach is straight down, so the jaws close horizontally across the head's narrow dimension seen from above. The sticker (top wide face) is the approach face; the bottom wide face rests in the slot and cannot be reached from above. Pads on the wide faces would need a SIDE approach, which none of the modes has.
All three were checked offline on the same saved frame against the grasp that worked (sticker mode, executed on the arm; head and cloud modes offline only). Details: `../../Media/Grasp/run_20261107_image_mode_offline_validation/`.

## How a grasp reaches the robot (any algorithm)
```
grasp algorithm (analytic housing_grasp, learned net, anything)
   -> TCP pose T_BG in base_link (z = approach, y = jaw axis) + opening width
   -> grasp node _run(): gate (rigid pose, width, workspace, above table floor)
        -> arm.plan_chain: IK pre-grasp (near current joints) + MoveIt straight lines down / lift, all checked BEFORE moving
        -> arm.approach: joint move to pre-grasp, line down  -> /joint_trajectory_controller/follow_joint_trajectory
        -> arm.grip_to: ramp close, stop on finger lag       -> /gen3_lite_2f_gripper_controller/gripper_cmd
        -> held check -> arm.retreat (line up)
```
MoveIt only answers `/compute_ik` and `/compute_cartesian_path`. Trajectories go to the controller directly (like HW7),
because the stock MoveIt config names the controller `gen3_lite_joint_trajectory_controller` while the driver runs
`joint_trajectory_controller`: MoveIt execute cannot work here, and its timeout aborted the old picks (error -6).
No collision checking now (same as HW7). Guards: TCP table floor (`scan.table_z_m` - 5 mm), joint-jump checks, `max_joint_dev_deg`.

External (learned) algorithm: publish once, then call the service. Same gate and motion as the analytic path.
```bash
ros2 topic pub --once --qos-durability transient_local /grasp/proposal std_msgs/msg/String \
  '{data: "{\"T_BG\": [[1,0,0,0.4],[0,-1,0,0],[0,0,-1,0.0],[0,0,0,1]], \"open_width_m\": 0.04, \"source\": \"mynet\"}"}'
ros2 service call /grasp/execute_proposal std_srvs/srv/Trigger
```
Start every motion service from the home pose (HW7 `scan_joints.json` "config 2"): scan poses keep the tool orientation they start with.
Scan poses: `scan.views` in `config/grasp.yaml` (x, y, camera height). Dry run reports which of them MoveIt can plan.
Tests: `python3 -m pytest tests -q`.

## Status (2026-11-07)
Run on the real arm, stage by stage through `run_pipeline.py`: 15-view colour scan (merge, ICP, voxel 0.8 mm), table RANSAC + DBSCAN, colour marker target (reproducible to ~1 mm between three scans), camera over it, green-sticker grasp, execute.
ONE grasp (n = 1) on one head (USB-A in a panel slot, sticker on top): the wrist-camera video shows the connector lifted out of the panel between the fingertips; the finger-position check wrongly said "empty" (thin object, HW7 docs 3.7), user confirmed it worked. Evidence: [`../../Media/Grasp/run_20261107_142037_green_sticker_grasp`](../../Media/Grasp/run_20261107_142037_green_sticker_grasp/).
NOT verified: the wrist-camera hold check on a live held object (the camera dropped off USB right after), `/grasp/pick` as one service call, repeatability (n >= 10), other heads / placements. The finger-collision check and the optimizer limits were switched off by user decision, so nothing in the software stops a grasp next to an obstacle.
The merged cloud smears the head (views disagree by 5-15 mm): only the marker CENTROID of the merged scan is used; the grasp comes from one close frame. Depth on the glossy black head is unreliable (streaks), which is why its housing is measured from the colour image.
