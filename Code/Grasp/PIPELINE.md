# The grasp pipeline, every detail

Everything below is what the code in this folder does (`ros2_iface/grasp_node.py`, `arm.py`, `housing_grasp/color_marker.py`, `config/grasp.yaml`). Items marked **[verified]** were run on the real arm or checked on real data; **[offline]** = checked only on saved data / synthetic tests; **[not run]** = never executed. Last updated 2026-11-07.

## 0. In one page
```
home -> SCAN 15 views (colour + depth) -> merge, ICP, voxel -> COLOUR MARKER coarse target (table RANSAC, DBSCAN, marker cluster centroid)
     -> VIEW camera 18 cm over that point -> GRASP COMPUTATION (one of 3 modes) -> EXECUTE (IK check, open, descend, close, lift, hold check)
```
| switch | values | where |
|---|---|---|
| grasp mode | `sticker` (default) / `head` / `cloud` | `run_pipeline.py --mode ...`, node param `grasp_mode:=...` for `/grasp/pick` |
| candidate search (CLOUD mode only; the head mode always runs its grid search, no switch; the sticker mode has none) | on / off | `color_marker.candidate_search` in the config, or node param `candidate_search:=on|off|config` (restart the node to change) |
| which side faces the pads touch (cloud mode) | `edge_prior` / `larger` / `smaller` | `color_marker.contact_faces` |
| where those sizes come from (cloud mode) | `sticker` (marker only) / `footprint` (dark + marker outline) | `color_marker.extent_source` |
| which sticker edge the jaws follow (`edge_prior`) | `long` / `short` | `color_marker.jaw_edge` |
| motion | dry run / real | node param `dry_run` (default true; `run_ros.sh dry_run:=false`) AND `/grasp/allow_grasp` for `execute` |

## 0b. The marker colour is a setting, not a rule
The pipeline needs a sticker of ONE distinct colour on the head's top face; it is found by thresholding in the image (`color_marker.h`, `s_min`, `v_min`, OpenCV HSV). It is teal today; any colour works once its range is set. Everything else is named `marker_*` / `color_marker` / `color_target` / `sticker_*` on purpose.
- Hue is 0-179. A colour that wraps past 179 (red) is written with the first number larger: `h: [170, 10]`.
- Measure it once on a saved frame (e.g. a `frame.npz` / `view_XX.npz`, colour is BGR; frames saved by the node are BGR, an old `frame.npz` from before 2026-11-07 is RGB): `python3 -c "import numpy as np, cv2; f=np.load('FRAME.npz'); h=cv2.cvtColor(f['color'][y0:y1, x0:x1], cv2.COLOR_BGR2HSV).reshape(-1,3); print(np.percentile(h, [5,50,95], axis=0))"`, then set `h` to about median hue +-10, `s_min` below the 5th percentile of S, `v_min` below the 5th percentile of V. The wood table is H 12-16 / S ~90, white parts are S < 90, so a saturated colour keeps them out.
- The dark-body outline used by the `sticker` mode and `extent_source: footprint` assumes a DARK housing (`color_marker.dark_v`); the `cloud` mode does not use it.

## 1. Hardware layer (`robot_ros.py`, `arm.py`)
- Joint states `/joint_states`, TF `base_link -> tool_frame`, camera topics `/camera/camera/color/image_raw`, `aligned_depth_to_color/image_raw`, `camera_info`.
- MoveIt (`move_group`) is only asked two things: `/compute_ik` (`avoid_collisions: false`) and `/compute_cartesian_path` (`avoid_collisions: false`, 5 mm steps). Its execution is NOT used (the stock config names the controller `gen3_lite_joint_trajectory_controller`, the driver runs `joint_trajectory_controller`).
- Trajectories go to `/joint_trajectory_controller/follow_joint_trajectory`: joint moves are quintic from the current joints to the target (duration max(2 s, largest joint move / `motion.joint_speed_deg_s` 10 deg/s)); straight lines are the MoveIt waypoints re-timed at `motion.line_speed_mps` 0.04 m/s (min 2 s). Gripper: `/gen3_lite_2f_gripper_controller/gripper_cmd`.
- Guards in `arm.py`: IK must be within `max_dev_deg` of its seed for every joint (no wrist flips); a pose whose TCP z is below `scan.table_z_m - 5 mm` (-46 mm) is refused; one 5 mm line step may not move a joint more than 0.1 rad.
- Not present (switched off or never built): collision checking of the arm/fingers against the scene (only the table floor above), a side approach.

## 2. Scan (`/grasp/scan`) [verified]
1. Start orientation = the tool orientation at the start pose (home). For each of the 15 views in `scan.views` `[x, y, camera height (, tilt deg (, yaw deg))]` a tool pose is built: camera placed at `(x, y, table_z + height)`, optical axis straight down (home orientation), tilted toward `scan.look_at` by `tilt`, spun about base z by `yaw`; tool position = camera position - R * hand-eye translation (`camera.hand_eye_*`, HW7's).
2. IK with seeds [start joints, previous view's joints], accepted if every joint within `scan.max_joint_dev_deg` (80). Views with no solution are skipped (1 of 15 once).
3. Joint move, wait 0.7 s, capture `scan.frames_per_view` (3) depth frames (median, zeros ignored) + the latest colour frame; `T_BC = T_base_tool x hand-eye`. Saved as `Media/Grasp/work/scans/<ts>/view_XX.npz` (depth, color, K, T_BC, T_BT). After the last view the arm returns to the start pose.
4. Per-view cloud: all valid pixels, 3x3 depth-range filter (`cloud.edge_jump_m` 3 mm, removes flying pixels), back-projected to the base frame, voxel `cloud.voxel_m` 0.8 mm, outlier removal (`outlier_radius_m` 3 mm, `outlier_min_nb` 4).
5. Per-view COLOUR MARKER cloud: HSV mask (`color_marker.h` 80-105, `s_min` 140, `v_min` 60, 3x3 opening), flying pixels removed on the FULL depth first (`color_marker.edge_jump_m` 4 mm), then the mask is applied, back-projected, voxel 0.5 mm, outlier removal (2 mm, 3 neighbours).
6. Trimmed point-to-point ICP of every view onto the views already merged (25 iterations, keep the best 70 %, max distance 20 mm, on 3 mm voxel copies); the SAME correction is applied to that view's marker cloud. Merged and voxelised. Output `fused_cloud.npy`, `marker_cloud.npy`, `fusion.json` (per-view shift mm / rotation deg).
- Measured: ICP corrections median 4-6 mm, max 10-16 mm; fused 2.2 M points. The merged cloud smears small objects (views disagree by 5-18 mm, table tilt 1.4-6.5 deg per view: see `Code/Calibration`), so the merged scan is used ONLY for the coarse target.

## 3. Coarse target (`/grasp/color_target`) [verified]
- Table plane: RANSAC (800 iterations, `color_marker.table_thresh_m` 3 mm) on a 150 k random subset of the points within +-30 mm of `scan.table_z_m`, refit on the inliers (normal up).
- Object points: inside `workspace.region`, 3-80 mm above that plane, voxel 3 mm, DBSCAN (`cluster_eps_m` 6 mm, `cluster_min_pts` 20). Marker points above the table: DBSCAN (`marker_eps_m` 3 mm, `marker_min_pts` 15). The biggest marker cluster is the head; the target is its MEAN position (x, y, z). Plots and `color_analysis.json` go to `Media/Grasp/work/scans/<ts>/`.
- The merged patch's orientation / normal is NOT used (registration smear). The label strip (also teal) is removed because it lies on the table (below 3 mm).

## 4. View (`/grasp/view`) [verified]
Camera straight over the target at `view_height_m` (0.18 m), home orientation, IK near the start joints (80 deg), joint move, wait 1 s.

## 5. Grasp computation (close view, ONE frame: no registration error between views)
Common first steps (all modes): latest aligned depth + colour; marker pixels -> 3D points; **sticker position/height = MEDIAN of those points** (median because the depth over the glossy head is unreliable and a mean was pulled by stray specks). The approach is always **straight down** (`z_G = (0, 0, -1)`); the jaws close horizontally (`y_G` = jaw axis, `x_G = y_G x z_G`); pre-grasp = TCP + 60 mm up (`grasp.standoff_m`), lift = 80 mm back along the approach (`grasp.lift_m`). Every candidate is expanded into 2 grasps (jaw direction +/-: the same axis, the gripper yawed 180 deg, which matters for reachability only). TCP height = sticker height - `color_marker.grasp_drop_m` (8 mm, a CHOSEN constant, not measured).

### 5.1 `sticker` mode (default; `/grasp/sticker_grasp`) [verified on the arm, n = 1]
- Head outline in the image: pixels darker than `color_marker.dark_v` (HSV V < 70) or marker-coloured, within `footprint_radius_m` (22 mm) of the sticker, 3x3 closing, connected to the sticker. Back-projected at the STICKER depth (flat assumption). PCA of the outline: long axis (along the cable), short axis (across).
- Jaw axis = the short axis (jaws close across the head). Opening = outline width + 2 x `grasp.open_margin_m` (12 mm). Position = the sticker median xy.
- **No search**: one grasp point, 2 candidates (the flips). It is a formula, not a ranking.

### 5.2 `head` mode, the IMAGE method (`/grasp/head_grasp`) [offline]
- Same outline mask, but ONLY those pixels are lifted to 3D with their OWN depth (no flying-pixel filter, `head_band_m` +-60 mm around the sticker height, voxel 0.5 mm, outliers removed): the head cloud. Needs `min_head_points` (200).
- **Grid search (always on, no switch)**, `grid_grasp_search` in `color_marker.py`:
  1. Grid: jaw centres every `grid_m` (1 mm) over the head's xy extent, kept where the head cloud is.
  2. Yaw: every `yaw_step_deg` (5 deg) over 180 deg at each grid point (jaw axis b, pad strip along a).
  3. Pad window: head points within +-finger_width/2 along a. Gap = 2nd-98th percentile extent along b. The jaw centre must sit within `center_tol_m` (1.5 mm) of the middle of the gap. Opening = gap + 2 x `open_margin_m`, must be <= `max_open_m`. Window needs `min_window_points`.
  4. Contacts: a band of `contact_band_m` (1.5 mm) at each end of the gap, >= 3 points each.
  5. Finger collision: both finger boxes (finger thickness x finger width, from the fingertip up `finger_box_h_m`) may hold at most `max_finger_hits` non-head scene points (`scene_points`: every depth pixel outside the head mask within footprint radius + 25 mm).
  6. Cost = `w_anti` x (1 - antipodal) + `w_even` x width unevenness + `w_center` x distance to the sticker / 10 mm + `w_contact` x (1 - contact / best) + `w_width` x gap / 20 mm. Antipodal = mean of (outline normal . jaw direction) over the two contact bands. The outline normals are horizontal and outward (min-variance direction of each point's xy neighbourhood, `outline_normals`): the 3D PCA normals on this head follow the depth ramp, not the walls, and scored the best yaw 28 deg off the working one (antipodal 0.41 vs 0.77).
  7. TCP height = sticker median height - `grasp_drop_m` (a per-window height is not used: the 3D sticker points ramp 27 mm over 11 mm). Best `max_candidates_kept` (200) go to the execute list, both jaw flips each; all valid ones are plotted.
- Real frame of the working grasp (`Media/Grasp/run_20261107_image_mode_offline_validation/plots_grid_search/`): 268 grid points x 36 yaws = 9648 generated, 2118 valid (7529 dropped because the jaw centre was not in the middle of the head gap; the opening, contact and collision filters removed none on this frame). Best: yaw 105 deg (the working grasp is 102 deg), TCP (256.7, -29.1, -0.4) mm vs (256.6, -28.9, -0.4), gap 12.0 mm, opening 36.0 mm vs 37.4. Offline only, never executed on the arm.

### 5.3 `cloud` mode (`/grasp/cloud_grasp`): marker region only [offline]
- The black body is not used (unless `extent_source: footprint`). Marker points further than `footprint_radius_m` from the median (stray specks) are dropped.
- Sticker rectangle from the IMAGE: min-area rectangle of the marker blob at the sticker; its 4 corners are intersected with the horizontal plane `z = sticker height`. This gives two edge directions (long, short), their lengths, and the centre. (The 3D sticker points are not used for axes: their plane comes out tilted 73 deg although the image shows a flat sticker facing up.)
- **Candidate search (`candidate_search: true`)**: families = jaws along the LONG edge or along the SHORT edge (pads then touch the faces perpendicular to the jaw; `face_width_m` = the other extent) x yaw offsets `yaw_steps_deg` (-10, 0, +10) x `n_offsets` (5) pad positions across the sticker (within +-min(`offset_span_m`, extent/2 - finger_width/2)) x 2 flips. Opening = extent along the jaw + 2 margins (rejected if over `gripper.max_open_m`). Cost = family prior (0 for the preferred family, 0.5 for the other) + 0.3 x centring + `yaw_penalty` (0.3) x |yaw| / 45. Typically 30 candidates x 2 flips.
- **Search OFF**: ONE candidate = preferred family, yaw 0, centred (the top of the searched list).
- Preferred family: `contact_faces: edge_prior` -> `jaw_edge`; `larger` -> jaw along the SHORTER extent (pads on the wider faces); `smaller` -> the opposite. **The extents come from `extent_source`**: from the sticker (marker only) or from the head outline.
- **Important, measured on the real head**: the sticker's proportions are OPPOSITE to the head's (sticker long edge 12.5 mm lies ACROSS the head, whose long axis 24.7 mm runs along the cable). So `larger` + `sticker` extents picks the jaw 79 deg off (along the cable); `larger` + `footprint` picks 0 deg off (the grasp that worked); `edge_prior` with `jaw_edge: long` is 11 deg off (covered by the +-10 deg yaw candidates). Marker data alone cannot tell which edge is right; `jaw_edge` is a convention for how the sticker is applied.

### 5.4 Which faces the pads touch
Straight-down approach: the jaws can only close across the head seen from above, so the pads always touch SIDE faces, never the top (sticker) or bottom (rests in the slot) faces. The working grasp (jaw across the 13.4 mm dimension) touched the two long side faces (24.7 mm wide), the larger-area pair that is reachable. Touching the wide faces would need a side approach: not built.

## 6. Execute (`/grasp/execute`) [verified on the arm, n = 1]
1. Candidates are tried in rank order, at most `max_candidates` (4) grasps: for each, `plan_chain` checks pre-grasp IK near the current joints (`grasp.max_joint_dev_deg` 120), the straight descent and the straight lift (MoveIt line paths, joint-jump check, table floor). The first feasible one is used (in the working run the first flip had no near IK and the second was used).
2. Unless `dry_run:=false` AND `/grasp/allow_grasp` were called, it stops here ("NOT EXECUTED, motion checked only").
3. Open the gripper to the position that gives the opening (table `gripper.position_to_width_m` from `/grasp/scan_gripper`), ramp rate 0.3. Joint move to the pre-grasp. Straight descent to the TCP target.
4. Close: smooth ramp to `gripper.close_cmd` (0.81 rad) at rate 0.2, effort 30. While closing, after 15 % of the ramp, if the fingers lag the command by more than 0.05 (fraction of full) for 3 consecutive 40 ms steps = contact: squeeze +0.04 and hold 0.5 s. `held` (finger check) = contact or final position below `empty_frac` x close_cmd (0.790).
5. Lift 80 mm along the retreat direction.
6. Wrist-camera hold check after the lift (HW7 rule): median of 3 depth frames; depth between the fingertips (rows 400-480, cols 340-510) below `hold_check.tip_max_m` (0.20 m) AND the table in the lower corners (rows 320-480, cols 0-255 and 595-848) at least `margin_m` (0.08 m) farther. Success = finger check OR camera check [camera check: **not run** on a held object yet; the finger check alone gave a false "empty" on the working grasp: thin objects read ~1.0].
7. The object stays held, lifted; nothing releases it.

## 7. Data collected
`run_pipeline.py RUN_DIR --grasp [--skip-scan] [--mode ...]`: records camera video + joints/gripper/tool pose at 10 Hz (`record.py`), stage logs and results (`controller.log`, `metrics.json`), copies the scan and the plan folders and plots. `export_evidence.py RUN_DIR`: PLY point clouds, perception overlays, annotated video. Layout rules: `../../Media/RULES.md`; runs live in `../../Media/Grasp/`.

## 8. What is switched off / assumed (read before trusting a run)
- Finger-volume collision check and the optimizer limits: removed with the archived depth planner; nothing checks fingers against the panel, bolts, post or cable. Table floor only.
- The 8 mm drop, the 12 mm margin per side, the `jaw_edge: long` convention and the +-10 deg yaw steps are chosen values.
- Needs a TEAL sticker on the head's top face, visible from above; a bigger teal object in the workspace would be picked instead. The head must be inside `workspace.region` and the close view reachable by IK.
- Not run on the arm: `head` and `cloud` modes, the hold check, `/grasp/pick` as one call, repeats (n >= 10).
- `color_marker.normal_knn`, `max_nz`, `min_plane_points` and `color_marker.marker_planes()` belong to the earlier two-plane idea and are not used by any mode.

## 9. Files
`ros2_iface/grasp_node.py` services; `arm.py` motion + gripper; `robot_ros.py` ROS layer; `run_pipeline.py` runner; `record.py`, `export_evidence.py`; `housing_grasp/{cloud,scan,color_marker,color_marker_report,gripper,config}.py` library; `config/grasp.yaml`, `config/gripper_scan.yaml`; `tests/`; `archive/` the old depth planner. Teleop: `../Teleop`. Calibration: `../Calibration`.
