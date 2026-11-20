# Close-scan connector grasping: context, methodology, status

Branch `sahil`. Kinova Gen3 Lite, wrist-mounted RealSense D405, ROS 2 Humble. Work from 2026-11-18 to 2026-11-19.
Goal: find a cable connector in its cradle with the wrist camera, then pick it. USB-A plug first, then the aux (6.35 mm) jack, then the power plug.
This folder holds the tools, the per-connector settings, the run results and the reasons behind each design choice. The team's own grasp code was changed only for safety (first commit on this branch). Method 2's local edits are patch files in `method2_patches/`.

## Status (honest)

| item | state |
|---|---|
| USB plug, stage 1 (find) | passed on 4 close scans, same layout |
| USB plug, grasp | 1 pick confirmed by eye (run G2, camera rule HELD). G1 closed on the plug but the old script released it from the finger reading (bug, fixed) |
| Aux jack, stage 1 and grasp | 1 pick confirmed by eye (run AUX1): stage 1 passed, camera rule HELD, operator confirmed the jack was picked up properly; put back with `put_back.py` afterwards |
| Power plug, stage 1 and grasp | 1 pick confirmed by eye (run POW1): stage 1 passed, camera rule HELD, held 8 s, then the jaws opened where the arm was (no put-back) |
| Placement and cable trials (3 x 3, pass at 7 of 9) | not started |
| Repeatability numbers | none yet. All runs are one layout |

Limits to keep in mind
- Stage 1 gates check that the measurements agree with each other, not that they are accurate. The grasp is the accuracy test. The gate thresholds were set while looking at the first few scans.
- Absolute position differs by about 7 mm in x between the tilted close scan and the earlier near-top-down scans (cause not resolved). Position relative to the board strip agrees to under 1 mm between them. The grasps used the close-scan coordinates and worked.
- Camera-to-base error depends on camera position: the estimated plug y moved 7.7 mm per 100 mm of sideways camera shift. A symmetric sweep cancels the linear part. A hand-eye refit from static-scene views was not done.
- Fingertip clearance for the aux is 6 mm over a board height known to about 2-3 mm.
- No force or torque sensing. The gripper reports only a position and cannot see thin parts.
- Power plug: the CAD size (26.5 mm square, 91 mm) does not match the real part (bright-teal lumpy body about 15 x 21 mm, top 30 mm above the board, thick cable from its top). Its leave-one-out tolerance is 6 mm, not 3 mm, because each view only sees its own side of the lump (per-view centres spread 10 x 21 mm). Fingers pass about 14 mm from the aux at the 52 mm jaw opening.
- Aux window height cap comes from the CAD (9 mm too tall for the real aux), so a little cable root can enter the plug points; the centre still agrees with the sticker blobs to about 1 mm.

## Final methodology

Stage 1, find the connector (`tools/scan_orbit.py`, `tools/stage1_close.py`)
1. Marker from the home view: dark teal sticker (hue 80-105, saturation >= 70, value 35-140). The blob nearest the connector position predicted from the board is used, because several stickers are visible.
2. Close pose: camera straight above the marker at the profile height above the board top (USB 0.228 m, aux 0.200 m). Capture, re-detect the marker (the home estimate is typically 8-10 mm off), re-plan the sweep if it moved more than 8 mm.
3. Sweep: 5 poses from the left to the right of the marker (USB +-70 mm, aux +-50 mm), constant height, each aimed at the connector, so both side faces are seen. Camera roll is free and chosen for the smallest joint motion.
4. Reference frame: the white board strip top face (about 151 x 51 mm), found by height. Its yaw (spread under 1 deg between views) and centre place the CAD relation, which predicts the connector position and height window.
5. Connector points: within the profile radius of the predicted position, from the board top plus 3 mm up to the predicted top plus a profile tolerance (USB 6 mm, power 60 mm), depth-edge pixels removed. USB and aux use sticker-teal or dark pixels. Power uses bright teal pixels only (hue 75-105, saturation >= 100, value >= 80), because dark pixels swept the thick cable in (38-48 mm "footprint"). A morphological opening to drop a cable only works when the part is clearly wider than the cable, so it is an option (`points.open_mm`), not used by any current profile.
6. Yaw (USB only): surface normals of the plug's wide faces, seen from the left (+y normal) and from the right (-y normal). They agree within 0.3-2.7 deg and match the strip direction. The aux is round and has no yaw cue.
7. Position: box centre of the merged points, and its shift when each view is left out in turn.
8. Gates (USB): at least 200 points per view and the board yaw spread <= 2 deg; left and right face yaw within 5 deg of each other (>= 150 px each) and the mean within 10 deg of the strip; leave-one-out centre <= 3 mm. Aux: 100 points per view, no yaw gate, same centre gate. Power: 200 points per view, no yaw gate, centre tolerance 6 mm.

Grasp (`tools/grasp_plug.py`)
1. Depth is measured: board top from the strip, part top from the points (USB: 90th percentile of the points near the centre; aux and power: 90th percentile height of the bright sticker blob nearest the centre, median over views, per-view spread must stay within a profile tolerance). The run stops if the band does not fit under the top.
2. Fingertip clearance above the board (USB 12 mm, aux 6 mm, power 10 mm), jaw band 12 mm above it, straight-down approach, jaws along the part's thin axis (aux and power: along the board strip direction). Both wrist branches are planned and the one with less joint travel is used.
3. Pre-grasp 90 mm above (power 120 mm), jaws opened for the whole descent (USB and aux 31.5 mm, power 52 mm). Descent 0.04 m/s to 20 mm above, photo, then 0.015 m/s to the grasp pose. Close, lift 5 mm, lift 80 mm.
4. Hold decision: the wrist camera after the lift. Held if the depth between the fingertips is under 0.20 m and the table in the lower corners is at least 0.08 m farther. The finger position is only logged. If the camera says nothing is held, the jaws open and the run stops.
5. After a confirmed pick: `--hold-then-open S` waits S seconds so the operator can look, then just opens the jaws where the arm is (the operator asked for no put-back). `tools/put_back.py` still exists and reverses a pick (down to the grasp pose, open, lift clear); it was used for the USB and aux picks.

Safety layers: joint limits minus a margin, joint-step screen (<= 5 deg per 2 mm sample), fault check and endpoint-arrival check on every leg, fingertip height floors, MoveIt planning scene (CAD fixture placed from the measured board, table, plug hulls) checked along every joint move, a pre-planned abort leg, slow speeds, a stage-by-stage photo record. Keep the e-stop within reach.

## Why the method looks like this (decisions and evidence)

- Method 2's CAD fit was dropped for stage 1. Its colour rule (dark or saturated) picked up the black cable; the fitted plug sat 20 mm too high and the joint cloud had a streak to 65 mm. The 15 x 7.5 mm CAD bar is narrower than the visible plug, so a nearly square blob gave no yaw preference (yaw spread 24-83 deg top-down, flat score over +-60 deg). Its earlier "successful" grasp ended with the gripper at 0.810 and a 0.4 mm gap, which the finger reading cannot interpret, so it was never a verified grasp.
- Straight-down views cannot see the flat faces. The close sweep sees both sides, which gives yaw. Outline yaw from the top view was noise (circular spread 34-39 deg) and its apparent agreement on two scans was luck; that gate was removed.
- The marker gate had to be loosened: the sticker is dark teal (median saturation 97-120, not >= 140). A value cap of 140 keeps the pale blue board out.
- The board strip is the reference: it is seen in every view, and the plug position relative to it repeats to under 1 mm between very different scan styles. The CAD fixture in the scene is placed from it; with only the strip's position and yaw, the CAD plug lands 4.2 mm from the measured plug.
- Grasp bug and fix: the Gen3 Lite fingers are underactuated and only the driven joint is reported, so a thin part pinched at the tips reads as fully closed (0.812, gap 0.4 mm). The first USB attempt closed on the plug and the script released it from that reading. The camera after the lift now decides. An earlier suggestion that the plug was missed by 10-13 mm was parallax (camera 5 cm to the side) and is withdrawn.
- Depth measured, not assumed: USB stands 47 mm above the board (CAD 43 mm); the aux barrel reaches 23.7 mm (CAD 32.6 mm) and its cable joins at 27-34 mm. Using the CAD numbers would have put the aux jaws in the wrong place.
- The jaw axis has two equivalent directions; one needs a 180 deg wrist turn from home and has no nearby inverse kinematics solution, so both are planned and the nearer one is taken.
- The power plug's CAD part is wrong in size (26.5 mm square against a measured ~15 x 21 mm body) but right in position (within about 3 mm). The profile uses the CAD only for the position prior and records the measured geometry. Cable direction was measured (leaves toward the arm, rising to 45-90 mm), which is why the jaws close along y and the pre-grasp is higher. The opening was widened for the large connector (user) but capped at 52 mm because the aux stands 52 mm away.
- Settings per connector live in `connectors/usb.yaml` (unchanged since the confirmed pick), `connectors/aux.yaml` and `connectors/power.yaml` (reasons next to each value). Every tool takes `--connector`.

## Evidence gaps against the repo's run rules (`Media/RULES.md`)

- Our run folders have `README.md`, `metrics.json`, `stage1_close.json`, `orbit.json`, `plots/` and, for grasps, the wrist-camera photos and `grasp.json`. They do not have `config.json`, `trajectory.csv`, `controller.log` or arm videos. The settings are in `connectors/*.yaml` and are also recorded in each run's metrics.
- The raw scan frames (`method2_perception/scan/*.npz`) stay on the lab laptop and are not committed (size). Stage 1 can be re-run offline from them.
- Run G1 (closed on the plug, then released by a software bug) is kept in `runs/` and labelled in its README, not moved to a `Failed_runs/` folder. Its photos are in `runs/run_20261119_212527_orbit_G1/grasp/`.
- Leaving out CAD-fit stage 1 (method 2) in favour of the close scan with side views was decided with the user, with the evidence above, and is a deviation from method 2's approach.
- The three picks (USB G2, aux AUX1, power POW1) are single attempts on one layout. The hold was judged by the wrist-camera rule and confirmed by the operator by eye for all three. There is no independent sensor.

## Run it

Paths in the tools assume this folder is at `~/kinova/work` and the team repo at `~/kinova/ins/repo`, with method 2 at `~/kinova/ins/method2_src` (see `method2_patches/`).
```
source ~/kinova/ins/repo/Code/Grasp/ros_env.sh
tools/run_stack_method2.sh rviz:=false                       # own terminal; arm at project home, gripper open
python3 tools/scan_orbit.py --connector usb                  # plan only, moves nothing
python3 tools/scan_orbit.py --connector usb --execute --name T1
python3 tools/stage1_close.py RUN_DIR --connector usb        # gates, plots, README, log line
python3 tools/grasp_plug.py --run RUN_DIR --connector usb    # plan only
python3 tools/grasp_plug.py --fresh --execute --connector usb --name G3
python3 tools/grasp_plug.py --run RUN_DIR --connector power --execute --hold-then-open 8   # hold 8 s, then just open
python3 tools/put_back.py RUN_DIR/grasp/grasp.json --execute  # optional: reverse a pick
```
Every stage 1 and grasp run writes `plots/`, `metrics.json` and `README.md` into its run folder (`tools/report_run.py`).

## Files

- `tools/`: scan_orbit, stage1_close, grasp_plug, put_back, report_run, plug_points, board_yaw, add_fixture_to_scene (CAD fixture and plug hulls into MoveIt), fixture_markers (RViz), run_stack_method2.sh, and the diagnostics used along the way (fit_plug_local, plug_box, align_views, clean_scan, mapping_check, scan_grid, calibration scripts).
- `connectors/`: per-connector settings. `setup_cad/`: part index, fixture placement, part meshes. `calibration/`: hand-eye values in use and candidates. `rviz/`: saved scene.
- `runs/`: light artifacts (plots, metrics, grasp photos) of runs o1, P1_A, G1, G2 (USB), AUX1 and POW1 (power), plus `usb_results.md`, `aux_results.md`, `power_results.md`, `stage1_close_trials.csv`. Raw frames are not committed.
- `method2_patches/`: the local edits to method 2 (planning-scene path check in `Robot.move_joints`, an `--allow-pickup` flag, the sticker colour gate block).
- Not committed: the 93 MB STEP and 36 MB GLB of the setup (regenerate the part meshes with `tools/export_setup_parts.py`), raw scan frames, videos.
