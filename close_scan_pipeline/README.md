# Our work (kinova insertion project)

Naming: no people's names in files or folders. The two existing grasp pipelines are called **method 1** (main branch of the team repo: sticker-relative tilted close views, CAD fit with panel plane) and
**method 2** (the CAD multi-view branch: fixed nearby views shifted sideways, CAD fit, staged pickup). What we build next is **method 3**. Runs are numbered `test1`, `test2`, ...

```
~/kinova/work/
  tools/          run_stack_method2.sh     starts arm link, driver, camera (enable_sync), MoveIt and method 2's grasp node (dry run)
                  run_method2_full.py      method 2 pipeline (or --replay-fit grasp from a saved fit) into a run folder; plan/help without --execute moves nothing
                  auto_hand_eye.py         automated, safety-checked hand-eye capture around the ArUco marker (plan only by default)
                  auto_chessboard_cal.py   same for the chessboard: intrinsics check + hand-eye (plan only by default)
                  open_rviz.sh / stop_stack.sh   open the saved RViz scene on a running stack / stop the whole stack
                  add_fixture_to_scene.py  CAD fixture + table into MoveIt's planning scene (the stack script does it at start); arm.go_joints then refuses joint moves that collide
                  stage1_find.py           STAGE 1 trial: live (3 camera views, no grasp) or --run on saved views; plug pose + pass/fail, logs grasp_runs/stage1_trials.csv
                  stage1_close.py          STAGE 1 check for the close scan: wide-face yaw from the left and right views, plug centre (leave-one-view-out), board-frame position; logs grasp_runs/stage1_close_trials.csv
                  plug_points.py, plug_box.py, fit_plug_local.py, clean_scan.py, align_views.py   helpers/diagnostics behind it (plug-only points, box fit, CAD fit around the board prior, depth cleaning, per-view ICP)
                  report_run.py            plots, metrics.json and README.md for a run folder (run automatically after stage 1 and after a grasp)
                  put_back.py              reverse of a pick: down to the grasp pose, open, lift clear
                  connector.py + connectors/usb.yaml, aux.yaml, power.yaml   per-connector settings; every tool takes --connector
                  grasp_plug.py            grasp from a PASSED stage 1 close scan: measured depth, jaws across the thin axis, straight-down approach; plan only unless --execute (--fresh = scan, check, grasp)
                  scan_orbit.py            close scan: detect marker, close pose centred over it, sweep left to right tilted toward it (plan-only by default); add_fixture_to_scene.py --include-plugs for scans
                  scan_grid.py             grid of camera views (front-back x left-right, small steps) saved like method 2's views; plan-only by default; then the stage 1 check
                  board_yaw.py, find_plug_m3.py   offline stage 1 on saved views (scan folder): board yaw from the white strip, plug top-face position and yaw with cable/edge pixels removed
                  mapping_check.py         record (no motion) the cloud at several arm poses, compare: offsets between views = camera-to-base mapping error
                  align_fixture_to_cloud.py   ICP of the CAD cradle to one live cloud (unstable, nominal CAD placement is used instead)
                  fixture_markers.py, export_setup_parts.py   publish the CAD fixture in RViz / one-off export of the CAD parts
  calibration/    camera_calibration_values.yaml (values in use + candidates), test1_marker_hand_eye/, test2_chessboard/
  snapshots/      usb_pipeline_20261119 = the tools and settings as they ran for the first confirmed pick
  grasp_runs/     one folder per run (method1_cad_*, method1_head_*, method2_full_test1..8, method2_replay_test1), analysis/ (gate plots, offline fit)
  shared/         method2_run_evidence(.zip), method2_attempts.zip   (packages to send to teammates)
  setup_cad/      the setup assembly (STEP, mesh, per-part STL), fixture_pose.yaml (where the fixture sits relative to the arm: edit to tune), layout_views.png
  rviz/           setup.rviz = saved RViz scene (arm + CAD fixture + camera image); run_stack_method2.sh opens it automatically (rviz:=false for none)
~/kinova/ins/
  repo/           team repo checkout (method 1 = main branch) with our local edits (joint-limit/fault/arrival checks, slower motion, plotting-import fix, node-name fix, tests)
  method2_src/    method 2 source (git worktree of the CAD multi-view branch; one flag added to its runner)
```
Run order for a session: `tools/run_stack_method2.sh rviz:=false` (own terminal) -> arm home -> tool. Every tool prints its procedure with `--help`.
