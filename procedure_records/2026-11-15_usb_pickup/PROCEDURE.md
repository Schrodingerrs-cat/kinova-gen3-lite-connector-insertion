# USB-A pickup: successful staged procedure

Recorded 2026-11-15, America/New_York. This is a record of the user-operated trial, not a one-command autonomous routine. No robot commands were issued to create this record.

## Outcome and current state

The user reported that both pads held the rigid USB housing, the first 10 mm lift extracted the connector, the board stayed stationary, and the gripper retained it. Feedback subsequently showed tool XYZ approximately [0.268, -0.023, 0.042] m. A second 10 mm lift was proposed; its execution and endpoint were not supplied. Do not assume the tool reached 0.052 m.

Last reported state: USB held by the gripper; grasp-node dry_run=True, allow_grasp=False, pregrasp_only=True. These are historical reports, not live checks. No release, return, shutdown, or insertion has been performed as part of saving this procedure. Do not rerun pickup while already holding the connector. Arrange a supported release/reset before a future trial.

## Setup and geometry

- Kinova Gen3 Lite, wrist-mounted D405; unchanged camera mount.
- ROS 2 Jazzy, project environment Code/Grasp/ros_env.sh.
- Camera USB 3.2, color/depth 848x480 at 30 FPS; aligned depth and synchronization enabled by launcher.
- Model mode uses CAD_files/USB_A_Plug.glb and precomputed grasp candidates derived from USB A Plug.step. It places candidates from the tag map; it does not fit the full CAD to the depth cloud.
- ArUco DICT_4X4_50, ID 0, actual black-square side 0.100 m; tag lies on table.
- Panel map: xy_m=[0.04952,-0.08958], yaw_deg=4.6, z_m=0.035. The height was supplied by the user from board CAD, replacing an earlier 40 mm measurement. These values depend on the exact tag/board/plug arrangement.
- Housing dimensions from CAD: 15 x 7.5 x 20 mm, excluding metal and cable/strain relief.
- Used plan: Media/Grasp/work/runs/20261115_065819/plan.json, candidate index 1, flip -1. Fingertip XYZ approximately [0.2690,-0.0227,-0.0022] m.
- Gripper TCP offset in tool frame: [0,0,0.035] m; quaternion [0,0,-0.7071068,0.7071068]. See snapshotted config for exact source values.

## Sequence that worked

Commands below document separate operator-controlled stages. They are NOT a shell script to paste/run together. Work from Code/Grasp with source ros_env.sh in command terminals. Require a clear route, cable slack, an accessible emergency stop, and an empty/reset setup before a new pickup.

1. Start ./run_ros.sh dry_run:=true rviz:=false. Check parameters True / True / False for dry_run / pregrasp_only / allow_grasp. Run python3 ros2_iface/smoke_test.py; confirm feedback, camera and current-pose IK.
2. Home was reached via /grasp/home with dry_run temporarily false, then restored true. Full tag visible at home. Six-view scanning was also exercised successfully, but the successful CAD grasp was generated from the home tag view and does not require a fused cloud to determine the model pose.
3. Generate /grasp/model_grasp (std_srvs/srv/Trigger "{}"). Inspect overlay and pose. Run /grasp/execute with dry_run=True for feasibility only. Reject failed/stale plans; the node uses a 60-second age limit.
4. With pregrasp_only=True and a fresh checked plan, call /grasp/allow_grasp, set dry_run false, and call /grasp/execute. The corrected branch opens the gripper and moves only to pregrasp, clears self.last, and returns PREGRASP ONLY. Restore dry_run true and allow_grasp false after the service returns. Verify actual physical alignment.
5. ros2_iface/test_descent.py planned, then with --execute moved down 10 mm over 5 seconds without gripper commands. Start tool approximately [0.2682,-0.0226,0.0923]; measured endpoint [0.2672,-0.0227,0.0817] m. Cable clearance and photos were checked.
6. Set the gripper opening using the action below. Command position 0.6168 rad maps to approximately 31.5 mm in the saved calibration. Reported actual position 0.6068286 rad, reached_goal=True, stalled=False; approximate calibrated gap 33.5 mm. User confirmed no contact and centered housing.

   ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.6168, max_effort: 20.0}}" --feedback

7. ros2_iface/test_grasp_descent.py computes the tool goal from saved candidate 1 and inverse TCP transform. Plan-only checks precede --execute. It moved over 15 seconds from [0.2674,-0.0226,0.0818] to goal [0.2690,-0.0227,0.0328] m; measured endpoint [0.2684,-0.0226,0.0323] m. No gripper command or lift. User confirmed pads flank rigid housing and fingertips clear board.
8. Close gripper ONLY using the following action. User confirmed both pads held the rigid housing. No closing-action result was supplied for this successful closure.

   ros2 action send_goal /gen3_lite_2f_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.81, max_effort: 20.0}}" --feedback

9. ros2_iface/test_lift.py plans a 10 mm upward move from guarded starting XYZ [0.2684,-0.0226,0.0323] m; --execute sends it over 5 seconds, preserving gripper command. User reported USB extracted, board stationary, connector retained. Subsequent smoke test: tool XYZ [0.268,-0.023,0.042] m and IK passed. Stop holding after extraction; no automatic release.

## Code changes and critical limits

- Corrected pregrasp_only branch MUST be outside the conditional that opens an already-closed gripper. An earlier indentation mistake allowed a full grasp sequence when the gripper was already open. Correct structure:

      if self.arm.grip_pos() > 0.05:
          self.arm.grip_to(0.0, gp["close_effort"], rate=0.3)
      if self.get_parameter("pregrasp_only").value:
          self.arm.go_joints(plan["q_pre"])
          self.last = None
          return "PREGRASP ONLY ..."
      # Full approach/closure/lift follow only after that return branch.

- Model mode uses _close_frame() for median depth across several frames. Three stationary tag-height estimates were -38.2,-38.1,-38.2 mm. Repeatability is not absolute accuracy; depth and tag-pose estimates differed by about 2.5 mm in that test.
- Sticker verification was often skipped (<40 usable points). verify_tol_m remains null, so sticker comparison is not an enforced validation gate. Success of model planning is not physical grasp verification.
- The largest-colored-blob edit can select unrelated blue tape. Remove distractions and inspect the mask. Head-mode tuning (dark_v=90, radius=0.015 m, extra depths 0..0.007 m) did not yield a validated pickup; the successful procedure used model mode.
- Removed original first saved scan pose because joint 3=161.099 degrees exceeded installed +/-2.69 rad limits. Six remaining poses passed joint-limit checks. Point-based scan checks are not whole-robot collision checks.
- Standalone scripts and direct gripper actions BYPASS grasp-node dry_run/allow_grasp. --execute is real motion. Never repeat or alter starting-position guards merely to force a move.
- Cartesian helper sets avoid_collisions=False. Paths are kinematic checks, not scene collision validation or force regulation. Gripper max_effort is a command parameter, not a measured pad-force guarantee.
- Earlier full execution incorrectly reported held from a camera heuristic despite near-empty closure; user confirmed plug remained seated. Do not count that as success or rely on camera heuristic alone.
- Existing test_grasp_descent.py contains the specific archived plan path above. Existing test_lift.py still contains the FIRST lift's start guard. Snapshots are trial records, not portable scripts for changed geometry.

## Before reuse

Verify hardware, camera mounting, physical setup, calibration, cable routing, start state and exact selected plan. Generate a fresh plan rather than replaying archived coordinates. Preserve staged checks and stop on any failed service, path, or contact observation. Full end-to-end automation, insertion, repeatability statistics and force-controlled extraction remain unvalidated.

## Snapshot

snapshot.zip contains the current scripts/configs, CAD model and candidate file, source changes, available plan evidence, and this runbook. SHA256SUMS.json records checksums; snapshot_sources.json maps archived paths to source paths. Python source syntax was checked without running the hardware scripts. No git commit, push, or robot-state change was made.
