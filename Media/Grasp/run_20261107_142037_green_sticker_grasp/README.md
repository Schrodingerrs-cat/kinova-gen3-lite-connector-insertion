# run_20261107_142037_green_sticker_grasp  (SUCCESS, n = 1: connector held and lifted; the node's finger check wrongly said empty)

## A. Identification
- 2026-11-07 14:20. Real Kinova Gen3 Lite + wrist D405. Grasp pipeline "green sticker": `run_green_pipeline.py --grasp --skip-scan --green-grasp`.
- Objective: grasp the connector head using only the green sticker + colour footprint (no depth-measured housing, no collision check, no optimizer, no gates; user decision 2026-11-07, user at the e-stop). Scan reused: `scans/20261107_140836` (see `scan_used.txt`).
- Hypothesis: sticker median + colour footprint (jaw axis across the housing) is enough to place the fingers around the head.
- Status: **SUCCESS (n = 1)**. The user watched the run and confirmed the gripper held the wire/connector properly after the lift; the wrist-camera video and end image agree (connector between the fingertips, lifted out of the panel). The node reported `GRASP_FAILED: gripper closed on nothing` (finger-position false negative on a thin object, kept as recorded in `metrics.json`). Limits: one trial, one head and placement, no pull test, fingertip drop chosen not measured.

## B. Implementation
- Stages: home -> coarse green target from the merged scan (257.0, -25.0, 4.0) mm -> camera over it at 180 mm -> `/grasp/green_grasp`: green pixels (HSV H 80-105, S>=140, V>=60) back-projected, median -> sticker (256.6, -28.9, 7.6) mm (458 pts); housing = dark+green pixels connected to the sticker within 22 mm, back-projected at the sticker depth -> 24.7 x 13.4 mm, jaw axis (-0.21, 0.98); opening = width + 2 x 12 mm = 37.4 mm; TCP = sticker median z - 8 mm (`green.grasp_drop_m`, CHOSEN, not measured) = (256.6, -28.9, -0.4) mm; approach straight down.
- Checks removed on purpose: finger-volume collision (`grasp.max_collision_points`=1e6), optimizer fail-rate (`max_fail_rate`=1.0), gates. Kept: IK reachability of pre-grasp/descent/lift (candidate flip +1 had no near IK, flip -1 was used), table floor guard.
- Config: `config_grasp.yaml`. Hand-eye = HW7's. Gripper close ramp: stall detection on finger lag, close effort 30.

## C. Results
- Executed pose: recorded tool point lowest at (256.5, -28.8, 34.2) mm = TCP z about -0.8 mm (tool z minus 35 mm tip offset, assumes the tool points straight down); target was (256.6, -28.9, -0.4): within about 0.1 mm xy and 0.4 mm z of the command.
- Gripper after the close: 0.812 rad (empty threshold 0.790), ~0.4 mm width -> node verdict "closed on nothing".
- Timing: execute 9 s; total run 24.6 s after the home move.

## D. Evidence
- **Full package:** `perception_walkthrough.md` (how the scene was perceived, step by step), `perception/` (overlays, green masks, point-cloud views), `point_clouds/*.ply` (fused scan, green cloud, single close frame in colour, green sticker; base frame, metres; open in MeshLab / CloudCompare), `videos/grasp_annotated.mp4`, `scan/` (the scan used, with colour).
- Original list:

`videos/camera.mp4` (wrist camera, 10 fps), `trajectory.csv` (joints, gripper, tool pose, 10 Hz), `screenshots/grasp_frames_t12_to_t23.png`, `screenshots/99_end.png` (connector between the fingertips above the panel, empty slot behind it), `plan/` (plan.json, plots/01_green_sticker_grasp_topview.png), `controller.log`, `grasp_node.log`, `metrics.json` (raw outcome kept as recorded: GRASP_FAILED).

## E. Observations / conclusions
- Seen (video t=18.5-23 s, 99_end.png): fingers around the head, the head lifted out of the panel, metal USB tip up between the fingers.
- Why the node says empty: HW7 `docs/TROUBLESHOOTING.md` 3.7 - the Gen3 Lite gripper position reads ~1.00 for thin or small objects held at the tips (this head is 13.4 mm wide), so `contact = pos < empty` gives false negatives. This pipeline has no wrist-camera hold check (HW7 has `object_in_hand`).
- Not proven: that the hold is secure (no pull/shake test), that this repeats (n = 1), and that the 8 mm drop is right (it was chosen).
- Next: add the wrist-camera hold check to the node, repeat for n >= 10 with the head re-seated each time, then re-enable the collision / robustness checks and see whether the grasp still passes.
