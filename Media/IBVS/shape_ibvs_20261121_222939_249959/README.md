# Marker servo, first hardware run: CONVERGED (free space, no plug)

- Date 2026-11-21. Marker ID 11 (audio socket, DICT_4X4_50, 30 mm), goal taught with `teach_goal.py` at the leveled post-grasp pose (corner std 0.06 px).
- Start: arm nudged +3 deg on joint 1 from the taught pose (center error 42.9 px, size 100.5 %).
- Controller: default conservative profile (no `--fast`), `--live`, gripper open and empty, collision scene from depth with the remeasured gripper self-depth reference.
- Result: `summary.json` status CONVERGED. 8 steps, 12.2 s, final max corner error 4.73 px (tolerance 5 px for 5 fresh frames), 0 tracking recoveries, tool rotation 2.99 deg (= the nudge undone).
- Step timing (`loop_timing.jsonl`, mean of 8): choose 0.02 s, collision check 0.01 s, move 1.27 s, observe 0.18 s, cycle 1.47 s (0.67 Hz). Motion time dominates.
- Evidence: `ibvs_correction.mp4`, `converged.png`, `converged_features.png`, `trial.csv`, `loop_timing.jsonl`, `motion.jsonl`, `config.json`.

## Limits of this result
- The goal was taught at the same pose the arm returned to, so this shows tracking, depth, collision checking and the control loop work on the real arm. It does NOT show plug-to-socket alignment: no plug, no taught pre-insertion pose, no insertion, no seating check.
- 4.7 px at 289 mm is about 3 mm; the run stops as soon as it is inside the 5 px tolerance.
- Hand-eye in `grasp.yaml` is still the old estimate; the camera/tool tilt against the CAD wall normal is about 4 deg by that estimate (the camera itself measured the wall level to within 0.5 deg).
- One run, one marker, small perturbation (3 deg).
