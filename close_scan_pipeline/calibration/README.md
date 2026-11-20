# Camera calibration (2026-11-19)

Final decision for now: keep the hand-eye that is already in use and the factory camera intrinsics. Values and numbers: `camera_calibration_values.yaml`.

| folder | what | result |
|---|---|---|
| `test1_marker_hand_eye/` | 100 mm ArUco marker, 18 poses, 16 samples | new hand-eye only slightly better on held-out poses (4.1 vs 4.9 mm), not applied |
| `test2_chessboard/` | 8 x 6 board (30 mm squares), 19 poses | factory intrinsics 0.245 px held-out reprojection (fine); board hand-eye not better than the one in use, not applied |

Each folder has `plan.json`, the samples/captures, `solve.json`, `console.log`, `README.md` (and frames). Re-run with `python3 ~/kinova/work/tools/auto_hand_eye.py` and `auto_chessboard_cal.py` (plan only by default; `--execute` moves the arm; `--help` has the whole procedure).
