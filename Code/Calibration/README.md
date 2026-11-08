# Calibration: scan consistency, hand-eye, camera intrinsics
Self-contained (own `robot_ros.py`, `arm.py`, `ros_env.sh`, `calibration.yaml`). Why it exists: the merged multi-view cloud of the grasp pipeline smears objects by 5-15 mm, because the camera pose of each view is `T_base_tool x hand-eye` and the hand-eye (and intrinsics) are not exact. These tools MEASURE that error and can improve the hand-eye. They never edit the grasp config; they print what to paste.

## 1. Calibration scan (is the calibration consistent?)
```bash
source ros_env.sh                     # every terminal (same DDS settings as the rest)
python3 calib_scan.py                 # EMPTY table in scene.region, arm at home: visits calibration.yaml calib_scan.views, saves every view, analyses
python3 calib_scan.py --analyze DIR   # analyse saved views again (also works on a grasp scan folder, e.g. Media/Grasp/<run>/scan/<ts>; link the view_*.npz files into results/)
```
Fits the table plane in every view (RANSAC) and reports tilt (deg) and the table height at the region centre. Perfect calibration: tilt 0 and the same height in every view. Output: `results/scan_<ts>/calib_scan.json`, `tilt_per_view.png`, `height_per_view.png`.
First measurement (the real 14-view grasp scan of 2026-11-07, head and panel inside the region so only approximate): tilt 1.4-6.5 deg (mean 4.0), table height per view -30 .. -48 mm (std 4.9 mm, range 17.8 mm, mean -41.0 mm = the configured table height). See `results/analysis_of_grasp_scan_20261107_140836/`.

## 2. Hand-eye calibration (ArUco marker)
```bash
python3 hand_eye_calibration.py            # interactive: c capture, u undo, s solve + save, q quit
python3 hand_eye_calibration.py --from results/hand_eye_<ts>_samples.json    # solve again, no robot
```
Print an ArUco marker (`aruco` in `calibration.yaml`), measure the black square with a ruler into `aruco.marker_size_m` (null until you do: the tool refuses), stick it FLAT on the table. Move the arm with `Code/Teleop/run.sh` in another terminal, capture 12+ poses with the marker visible and the camera ROTATED between poses. The solver (Park-Martin, `calib_lib.hand_eye`) uses all pairs; the quality number is how much the FIXED marker moves in the base frame (std / max in mm): lower is better, compared against the current hand-eye. Paste the printed 2 lines into `Code/Grasp/config/grasp.yaml` only if NEW is clearly better, then rerun the calibration scan to confirm.

## 3. Camera intrinsics check (chessboard)
```bash
python3 camera_intrinsics.py               # interactive: c capture, u undo, s solve + save, q quit
python3 camera_intrinsics.py --from results/intrinsics_<ts>_corners.json
```
Fill `chessboard.square_size_m` (measured) and `inner_corners` first. Reports the RMS reprojection error (< ~0.5 px is good) and compares K with the factory values from `camera_info`.

## Tests (synthetic, true answer known; NOT hardware evidence)
`python3 -m pytest tests -q`: hand-eye exact without noise and within 3 mm / 0.7 deg with marker noise, plane tilt and height, intrinsics recovery (< 1%), ArUco detection and distance, view poses.
Verified offline: the hand-eye `--from` path recovered a planted 3 mm / 1 deg error (marker spread 2.5-3.0 mm -> 0.35-0.53 mm std); the intrinsics solver matched the factory values within 1.8 px. NOT yet run on the real arm: the capture sessions and a calibration scan with the robot (arm driver and camera were down).
