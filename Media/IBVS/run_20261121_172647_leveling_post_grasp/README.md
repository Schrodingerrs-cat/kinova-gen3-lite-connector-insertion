# Leveling the post-grasp pose on the wall (hardware, no plug)

Why: the post-grasp pose recorded 2026-11-18 looked at the wall tilted. Measured from the camera: wall-plane normal 19.5 deg (about camera x) and 11.8 deg (about y) off the optical axis, marker row rolled 4.5 deg, wall 286 mm away.

Method (`level_view.py`, closed loop, arm moved only by joint trajectories): each iteration grabs a color + aligned-depth frame, fits the wall plane (RANSAC, 4 mm inliers) on the brown board below the markers, fits the line through the centres of markers 10, 11, 12, builds the camera rotation that squares the view, converts it to a joint step with the camera Jacobian (URDF FK, old hand-eye) at 0.8 gain, translation held. Aborts if the tilt grows, if a joint step exceeds 30 deg, or if the fingertips would pass x 0.50 m / z 0.

Result (`leveled_pose.json`): tilt about cam x / y / marker roll
- before: 19.50 / 11.75 / 4.54 deg
- iteration 1: -0.94 / 1.03 / 0.13 deg
- iteration 2: 0.07 / 0.47 / 0.06 deg (all under the 0.5 deg stop)
New pose joints (deg) [36.2, -82.0, 134.28, -49.16, 122.4, -66.71] written to `Code/Grasp/config/grasp.yaml` (`post_grasp`); the old pose is kept in a comment there.

Limits: this levels the CAMERA against the wall as the depth camera sees it. With the old hand-eye estimate the tool axis is still about 4 deg off the CAD wall normal, so tool-vs-wall alignment needs the new hand-eye calibration. `snapshot.py` is the frame grabber the script calls (it expects to sit next to `level_view.py` and the generated `cad/gen3_lite.urdf`).
