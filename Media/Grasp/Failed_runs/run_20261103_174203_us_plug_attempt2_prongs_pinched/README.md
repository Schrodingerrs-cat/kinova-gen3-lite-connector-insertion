# Run 20261103_174203: plug attempt 2 (FAILED)
- **Date:** 2026-11-03 | **Controller:** analytic housing grasp (Task A grasp front end) + MoveIt Cartesian pick | **Platform:** REAL Kinova Gen3 Lite + D405 (eye-in-hand), no simulation
- **Object:** black 2-prong US power plug (unknown dimensions to the code) on a wooden table
- **Code version:** no git repo; `Code/Grasp/` as of this date. The plots here re-analyse the saved frame with the CURRENT pipeline (cleaned cloud); the attempt itself ran the PRE-cleaning pipeline.
- **Calibration:** HW7 hand-eye and 0.035 m tool tip offset (same robot/camera; not re-verified). Finger size and gripper position->width table are PLACEHOLDERS (collision check / open width only).
- **Video:** none recorded (UNKNOWN timing; only frames below).

## Objective / hypothesis
Same as attempt 1 with open margin 12 mm per side (47 mm opening) and the camera re-centred over the plug.

## Result: `FAILED`
Executed without MoveIt errors; gripper stopped at 0.800 (fully closed) so the check reports "closed on nothing". The wrist view after the lift shows the plug gone from the table and its metal prongs between the fingertips, i.e. the plug was lifted but whether by the body or only pinched by the thin prongs is UNKNOWN (body out of frame). Not a verified clean grasp.
See `failure_analysis.md`.

## Evidence
- `screenshots/01..04` (planning view + ROI, planned jaw line on the planning view, wrist view after the lift, tool-tip projection check)
- `plots/01..04`, `pointclouds/`, `metrics.json`, `metrics_pipeline.json`, `config.json`, `controller.log`
