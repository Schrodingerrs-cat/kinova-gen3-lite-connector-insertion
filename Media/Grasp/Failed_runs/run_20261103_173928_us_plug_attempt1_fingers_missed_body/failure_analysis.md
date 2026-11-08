# Failure analysis, attempt 1
## 1. Expected behaviour
Jaws open wide (33 mm), straddle the plug body, close on it; verifier reports an object between the fingers.
## 2. Observed failure
Gripper closed fully (0.800). Plug displaced on the table. Planned grasp point projects beyond the body in a fresh view.
## 3. Evidence
`screenshots/`, `metrics.json`, `plots/02`, `plots/03`.
## 4. Root cause
- **Verified:** the miss is not a MoveIt/controller failure (no execution error). The plug was disturbed.
- **Suspected (untested):** open margin was only 6 mm per side (33 mm opening) against ~10 mm camera-to-base inconsistency seen across viewing heights (housing centre 367/370/373 mm x at 0.18/0.22/0.28 m); a finger landed on the plug.
- **Unknown:** whether the residual error is camera extrinsics, depth bias or TCP definition; no wrist frame was captured at the grasp pose.
## 5. Corrective actions
Open margin raised to 12 mm per side; cleaner cloud / ROI-independent segmentation. Next: capture a wrist frame at the grasp pose.
## 6. Status: `FIX_ATTEMPTED` (validating run: attempt 2 did not give a verified grasp, see `../run_20261103_174203_us_plug_attempt2_prongs_pinched/`)
