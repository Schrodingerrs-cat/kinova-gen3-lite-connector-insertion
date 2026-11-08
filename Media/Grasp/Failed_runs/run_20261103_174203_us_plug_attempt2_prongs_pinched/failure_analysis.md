# Failure analysis, attempt 2
## 1. Expected behaviour
Fingers straddle the plug body (planned jaw line crosses the body, `screenshots/02`), close, verifier reports an object.
## 2. Observed failure
Closed to 0.800 (empty); wrist frame afterwards shows prongs between the fingers.
## 3. Evidence
`screenshots/02-04`, `plots/03_cross_section_at_grasp.png`, `metrics.json`.
## 4. Root cause
- **Verified:** perception is stable (same housing size/centre for any ROI box, `../../run_20261103_roi_size_invariance_us_plug/`). The tool-tip projection (TF + HW7 hand-eye) lands between the fingertips in the image (`screenshots/04`), so near-field camera-to-tool geometry is consistent.
- **Verified:** the top-down view only sees the plug's upper surface (top ~24 mm); the planned fingertip line is at ~12 mm (`plots/03`). The lower part of the housing is unobserved.
- **Suspected (untested):** the fingers closed on the prongs ~15-20 mm in front of the body; the thin-part stop position (0.800) cannot distinguish blades from nothing.
- **Unknown:** actual finger positions at the grasp pose (no frame captured there); placeholder finger dimensions.
## 5. Corrective actions
Wrist-camera check after lift; capture a frame at the grasp pose; closer re-measure before descent; measure finger geometry and the width table.
## 6. Status: `UNDER_INVESTIGATION`
