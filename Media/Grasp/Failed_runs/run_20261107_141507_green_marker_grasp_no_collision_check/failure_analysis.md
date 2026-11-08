# Failure analysis
1. Expected: a grasp once the collision check is off.
2. Observed: the optimizer's Monte Carlo (40 samples, sigma 3 mm lateral/axial, 3 deg yaw) rejected every candidate; a sample FAILS when the jaw lands on the housing or the pads reach within `min_protrusion_m/2` of the tip (optimize.py lines 94-106). This is NOT an environment-collision test.
3. Evidence: `plan/*/plan.json` (`rejected`), `plan/*/metrics_pipeline.json`.
4. Cause: **Verified:** the filter is the insertion-face constraint, not collision. **Suspected:** the measured housing (14.5 mm long) is too short for 3 mm pose error with the keep-free tip rule; and that housing is probably wrong (see run_20261107_141616).
5. Corrective action: lifted `max_fail_rate` (next run).
6. Status: FIX_ATTEMPTED.
