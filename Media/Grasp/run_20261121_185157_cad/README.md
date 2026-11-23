# run_20261121_185157_cad: CAD perception only (usb_a)

`--mode cad --connector usb_a`, no `--grasp`. Not a failure of the pipeline, but not a good fit either.

- Outcome: PERCEPTION_ONLY. 3 close views and a CAD fit; no grasp requested.
- Fit quality (plan folder): held-out points within 2 mm per view [31, 0, 0] %, mean p80 10.4 mm, implied seat -18.2 mm (model 0.0). The gate failed (not blocking since 2026-11-21); the fit is NOT trustworthy.
- Cause of the poor fit: the 3 close views disagree by 7-12 mm in the base frame (see `../summary_plots/`).
- Plots: `plots/plan_01_cad_overlay_views.png`, `plan_02_cad_fit_side_views.png`, `plan_03_cad_heldout.png`.
