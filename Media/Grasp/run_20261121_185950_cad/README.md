# run_20261121_185950_cad: CAD grasp executed (usb_a), hold NOT independently verified

`--mode cad --connector usb_a --grasp`. The only run today in which the arm grasped.

- Outcome (pipeline): GRASP_REPORTED_HELD.
- The two hold signals DISAGREE: the finger-position check says no (fingers closed to ~0.4 mm, i.e. almost shut); the wrist camera says yes (depth between the tips 0.136 m vs table 0.247 m). A USB plug is ~4.5 mm thick, so a 0.4 mm gap suggests the fingers closed on something much thinner than the plug, or on nothing.
- No independent confirmation (video check by an operator is still needed). Do NOT count this as a successful grasp.
- The CAD fit used for this grasp was poor: held-out within 2 mm [42, 0, 0] %, mean p80 9.5 mm, implied seat -18.7 mm (model 0.0). The grasp point is therefore doubtful.
- Plots: `plots/plan_01_cad_overlay_views.png`, `plan_02_cad_fit_side_views.png`, `plan_03_cad_heldout.png`; screenshots in `screenshots/` (see `04_after_grasp_plan.png`, `05_after_grasp.png`), video in `videos/`.
