# Failure analysis: run_20261121_190708_cad

- **Date:** 2026-11-21
- **Mode / command:** cad (`--mode cad --connector micro_jack --grasp`)
- **What happened:** cad_grasp OK (gate off), execute -> FAILED: grasp NOT held: fingers closed on nothing and the wrist camera sees nothing between the tips
- **Root cause:** The arm reached the pre-grasp and grasped, but the fit was bad (held-out within 2 mm [1, 0, 0] %, p80 11.4 mm, implied seat -5.2 mm vs model 2.1), so the grasp point missed the plug. With the gate off the bad fit was used. Whether the micro_jack STEP model describes the jack on the panel is unconfirmed.
- **Status / fix:** Open: view-to-view disagreement of 7-12 mm breaks the multi-view CAD fit (measured; see summary_plots/).
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
