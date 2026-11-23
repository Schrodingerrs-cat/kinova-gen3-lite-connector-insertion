# Failure analysis: run_20261121_185341_cad

- **Date:** 2026-11-21
- **Mode / command:** cad (`--mode cad --connector usb_a --grasp`)
- **What happened:** cad_scan OK, cad_grasp OK (gate off), allow_grasp OK, execute -> FAILED: no candidate passed the motion check (all 120: pre-grasp: no near-current IK)
- **Root cause:** grasp.max_joint_dev_deg was 120. The best candidate's pre-grasp needs joint 6 to turn 158 deg from the arm's pose (flip +1; flip -1 has no IK), so every candidate was refused. Checked read-only against MoveIt afterwards. The CAD fit itself was also poor (held-out within 2 mm [35, 0, 0] %, implied seat -15 mm), so the grasp point is doubtful.
- **Status / fix:** grasp.max_joint_dev_deg set to 360 (no cap); the full pick plan then passed read-only for flip +1. Joint-limit checks still apply.
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
