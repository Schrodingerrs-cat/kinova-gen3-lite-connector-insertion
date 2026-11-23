# Failure analysis: run_20261121_185742_cad

- **Date:** 2026-11-21
- **Mode / command:** cad (`--mode cad --connector usb_a --grasp`)
- **What happened:** same as 185341: execute -> FAILED: no candidate passed the motion check (all 120)
- **Root cause:** Same cause as run 185341 (joint 6 needs ~158 deg, cap 120). CAD fit held-out within 2 mm [33, 0, 0] %, implied seat -15.3 mm.
- **Status / fix:** Same fix: cap removed.
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
