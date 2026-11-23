# Failure analysis: run_20261121_190409_strip

- **Date:** 2026-11-21
- **Mode / command:** strip (`--mode strip --connector usb_a --grasp`)
- **What happened:** strip_scan -> FAILED: no sticker seen from home
- **Root cause:** Same as run 190350.
- **Status / fix:** Open, as run 190350.
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
