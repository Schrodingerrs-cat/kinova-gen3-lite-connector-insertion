# Failure analysis: run_20261121_185036_strip

- **Date:** 2026-11-21
- **Mode / command:** strip (`--mode strip --connector usb_a`)
- **What happened:** strip_scan -> FAILED: white strip not seen complete from home (size 150 x 51 mm, expected [150, 51])
- **Root cause:** The strip was found (size shown is the nominal one) but the 'complete' test failed; the message hid the real fill / score. A later probe on a saved home frame gave fill 0.75 / score 0.70, which passes the current thresholds (min_fill 0.65, min_score 0.55), so this was an earlier detector / threshold state. Not reproduced.
- **Status / fix:** Detector now searches around the stickers and min_fill is 0.65 (the crib has holes). Unconfirmed on the arm.
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
