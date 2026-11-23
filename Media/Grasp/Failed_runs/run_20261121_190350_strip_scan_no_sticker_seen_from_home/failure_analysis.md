# Failure analysis: run_20261121_190350_strip

- **Date:** 2026-11-21
- **Mode / command:** strip (`--mode strip --connector usb_a --grasp`)
- **What happened:** strip_scan -> FAILED: no sticker seen from home
- **Root cause:** From home (camera 344 mm up, 386 mm to the crib) the stickers are tiny. On a saved home frame: the USB sticker is 40 px but only 6 depth points survive the depth-edge filter (needs 15); the aux sticker splits into three 18 px pieces (needs 30 px each). cad mode avoids this by falling back to the 7-pose scan; strip mode has no fallback.
- **Status / fix:** Open. A direct fix (small-blob detection) was written, then reverted at the user's request; strip mode needs either that or the cad-style 7-pose fallback.
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
