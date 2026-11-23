# Failure analysis: run_20261121_185127_cad

- **Date:** 2026-11-21
- **Mode / command:** cad (`--mode cad --connector usb_a`)
- **What happened:** cad_scan -> FAILED: no sticker within 30 mm of usb_a's spot [252, -23] mm; seen at [[309, -209], [305, -226], [253, 20]]
- **Root cause:** From home the USB sticker was not detected at all (only the aux sticker at (253, 20) and two blobs far to the side), so the predefined-spot rule had nothing near the USB spot. From home the stickers are ~10 px; the USB sticker keeps only ~6 depth points after the depth-edge filter. Intermittent: the next cad run (185157) found a sticker.
- **Status / fix:** Open. cad mode works when it falls back to the 7-pose scan; detecting the small stickers directly from home was prepared and then reverted at the user's request.
- **Evidence:** `controller.log`, `metrics.json` in this folder.
- **Not a success:** no grasp was held in this run.
