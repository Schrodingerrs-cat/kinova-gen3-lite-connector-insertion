# Failure analysis: run_20261121_165125 (several runs in one folder)

**This folder mixes at least three runs.** One `$RUN` path was reused for several commands, so their outputs overwrote and accumulated into the same folder. The files were not separated.

| content | belongs to | outcome |
|---|---|---|
| `01_usb_a/`, `02_micro_jack/` (+ their `plan/`, `cad_views/`) | cad runs, `--mode cad --connector usb_a micro_jack` (~16:54-16:55) | CAD fit rejected by the gate each time (held-out within 2 mm: usb_a [73, 55, 62] %, micro_jack [32, 0, 0] %; implied seat -5.2 / -3.0 mm; leave-one-out pose differs 7.2 mm). No grasp planned. |
| top-level `controller.log`, `metrics.json`, `scan/`, `screenshots/`, `plots/` | head run, `--mode head` | scan OK (6 views, pose 1 skipped), color_target -> (249, 23, -1) mm, view OK, head_grasp -> FAILED: only 6 marker points in the close view |
| `summary.json` | a later strip run, `--mode strip --connector usb_a aux` | strip_scan failed (no sticker seen from home); `aux` not run |

## Root causes
- **cad fits:** the 3 close views disagree by 7-12 mm in the base frame (a rigid per-view camera-pose error, measured; not fixed by the alternative hand-eye values tried), so the multi-view CAD fit cannot agree with itself. The second run also fitted the micro_jack model to the USB's sticker (no way to tell stickers apart at that time).
- **head run:** the coarse target was the aux plug's small sticker (6 x 7 mm), not the USB's; head mode takes the largest merged marker cluster. A 6 x 7 mm sticker gives 6 marker points in the close view (below `color_marker.min_marker_points`).
- **strip run:** stickers are ~10 px from home; the USB sticker keeps 6 depth points after the depth-edge filter (needs 15), the aux sticker splits into 18 px pieces (needs 30).

## Status
Open for cad (view-to-view disagreement) and strip (small stickers from home). head / cloud now take `--connector` (sticker at the connector's predefined spot); not re-run.
