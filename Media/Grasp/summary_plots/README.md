# Summary plots, cad runs of 2026-11-21

Re-made by `python3 Media/Grasp/summary_plots/make_plots.py` (after `source Code/Grasp/ros_env.sh`). Data: the `plan.json` of every cad fit and the saved close views in `work/cad/`.

| plot | what it shows |
|---|---|
| `01_cad_fit_heldout_accuracy.png` | Per CAD fit, the share of held-out points within 2 mm for each of the 3 views. The two first fits (16:53, 16:54, usb_a) reach 55-85 %; every later fit is 0-42 %, and 0 % means the views do not agree with each other. |
| `02_view_to_view_sticker_offsets.png` | The same sticker as seen from each of the 3 close views, as an offset from that run's mean. Each view lands in a different place (x/y spread 12-22 mm), and a given view lands in a similar place in most runs: a per-view camera-pose error, not noise. |

Per-run plots (CAD overlay on the camera views, side views, held-out accuracy) are in each run's `plots/` and `plan/<time>/plots/`:
- kept: `run_20261121_185157_cad` (perception only), `run_20261121_185950_cad` (grasp executed, hold unverified)
- failed runs with their analysis: `Failed_runs/run_20261121_*`

Failed runs of 2026-11-21 are in `Failed_runs/`, each with `failure_analysis.md`. One folder there (`..._MIXED_...`) holds several runs written to the same output path; its analysis says which file belongs to which.
