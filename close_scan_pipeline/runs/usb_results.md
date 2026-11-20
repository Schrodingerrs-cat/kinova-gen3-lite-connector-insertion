# USB plug results, 2026-11-19 (close scan + grasp)

Layout for all runs: placement P1, cable A (the setup as it was). Pipeline version: `snapshots/usb_pipeline_20261119` (G2 version). Each run folder has `README.md`, `metrics.json`, `plots/`, and for grasps a `grasp/` folder with the wrist-camera images and `grasp.json`.

| run | what | stage 1 | grasp | notes |
|---|---|---|---|---|
| run_20261119_210108_orbit_o1 | first close scan, camera 0.228 m above the board | PASS | none | wide-face yaw 93.3 / 94.1 deg, leave-one-out 2.3 mm |
| run_20261119_211805_orbit_P1_A | trial 1 of 9 (scan only) | PASS | none | wide-face yaw 92.3 / 92.6 deg, leave-one-out 2.8 mm |
| run_20261119_212527_orbit_G1 | scan + first grasp | PASS | closed on the plug, then the script released it | operator saw the jaws close on the plug; old script judged "not held" from the finger reading and opened before lifting. Bug, fixed. Not a missed grasp |
| run_20261119_212938_orbit_G2 | scan + grasp with the camera deciding | PASS | picked: camera rule HELD (0.109 m between the tips, table 0.256 m) | operator confirmed by eye; plug then put back with `tools/put_back.py` |

Counts: stage 1 passed 4 of 4 scans (same layout, so this is not the 9-trial test). Grasp: 1 confirmed pick out of 1 attempt since the hold logic was fixed. Not yet tested: other placements and cable arrangements, repeatability statistics.
Stage 1 log: `stage1_close_trials.csv`.
