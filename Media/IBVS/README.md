# IBVS run output

IBVS runs save their configuration, measured features, motion logs, summary, and recordings in this directory. The runners create timestamped `shape_ibvs_*` or `run_*_free_space` subfolders.

See [Code/IBVS/README.md](../../Code/IBVS/README.md) for setup and testing commands.

## 2026-11-27: insertion servo on the real arm (`run_*_insert_<wire>/`)

Every `run_insert.sh` run is recorded here (config.json = what ran, controller.log, trajectory.csv, metrics.json, video, plots). 40 folders today; the folders without `metrics.json` and without a wire are runs that ended before the servo started (arm refused or driver down) or were killed by hand. Failure analyses are in the table below, not yet filed under `Failed_runs/`.

| Time | Wire | What happened |
| --- | --- | --- |
| 15:23 | aux | stepwise servo converged at the entrance (marker 103 px), first working hardware run |
| 15:28 - 16:21 | aux | continuous mode: stalled, vibrated in place (new trajectory every frame), then "marker lost" (depth stream missing); fixed: replan only every half horizon, read the colour topics directly |
| 16:33 - 17:40 | aux, usb, power | continuous mode reaches the entrance within a few mm, aux and usb repeatedly; stops "blocked" when the plug rests on the plate |
| 16:35, 17:10, 18:06 | usb | stopped by the contact check 3 mm short of the goal: plug on the socket plate; later counted as arriving at the entrance |
| 16:47 | power | false "blocked" stop at the start of the insertion (ramp-up from rest); fixed |
| 17:40 | power | goal shifted +37.7 px by a mis-detected fingertip pair; arm advanced 100 mm while the marker barely grew: the board gave way |
| 18:01 | aux | seated depth taught with a person in the loop: marker 104.0 px, 5.0 mm past the entrance |
| 18:10 | usb | seated depth taught: marker 102.3 px, 5.5 mm past the entrance |
| 18:14 | power | plug touched the board at 89 px (goal 95); tool advanced 35 mm with the marker flat, marker lost, driver crashed: the plug is held differently from when its goal was taught |
| 18:19 | power | seated judged by the person at 7 to 16 mm past the entrance, run killed before it was recorded: not taught |

Not a verified insertion at any point: there is no seating sensor, and the push-in stage (`--push-mm`, taught depth) has not run on the arm yet. See Code/IBVS/README.md, "State of the insertion servo".
