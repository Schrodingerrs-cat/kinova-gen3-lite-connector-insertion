# Power plug results, 2026-11-19

Profile: `connectors/power.yaml` (scan 0.260 m above the board, +-80 mm sweep, fingertip 10 mm above the board, jaw band 12 mm, jaws open 52 mm during the descent, pre-grasp 120 mm above). The CAD part (26.5 mm square, 91 mm) does not match the real plug; the profile records what was measured.

| run | what | stage 1 | grasp | notes |
|---|---|---|---|---|
| run_20261119_220300_orbit_POW1 | scan + grasp, then just open (no put-back) | PASS (699-1210 teal points per view, centre leave-one-out 4.5 mm, tolerance 6 mm) | camera rule HELD: 0.111 m between the tips, table 0.254 m | real body: bright teal, ~15 mm along the jaw axis, ~21 mm across, 7-30 mm above the board (per-view tops 26.2-32.5 mm); cable ~10 mm leaves the top toward the arm and rises to 45-90 mm. physical hold confirmed by the operator by eye (picked up, then released by opening, no put-back) |

Not yet tested: other placements and cable arrangements, repeatability. The first mask (dark or teal pixels) swept in the cable and gave a 38-48 mm "footprint"; it was replaced by bright-teal-only points.
