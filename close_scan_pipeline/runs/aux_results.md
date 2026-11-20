# Aux (6.35 mm jack) results, 2026-11-19

Profile: `connectors/aux.yaml` (scan 0.200 m above the board, +-50 mm sweep, fingertip 6 mm above the board, jaw band 12 mm, jaws open 31.5 mm during the descent). Each run folder has `README.md`, `metrics.json`, `plots/`, and `grasp/`.

| run | what | stage 1 | grasp | notes |
|---|---|---|---|---|
| run_20261119_214552_orbit_AUX1 | scan + grasp | PASS (220-432 plug points per view, centre leave-one-out 2.2 mm) | camera rule HELD: 0.114 m between the tips, table 0.251 m | barrel measured 2-8 mm to 23.7 mm above the board (per-view tops 23.4-26.2 mm), cable joins at 27-34 mm. Physical hold: operator to confirm |

Measured, not assumed: board top -0.0125 m, barrel top 0.0112 m; grasp tcp z -0.0065 m; top margin set to 5 mm from the measured 2.8 mm spread between views.
Not yet tested: other placements and cable arrangements, repeatability.
