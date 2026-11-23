# Failure analysis: run_20261121_164203_test

- **Date:** 2026-11-21
- **Mode / command:** head (`--mode head --grasp`)
- **What happened:** scan reported 7 views OK, then head_grasp -> FAILED: Unknown projection '3d'
- **Root cause:** TWO causes. (1) The 7-pose scan saved 7 views with an identical camera pose (spread 0.000 m) and identical depth although they were captured 4-6 s apart; trajectory.csv shows every joint frozen from ~8 s on. Scan pose 1 (joints 18.2, 29.4, 161.1, ...) has joint 3 = 161.1 deg, outside the +-149.5 deg limit; the arm stopped short (fault suspected, not read) and the old move code never checked that the arm arrived. The colour target (248, 21, 5 mm) therefore came from 7 copies of one pose. (2) The 3d projection was not registered: pip matplotlib 3.10.9 shadows the system 3.5.1 and the mpl_toolkits workaround ran after another module had already imported pyplot (import-order bug introduced with the new cad_report module).
- **Status / fix:** (1) Arm safety checks added (fault, joint limit +- margin, arrival within 2 deg; scan poses outside the limits are skipped). Pose 1 still needs re-recording. (2) Workaround moved into housing_grasp/__init__.py + tests/test_plot_env.py.
- **Evidence:** `controller.log`, `metrics.json`, `trajectory.csv` in this folder.
- **Not a success:** no grasp was held in this run.
