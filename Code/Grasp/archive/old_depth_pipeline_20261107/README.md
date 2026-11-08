# Archive: the old depth-based grasp pipeline (snapshot 2026-11-07)

Archived, not deleted, when the green-sticker pipeline became the main one. Not used by the live pipeline; kept for reference and comparison.

What this snapshot contains (state just before removal, so it still holds BOTH the old depth planner and the first green code):
- `housing_grasp/`: `housing.py` (dimension-free housing pose from depth), `segment.py` (support plane + solid object), `heads.py` (head locator on the fused scan), `grasp.py` (antipodal candidates + collision filter + `plan_grasp`), `optimize.py` (the grasp optimizer: weighted Q, Monte Carlo pose-error robustness), `report.py` (plots), plus the shared modules.
- `ros2_iface/grasp_node.py` (node with the services `find`, `plan`, `green_plan` and the ROI flow), `ros2_iface/roi_picker.py`, `ros2_iface/run_green_pipeline.py` (with `--depth-plan` and the gates).
- `config/grasp.yaml` (with the `housing`, `optimizer`, `heads` blocks), `tests/` (the old planner's tests), `docs/RESEARCH.md`, `docs/Strategy.md`, `README.md`.

Why it was replaced: on the black connector head the depth-measured housing was unstable (6-14 mm long, 39 mm tall for a head that looks about 18 x 24 mm); see `Media/Grasp/Failed_runs/run_20261107_140835*` and `run_20261107_141616*`.

Run its tests: `python3 -m pytest archive/old_depth_pipeline_20261107/tests -q` (from `Code/Grasp`). To use it again, copy the files back over `housing_grasp/`, `ros2_iface/`, `config/` of `Code/Grasp`.
