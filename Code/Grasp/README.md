# Grasp synthesis (Task A)

ROI on a cable head → grasp on the rigid connector housing. Method choice and evidence: [`docs/RESEARCH.md`](docs/RESEARCH.md).

```python
from housing_grasp import cloud_from_depth, plan_grasp, verify_grasp
from housing_grasp.config import load_config

cfg = load_config("config/grasp.yaml")          # refuses to run while any value is null
pts_B, in_roi = cloud_from_depth(depth_m, K, T_BC, (u0, v0, u1, v1))
res = plan_grasp(pts_B, in_roi, cam_origin_B, cfg)
# res["ok"], res["reasons"], res["T_BH"], res["grasps"] (ranked: T_BG, T_pre, T_lift, open_width_m, T_G_housing)
ok, why = verify_grasp(closed_width_m, cfg["housing"], cfg["gripper"]["close_tol_m"])
```

Pure NumPy/SciPy, no ROS. The executor (separate) tries `res["grasps"]` in order with MoveIt IK, closes, then calls `verify_grasp`.

Test: `python3 -m pytest tests -q` (synthetic scenes; geometry only, not hardware evidence).

Status: implemented, synthetic tests pass, **not validated on the real arm or real D405 data**. `config/grasp.yaml` still has unmeasured values.
