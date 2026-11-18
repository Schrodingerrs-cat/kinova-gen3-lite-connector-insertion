# CAD grasp workflows

The latest demonstrated workflow is the three-view batch pipeline described in [CAD_MULTIVIEW.md](CAD_MULTIVIEW.md). The service workflow below is a separate, stricter experimental path. Its individual-view consistency gates should not be confused with the batch joint fit.

## Experimental service workflow (hardware validation pending)

The new `/grasp/cad_capture` and `/grasp/cad_grasp` services use measured CAD registration, without the ArUco map. They are separate from `/grasp/model_grasp`, which retains its tag-based behavior. Nothing automatically starts moving during a fit.

## What it does

`cad_capture` checks camera freshness (<=1 second), initial colour/depth stamp separation (<=100 ms), and that joints remain within0.2 degrees during median-depth capture. It saves a fresh NPZ and runs the project-venv offline fitter with six DOF and above-panel geometry. Logs, overlays and transforms are saved in `Media/Grasp/work/cad_live/<time_ns>/`. A failed capture/fit clears the collected CAD views and any previous grasp plan. Colour/depth are not exact-time synchronized; median depth uses multiple frames, so the arm and scene must remain stationary.

`cad_grasp` requires three recent views (<=120 seconds old), with each pair separated by20 mm camera translation OR10 degrees rotation. Each fit must meet the provisional residual screen: p95<=5 mm and >=80% within2 mm. All three CAD origins must agree within3 mm and orientations within5 degrees (allowing housing 180-degree yaw symmetry); fitted tilt must be<=20 degrees. These are engineering rejection thresholds, not established accuracy guarantees. The latest pose is used only after agreement, not averaged to conceal disagreement.

Precomputed CAD candidates are transformed by the fitted full rigid pose, including tilt. The existing grasp node checks workspace/width, IK and Cartesian paths, then executes through its existing gates. CAD captures expire and actual motion plans are single-use. There is no new controller implementation.

## Start and collect (when connected again)

From `Code/Grasp`, start the usual stack:

```bash
./run_ros.sh dry_run:=true rviz:=false
```

In a separate terminal in the same directory:

```bash
source ros_env.sh
ros2 param set /grasp_node dry_run true
ros2 param set /grasp_node allow_grasp false
ros2 param set /grasp_node pregrasp_only true
ros2 service call /grasp/cad_capture std_srvs/srv/Trigger "{}"
```

Inspect that capture's overlay and segmentation. Move to a second and third previously checked camera view using the existing supervised motion workflow, stop, and call `cad_capture` once at each. This service does not move between views. The connector and board must remain fixed. Do not collect three frames from the same pose. If a view fails, collect a new complete set; do not edit metrics or thresholds to force acceptance.

```bash
ros2 service call /grasp/cad_grasp std_srvs/srv/Trigger "{}"
ros2 service call /grasp/execute std_srvs/srv/Trigger "{}"
```

Only proceed if BOTH calls report success and the latter says NOT EXECUTED / motion checked only. Stop on a rejected fit. The saved Nov15 comparison views deliberately fail the inter-view position gate.

## First supervised physical test: pregrasp only

After reviewing a fresh accepted CAD plan and the route/cable clearance:

```bash
ros2 param set /grasp_node pregrasp_only true
ros2 service call /grasp/allow_grasp std_srvs/srv/Trigger "{}"
ros2 param set /grasp_node dry_run false
ros2 service call /grasp/execute std_srvs/srv/Trigger "{}"
ros2 param set /grasp_node dry_run true
ros2 param set /grasp_node allow_grasp false
```

Expect PREGRASP ONLY, gripper open, no descent/closure/lift. The plan is consumed. Inspect actual alignment. Before a later complete pickup, create another fresh accepted CAD plan, dry-run it, and use the same execution gates with `pregrasp_only=false`; that path descends, closes and lifts via the existing implementation. Do not run it immediately after a rejected plan or on a setup already holding a connector. No automatic release/insertion is added.

## Known limits

This is implemented and offline-tested, not hardware-validated. Existing Cartesian planning uses `avoid_collisions=False`; IK/path success does not establish whole-arm, finger, board or cable clearance. CAD fitting does not currently evaluate swept gripper collisions against the scene, and grasp ranking is precomputed CAD geometry rather than a new scene-aware grasp search. Existing camera-based held checks previously produced a false positive. Physical confirmation remains necessary. No force-controlled extraction is added.

The current saved clouds fit individual views well but disagree by several millimetres between views. These services intentionally reject that condition. Fixing capture/calibration consistency is still required before the first CAD-driven motion trial.

## Offline checks

```bash
../../.venv/bin/python tools/test_cad_execution.py
MPLCONFIGDIR=/tmp/mpl ../../.venv/bin/python tools/test_fit_cad_cloud.py
python3 -m py_compile ros2_iface/grasp_node.py housing_grasp/cad_execution.py
```
