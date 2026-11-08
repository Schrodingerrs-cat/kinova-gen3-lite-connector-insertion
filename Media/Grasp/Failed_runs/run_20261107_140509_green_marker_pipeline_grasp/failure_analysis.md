# Failure analysis

## 1. Expected behaviour
After the scan, `/grasp/green` runs the table RANSAC / DBSCAN / green-cluster analysis on the merged scan and sets the view target.

## 2. Observed failure
The grasp node process died when `/grasp/green` was called (grasp_node.log: `TypeError: 'NoneType' object is not callable` in the service handler, then `AttributeError: 'NoneType' object has no attribute '__name__'` in the error handler, which killed the executor). The orchestrator then waited on a dead service and was interrupted by hand (outcome INCOMPLETE).

## 3. Supporting evidence
`grasp_node.log` lines 22-60; `metrics.json` (only the scan stage recorded).

## 4. Root cause
**Verified:** `GraspNode.__init__` assigned the merged green cloud to `self.green`, which shadowed the new service method `self.green`; the service was registered with `None`. Confirmed by the traceback and by the fix below.
Secondary weakness (not fixed): the service error handler uses `fn.__name__`, so an unexpected handler type kills the whole node instead of returning an error.

## 5. Corrective action
Renamed the cloud attribute to `self.green_pts` (`ros2_iface/grasp_node.py`). The next run is `run_*_green_marker_pipeline_grasp` (see `Media/Grasp/README.md`).

## 6. Resolution status
FIX_ATTEMPTED (not yet validated by a later run).
