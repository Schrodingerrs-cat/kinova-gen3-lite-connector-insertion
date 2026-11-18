# Figures for the successful fresh-perception USB-A pickup

All figures use perception run1794789916872077379 and motion run1794789954005073051 (2026-11-15). PNG files are high-resolution; SVG files support scaling/editing.

1. **Combined connector cloud:** Exact segmented points used for fitting, coloured by camera view.1,426 points from three freshly captured RGB-D views, transformed into robot-base coordinates without table ICP.
2. **Combined cloud with CAD:** One rigid CAD pose fitted jointly to those views. CAD includes housing, strain relief and cable stub above the panel. Grey surfaces include sides unseen by the camera.
3. **Combined full scene:** RGB-coloured board/workspace points from the same captures, cropped and depth-edge filtered for visualization. This differs from the smaller segmented target cloud used by the fitter.
4. **Camera overlays:** Projected fitted CAD samples on all three real camera images, including the sideways viewpoint. The ArUco tag was not used to determine connector pose; the coloured housing sticker located the search region.
5. **Fit metrics:** Held-out pixel median / p95 errors:1.15/2.57mm,1.25/2.90mm,0.61/3.94mm. Within2mm:84.2%,76.2%,86.7%. These measure depth-to-CAD agreement, not absolute robot/grasp accuracy. Yaw spread10.54degrees. Experimental supervised_pickup profile used70% minimum inliers,5mm maximum p95 and15degree maximum yaw spread; strict profile would reject this run.
6. **Pickup stages:** Measured tool-frame heights from controller logs. Lines connect recorded stage endpoints and do not represent continuously sampled motion. The successful physical pickup was confirmed by the user; encoder feedback alone did not verify holding.

Suggested spoken description: “We captured three nearby RGB-D views, including a sideways view, combined the segmented connector points in the robot frame, and fitted the CAD model to that cloud. We transformed CAD grasp candidates into robot coordinates, then executed a Cartesian approach, gripper closure and55mm extraction. The run completed successfully under supervision.”

Do not claim100% success rate or submillimetre absolute grasp accuracy from this single demonstration. No uncertainty bounds on absolute pose or measured contact force were established.
