# Failure analysis
- Error: `observed-scene collision or invalid state: left_finger_dist_link / right_finger_dist_link vs ibvs_observed_..._positioning`.
- Measured cause (`positioning_scene.npz`): 6 obstacle voxels at 11.3-12.4 cm in front of the camera, pixels u 431-510, v 407-447, i.e. the closed fingertips / plug tip. The positioning scene voxelized everything beyond a hard-coded 11 cm. With the open gripper the fingers were inside 10 cm so this never triggered; with a plug the tip reaches about 12 cm, and about 1500 edge pixels were not in the stable self-depth reference (template 2019 px vs 3525 raw near pixels), so they became obstacles at the finger links.
- Classification: implementation bug (inconsistent near-range rule), not a hardware or perception limit. The servo scene already used max(18 cm, fingertip distance + 4 cm).
- Fix: both scenes use `ibvs.depth.scene_near_depth` (0.18 m with the current hand-eye). Unit-tested; not yet re-run on the arm.
