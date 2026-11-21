# Failure analysis
- Error: `no measured gripper reference matches: measured gripper depth no longer matches its calibration` (all three references).
- Measured cause: same camera and gripper (K, distortion, frame id and gripper opening all matched; finger depth 9.9 cm vs 9.6 cm reference), but only 13-23 % of finger pixels matched the stored colour appearance, i.e. lighting/white balance differed from when the references were captured.
- Classification: tuning/calibration issue, not a controller bug. The guard worked as designed.
- Remedy: remeasured the reference from 10 stationary frames (support 0.998 on its own frame, 0.91-0.98 on frames from other arm poses); old references archived in `Code/IBVS/config/archive_gripper_depth_20261121/`.
