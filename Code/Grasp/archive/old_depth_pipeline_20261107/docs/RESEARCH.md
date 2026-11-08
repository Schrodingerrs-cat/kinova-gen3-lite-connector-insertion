# Grasp method selection for cable connector housings

**Task:** given an ROI on a cable head (connector housing) in the eye-in-hand D405 view, produce a repeatable grasp on the rigid housing (never the cable) and a known in-hand pose for the later IBVS insertion.
**Hardware limits:** Gen3 Lite parallel gripper (max opening 85 mm in HW7 config), D405 (7–50 cm), laptop RTX 3050 with 6 GB VRAM.

## Requirements that drive the choice
1. **Repeatable, with a known in-hand pose.** Insertion needs `T_EP`. A generic grasp generator picks an arbitrary stable pose, so the plug lands in the gripper differently every time.
2. **Housing only.** The grasp must avoid cable, strain relief, latch and pins. A stability score does not know this.
3. **Small, thin parts.** USB-A / audio jack / IEC housings are roughly 6–20 mm across.
4. **Fits 6 GB VRAM** and runs offline.

## Options compared
| Option | Verdict | Why |
|---|---|---|
| Contact-GraspNet, AnyGrasp, GraspNet-1B class (learned, scene-level) | Reject as primary | Authors report small/thin objects get low-confidence contacts ([Contact-GraspNet](https://arxiv.org/pdf/2103.14127)); output is a stable grasp, not the housing grasp, so `T_EP` varies and cable grasps are possible. AnyGrasp is licence-gated (not checked here). |
| GraspGen (NVIDIA, diffusion, object-centric cloud) | Keep as optional comparison | Needs a clean object segmentation, stated dependency on depth and segmentation quality ([paper](https://arxiv.org/pdf/2507.13097)); I found no small-object evidence. Still produces arbitrary stable grasps, so it also needs a housing-zone filter. |
| FoundationPose / model-based 6D pose | Reject for now | Peak GPU about 7 GB, 8 GB recommended ([Isaac ROS](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose)); does not fit 6 GB with a segmenter. Overkill for a box-like part lying flat. |
| SAM 2 box-prompt segmentation | Optional front end | Useful if depth-only segmentation fails on dark or reflective housings. No small-object evidence found; test before adopting. |
| **Analytical, geometry-based antipodal grasp (chosen)** | **Primary** | Housing is a rigid, roughly prismatic body that is thicker than its cable. Its pose and size are measured from the depth data (support plane + width profile), so no connector dimensions are needed. The grasp is defined in the housing frame, so `T_G_housing` is known and logged per pickup. Deterministic, CPU only, auditable. |

Supporting evidence from the literature: a USB pipeline with coarse connector pose reached 100% of 50 insertion trials, and grasps that pinched the cable gave wrong in-hand pose and failed insertion ([Benchmarking Off-The-Shelf Solutions to Robotic Assembly Tasks](https://arxiv.org/pdf/2103.05140)). This is why `verify_grasp` rejects a closed width wider than the housing.

## Chosen pipeline (`housing_grasp/`)
1. **Clean cloud** (`cloud.py`): back-project the ROI window, drop flying pixels (3x3 depth jump > 3 mm), 1 mm voxel grid, remove speckle. Base frame via `T_BE @ T_EC` (HW7 hand-eye).
2. **Segment** (`segment.py`): support plane fitted on table points hugging the object (refit twice, so it does not depend on the ROI box); object = the connected component at the CENTER of the ROI, between 2 mm and 40 mm above the plane (gripper fingers hover above that).
3. **Measure** (`housing.py`): housing = the tall solid core (>= half the object's 95th-percentile height); long axis by PCA; width/height profile along it; a tall thin tail is cable (width ratio); tip vs cable end from the (low) points past each end, else the tip is flagged ambiguous and both ends are kept out.
4. **Grasp** (`grasp.py`): top-down, jaws across the measured width; candidate positions slide along the housing excluding tip and cable-end margins, ranked by flatness (constant width and top height); both jaw flips; rejected if the fingertip would be below the table or finger volume hits non-housing points.
5. **Verify**: `grasp_held()` = fingers stopped before fully closed (cannot tell a thick housing from thin prongs; vision check still to add).

## Assumptions and known limits
- Housing lies flat; roll/pitch come from the support plane. Cable visible on one side for a known tip direction (else flagged).
- A top-down view sees only the upper surface (the lower part of the housing is unobserved); dimensions are approximate.
- Not yet: multi-view scan fusion, automatic head finding without a user ROI, vision-based grasp check.

## Next experiments
1. Scan + fusion (needs views registered to well below the voxel size; hand-eye error must be measured first).
2. Analytic head finder: thick compact solid at the end of a long thin structure, ranked; markers only as a baseline condition.
3. Capture a wrist frame at the grasp pose to explain the two missed picks; measure finger pad geometry and the width table.
4. Optional learned baseline (GraspGen) only after the analytic pipeline is characterised.
