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
| **Analytical, CAD-dimension-constrained antipodal grasp (chosen)** | **Primary** | Housing is a rigid prism with measured dimensions. Pose from support plane + width profile; grasp is defined in the housing frame, so `T_G_housing` is known by construction and logged per pickup. Deterministic, CPU only, auditable. |

Supporting evidence from the literature: a USB pipeline with coarse connector pose reached 100% of 50 insertion trials, and grasps that pinched the cable gave wrong in-hand pose and failed insertion ([Benchmarking Off-The-Shelf Solutions to Robotic Assembly Tasks](https://arxiv.org/pdf/2103.05140)). This is why `verify_grasp` rejects a closed width wider than the housing.

## Chosen pipeline (`housing_grasp/`)
1. **ROI → cloud** (`cloud_from_depth`): back-project an expanded window so cable and support surface are visible; base frame via `T_BE @ T_EC`.
2. **Support plane**: RANSAC on points outside the ROI.
3. **Cluster**: points above the plane, connected component holding the most ROI points.
4. **Housing pose** (`housing_pose`): long axis by PCA; width profile along it; housing = widest run (≥ 0.7 × width); the cable side is the end that continues with thin points, which sets tip direction. Measured length/width/height must match config within tolerance, otherwise no grasp (no guessing).
5. **Candidates** (`plan_grasp`): top-down, jaws close across the width, TCP at several offsets inside the approved zone from the tip, both jaw flips.
6. **Filters**: opening ≤ gripper max, fingertip above the support plane, finger swept volume free of non-housing points (cable, neighbours, cradle).
7. **Output**: ranked grasps with grasp/pre-grasp/lift poses, open width, expected closed width, and `T_G_housing` for the repeatability dataset (plan P1.19).
8. **After closing**: `verify_grasp` compares gripper width to the housing width.

## Assumptions and known limits
- Housing lies flat (roll/pitch from the support plane). A tilted or standing housing is not handled.
- Needs a view where the cable is visible on exactly one end; otherwise it refuses (ambiguous tip).
- Jaw closes across the width only; IK reachability is left to the executor, which walks the ranked list.
- Not validated on real data. Synthetic tests only check geometry. Hardware is the final authority (see `insertion_task/CLAUDE.md`).

## Next experiments
1. Record D405 views of each connector in the cradle, fill `config/grasp.yaml`, run offline, and compare estimated pose to caliper/taught pose.
2. 20 hardware pickups per connector logging `T_G_housing`, closed width and slips (P1.19).
3. Optional: GraspGen + housing-zone filter as a learned baseline, only if the analytical pipeline fails on a real connector.
