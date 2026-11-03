# Task B — High-Fidelity Isaac Sim / Isaac Lab Reinforcement Learning for Connector Insertion

**Project:** Vision-Based Manipulation | **Status:** implementation specification v1.0 | **Updated:** 2026-11-02  
**Target:** Train a visual-feature-conditioned insertion policy in a **measured digital twin** of the Kinova Gen3 Lite + rigid housing plug + fixed socket + eye-in-hand camera, evaluate under plausible variations, and transfer safely to the physical robot.  
**Shared prerequisites:** [`SHARED_PRIMITIVES_CHECKLIST.md`](SHARED_PRIMITIVES_CHECKLIST.md). **Classical baseline / demonstration provider:** [`Task_A_IBVS_Search_PLAN.md`](Task_A_IBVS_Search_PLAN.md).

> **Frozen scientific goal:** Learn robust fine alignment and guarded insertion **without requiring a learned grasp policy**. Simulation fidelity, measurable physical verification, reproducible perturbations and deployment-observable inputs are requirements; the camera bracket, connector and fixture can change if accompanied by a versioned remeasurement and simulator update.

## 1. Firm deliverable / scope

**Deliverable B:** A version-controlled Isaac Sim + Isaac Lab environment with a calibrated Gen3 Lite digital twin and EIH camera, parallelized learning, an exported insertion policy, and a standardized evaluation package. **Full target deliverable additionally includes physical replay/deployment on the Gen3 Lite with independent seating verification and matched trials against Task A.**

**Required minimum acceptance:**

- [ ] Import and validate **the actual six-joint Gen3 Lite** URDF/Xacro and two-finger gripper (not a 7-DoF Gen3 or generic Franka). Save a versioned USD, joint mapping, actuator settings, end-effector frame and geometry proof.
- [ ] Reconstruct real socket entry, keyed clearance/chamfers, plug housing and gripper attachment from measurements/CAD. Collision representation **preserves the actual socket cavity** (a convex hull that fills the hole is invalid).
- [ ] Mount simulated EIH camera at **measured** `T_EC`, intrinsics, FOV, resolution, observation rate and calibrated socket view. Record real-vs-sim registration images, marker tracks, and depth-validity comparison.
- [ ] Simulate guarded contact and success using **geometric engagement + insertion depth + final pose tolerance + stable dwell / safe contact**; logging a commanded displacement is not a valid simulator success criterion.
- [ ] Provide a Gymnasium-style/Isaac Lab environment with documented reset, observations, action scaling, reward, termination, event randomization, safety filters, and deterministic evaluation seeds.
- [ ] Train at least **one standalone fine-insertion policy** from deployable visual features and proprioception; compare to a scripted/Task A baseline on the **same held-out simulated test set**. Archive checkpoints, learning curves, failure traces, and test summary.
- [ ] Run at least **one demonstration-boosted or residual policy** against the same test conditions, unless the benchmark reveals no safely learnable insertion feedback; record such a negative finding explicitly.
- [ ] Demonstrate **actual parallel environments**, publish the measured `num_envs`, steps/s, GPU memory use, camera rate and training wall time. Parallel physics ≠ unlimited parallel high-resolution EIH RGB-D rendering.
- [ ] Export a policy whose *actor* takes **only signals that can be produced on the real robot**, validate observations/actions via an identical adapter in real and sim, and run offline safety/replay checks before any real execution.
- [ ] For the **full physical deliverable**: supervised real trials at verified safe action limits with an independent seating sensor. If the physical gate cannot be cleared, label completion `SIM_VALIDATED_ONLY`; never call it successful sim-to-real deployment.

**Proposed performance targets (not promises):** achieve ≥80% independently verified simulated success across **at least 100 held-out episodes** at a pre-registered nominal domain, and ≥70% across **at least 100 held-out realistic perturbation episodes**; attempt improvement over Task A on matched cases. Real-world target: ≥20 supervised nominal episodes and ≥20 validated safe perturbation episodes, all with per-attempt traces and no omitted failures. These figures are aspirational decision thresholds rather than facts about this hardware. A failed target still yields a research result but cannot be represented as controller success.

**Out of scope:** learning arbitrary cable grasping/routing, claiming exact deformable cable physics without measured data, actor access to ideal simulator socket pose/contact forces if those cannot be observed on the real robot, using RL output as an unlimited force/torque command, or unsafe autonomous online exploration on hardware.

## 2. Simulation stack decision and real compute constraints

**Proposed implementation:** NVIDIA Isaac Sim for rendering/physics and robot USD; Isaac Lab for vectorized task management, observation/action/reward/termination configuration and RL training. **Pin a compatible release set at project start.** NVIDIA's current Isaac Lab 3.x docs describe Isaac Sim 6.x as an optional integration; use the versions you actually install rather than copying commands across generations.

**Critical hardware gate:** high-fidelity RTX-rendered Isaac Sim is **not** a CPU-only solution. Current NVIDIA docs list workstation-class hardware around **32 GB system RAM and 16 GB VRAM minimum**, with supported RTX GPU/driver (exact requirements depend on version); Isaac Lab camera workloads may need more. A prior ~15 GB RAM/no-GPU Ubuntu machine is inadequate for this plan unless replaced by an appropriate local/remote workstation. Run NVIDIA's compatibility checker **before** budgeting training. Some compute GPUs without RTX cores are not valid substitutes for Isaac Sim RTX rendering.

**Parallelism plan:**

1. Start **one physics-backed scene** for contact correctness, a single EIH render and instrumented validation.
2. Clone to **8→16→32→64** environments **only if** simulator stability, GPU memory, and steps/s support each increase. These are benchmarking steps, *not promised achievable counts*. Run 2–5 minute benchmarks at each tier and capture GPU memory, CPU/RAM use, speed and camera throughput.
3. Physics-only or privileged-teacher training can scale to larger `num_envs` after profiling; rich RGB-D camera rendering should use **tiled camera rendering**, controlled resolution, outputs and update rate, and should only be scaled after verifying accuracy and memory use.
4. Final student policy must consume simulated camera-derived observations from the **same EIH viewpoint** used on the real robot. A thousand environment state-based success cannot substitute for a deployable vision policy.
5. If GPU insufficient: use an adequate RTX GPU workstation/cloud instance, reduce image resolution/frame rate/env count, or train a compact marker-feature actor while retaining a smaller image-in-the-loop validation dataset. Record the compromise honestly.

**Candidate learning libraries:** RSL-RL PPO is the maintained Isaac Lab starting point for GPU-vectorized tasks and teacher/student distillation. RL-Games supplies PPO and SAC integrations; prefer **PPO for first successful parallel policy**, then consider **SAC** if sparse-reward sample efficiency or demonstration replay makes an off-policy method preferable. Do not assume every algorithm/config works with every custom camera task; run an integration smoke test.

## 3. Digital-twin fidelity requirements: match the real problem, not just the visuals

### Fidelity ledger (each parameter has source + confidence)

| Layer | Required match to hardware | How to validate | If missing |
|---|---|---|---|
| Robot geometry | Gen3 Lite links, exactly 6 actuated arm joints, joint axis order, base height, tool flange, hard limits | FK comparison and controlled joint poses vs live TF | **Block transfer** |
| Actuation | joint control mode, position/velocity response, acceleration, saturation, communication delay, control rate | non-contact command/response traces | Start nominal but report transfer gap; constrain actions |
| Gripper | right two-finger geometry, jaw gap, opposing housing contacts, friction, commanded/actual closure | repeat pickup and slip tests | Use fixed rigid attachment for phase-1 insertion *and flag simplification* |
| Plug | exact keyed cross-section, size, chamfers, tip frame, mass/inertia, contact material | calipers/CAD, photos, mass estimate/measurement | Cannot claim tight insertion realism |
| Socket | interior cavity, lead-in chamfer, depth, entry plane, fixed placement, mounting compliance | CAD/cross-section, tolerance measurements | **Block contact validity** |
| Cable | exit direction, strain relief, approximate added drag/torque, collision envelope | controlled supported motion / cable hang observations | Model as disturbance + randomized wrench only if justified; no deformable claim |
| Camera | actual camera model, `T_EC`, RGB intrinsics/distortion, optical frame, range, resolution/FPS | real camera calibration + overlays | **Block vision transfer** |
| RGB appearance | marker location/colors, illumination changes, shadows, reflections, background | compare real/sim frames + detector error | Use marker coordinates with measured noise until rendered RGB validated |
| Depth | D405 near/far range, spatial invalidity, aligned RGB-depth, dropout/outliers | collect multiple real depth frames at standoffs | Perfect depth is disallowed for transfer claim |
| Timing | camera/robot clocks, policy Hz, sensor delay, motion latency, timeout | stamped real command/observation logs | Add bounded delay randomization only after estimating |
| Contact | friction, restitution, frictional wedging, small-clearance behavior, solver settings | repeat bench pushes with independent sensor if possible | Treat simulation contact predictions as unvalidated |
| Reward truth | real seating sensor equivalent and simulator geometric engagement | partially inserted/jammed/full insertion test cases | No training against depth-only shortcut |

**Minimum realistic digital twin:** measured robot/connector/socket geometry, correct EIH optical viewpoint and feature appearance, non-penetrating stable contact, independently defined simulated seating, and measured uncertainty in grasp/camera/control. Full cable FEM and photorealistic material shaders are **not** prerequisites if the task concerns the rigid head and the policy consumes calibrated marker features.

### Contact and mesh construction (highest engineering risk)

- Obtain STEP/CAD, dimensional drawing, 3D scan or measured model for **the specific connector and socket**. Save part/manufacturer/revision; a generic USB-shaped mesh is not acceptable for claiming millimeter-clearance transfer.
- Keep high-quality **visual meshes** separate from physics **collision meshes**. Inspect the cavity in a collision-debug view: do not accidentally use a convex plug/socket hull that seals the opening.
- For moving plug, use a valid collision proxy preserving keyed shape/lead-in; for fixed socket use an appropriate supported concave/static collision construction or verified decomposition. Test the contact representation with the chosen PhysX solver/backend and version; not every backend/contact mode supports identical geometry.
- Calibrate contact offset, solver iteration/substeps, timestep, friction and drive stiffness at small clearance. Check that the plug cannot 'succeed' through interpenetration or numerical tunneling.
- Test three adversarial scenes: intentionally misaligned plug (must fail/stop), correctly aligned plug (should seat with reasonable motion), plug colliding with blocked cavity (must not tunnel). Record render, collider debug, penetration and contact logs.
- Realistic compliance: optional fixture flex/jaw slip can be introduced *after* rigid contact model is stable and measured. Do not fabricate force/torque data if no sensor is installed.

### Camera realism (critical for EIH)

- Create **a camera primitive parented to the same tool/rigid link as real bracket**, using `T_EC` measured from hand-eye calibration. Verify axes visually; ROS optical frames and USD camera frames may differ.
- Match real intrinsics `fx,fy,cx,cy`, principal point, distortion handling, image dimensions, RGB/depth stream orientation, clipping planes, capture rate and exposure behavior as well as supported by renderer.
- Arrange colored markers at measured metric board positions with measured `T_MS`. Use the **same detection code** in real and simulated RGB frames. Simulate lens distortion in processing or undistort both domains consistently.
- Camera occlusion is not optional: the plugin housing, gripper fingertips, camera bracket and board must occlude each other realistically. Collect screenshot comparisons at home, initial alignment standoff, near-contact and at simulated contact.
- Simulated perfect depth is **not equivalent** to D405 stereo depth. Add learned/measured depth noise, invalid pixels, range clipping, dropouts and color/depth registration errors; preserve a `depth_valid` mask. RGB-D frames may be unavailable at certain poses, which must propagate to actor observation validity.
- Start actor observations with marker pixels, confidence/validity, depth statistics and proprioception; run explicit RGB-image student as a stretch. Do not label a feature actor as raw-image RL.

## 4. Phase boundaries / actuator and observation contracts

**No grasp RL:** Task A (or a validated fixed pose) grasps the housing. Task B learns only **pre-contact fine alignment and insertion**, starting from a fixed cradle-derived grasp or a precisely specified distribution of grasp offsets.

**Episode start:** robot reaches a known `PRE_INSERT` standoff with connector securely held; plug-to-tool transform is drawn from the **measured repeat-grasp distribution**. Socket frame fixed, optionally small plausible jitter within measured fixture uncertainty. An episode that starts with the plug already inside the socket is invalid unless intentionally constructed for debugging and excluded from evaluation.

**Actor observations available at deployment:**

```text
obs_t = [
  marker_pixel_errors_normalized,  marker_valid_mask,  marker_confidences,
  depth_summary_m, depth_valid_mask,
  q_arm, dq_arm,  T_BE-derived tool pose / velocity,
  gripper_position_or_measured_proxy, previous_action, phase_bit,
  optional short history of past visible features/actions
]
```

Do **not** include the exact simulator socket pose, exact plug pose/penetration, true contact wrench, solver states or latent perturbation in **actor** inputs unless equivalent physical sensors are installed. A privileged **critic** or teacher can access simulator states in training, but the exported student/actor must not.

**Actions v1:** incremental task-space pose `a=[dx,dy,dz,dyaw]`, in a declared frame (prefer socket frame within sim, mapped through estimated board/robot transforms at real deployment). Limits and period are measured/approved from checklist. **If an action requires true socket frame at runtime, use `T_BS_est` derived from the camera/marker and FK**, not a simulator oracle. Freeze roll/pitch initially after taught alignment; extend only if connector geometry clearly needs it.

**Execution layer:** action clip → collision/position check → velocity/acceleration/jerk limits → short Cartesian pose/velocity command → feedback checks → next observation. Keep the real driver interface separate from the training action head; same physical semantics and applied command logging in both.

**Physics/control timing:** define `dt_physics`, `control_decimation`, `camera_period`, observation delay and execution deadline as measured or benchmarked values. Include action hold/integrator smoothing, controller saturation and end-of-action feedback. The agent may be slower than servo/physics loops; the low-level controller must remain stable if inference stalls.

**Terminal labels:**

- `SUCCESS`: true simulated geometry fully engaged, within angular/lateral constraints, at target seating depth, stable after a dwell, and without forbidden excessive penetration/contact.
- `FAILURE_JAM`: measurable wedging / blocked motion / invalid contact; conservative stop, not an unrewarded endless push.
- `FAILURE_BOUNDS`: workspace, force *in sim*, collision or action safety limit crossed.
- `FAILURE_TIMEOUT`: episode maximum duration/steps.
- `SENSOR_UNAVAILABLE`: key visual feedback invalid beyond allowed period; abort or run separately tested recovery stage.

## 5. Reward / algorithm / curriculum design

### Primary study: sparse terminal success

```python
reward = 1.0 if verified_geom_success else 0.0
# Safety terminations prevent dangerous exploration; log them separately.
# Optional time/safety penalty forms a distinct 'sparse + regularization' ablation.
```

This genuinely sparse reward may not produce useful exploration at final real tolerances. Address it with staged initialization, demonstrations and/or a curriculum, not by silently replacing terminal success with a dense distance reward while still claiming pure sparse RL.

### Suggested curriculum (with pre-registered transitions)

| Stage | Setup | Observation | Reward / learning | Exit gate |
|---|---|---|---|---|
| C0: motion check | plug in free-space pre-contact pose | proprioception + visual features | temporary supervised/shaped alignment bootstrap | correct signed actions and no controller instability |
| C1: accessible insertion | generous *sim-only* lead-in/clearance, near-correct start | visual features + q/pose | sparse terminal success, demo resets if needed | repeat successes on new seeds |
| C2: target geometry | exact measured clearance and chamfers | same deployment inputs | sparse terminal success | performance on untouched seeds recorded |
| C3: robustness | measured grasp, board, camera and friction perturbations | same deployment inputs + missingness mask | sparse success, safe termination | robust held-out performance |
| C4: visual transfer | RGB-D images rendered at actual EIH view, same perception pipeline | camera-derived feature actor or image student | distillation/BC or policy fine-tuning | real/sim feature statistics validated |
| C5: robot gate | physical Gen3 Lite in approved fixture | **same actor input/action schema** | no autonomous unsafe exploration; evaluate | independent physical seating outcomes |

**Curriculum promotion rule:** do not increase difficulty until an evaluation window on **held-out seeds** meets a pre-registered success floor, e.g., 70% over 100 episodes; use exactly the same threshold across runs unless documented. If training plateaus, diagnose access to successful rollouts, perception/physics shortcuts and action bounds **before** adding arbitrary dense reward.

**Domain randomization distributions:** derive central values and bounds from real measured errors and safe perturbation windows. Include `T_EP` variance, `T_EC` noise, socket board pose jitter (only if board physically moves), RGB/depth missingness, latency, control tracking, friction and cable disturbance. Never randomize 5 mm of hole offset when the real hole clearance is 0.2 mm and the arm would scrape the board; distinguish initial alignment uncertainty from safe contact displacement. Freeze an evaluation distribution and seeds; never train on them.

### Three policies / ablations

- **B0 — Scripted:** deterministic Task A alignment and guarded insertion logic reproduced under same simulation controller semantics.
- **B1 — Standalone RL:** PPO or SAC actor trained from sparse reward and curriculum, fed only deployment-observable features/proprioception.
- **B2 — Demo-augmented or residual RL:** optionally behavior clone A's logged successful corrections, then learn bounded residual `u = u_nominal + clip(pi(obs), delta)`; the nominal controller must be identical to comparison except for explicit residual. Record demonstration count, quality, source and dataset split.

If demonstrations are provided by Task A real trials, ensure their camera features, actions, observed state and success labels are converted to the same schema. Do not use only successful trajectories to claim recovery ability; collect near-failures, safe recovery examples and diverse offsets. A common reset distribution and observation/action limits are needed for a fair comparison.

## 6. High-fidelity training architecture (Isaac Lab)

```text
assets/
  kinova_gen3_lite/  robot.usd, imported_urdf.sha256, actuator_config.yaml
  gripper/          geometry, friction and jaw calibration
  connector_v1/     plug_visual.usd, plug_collision.usd, measurements.json
  socket_v1/        board.usd, socket_collision.usd, entry_frame.json
  camera/           mount.usd, intrinsic_extrinsic.yaml, noise_profile.json

sim/
  scene.py           robot + plug + board + EIH camera (one per cloned env)
  cfg/               exact backend/version/solver/contact/decimation
  observations.py    marker pixel/depth + robot feedback, all with valid flags
  actions.py         bounded incremental pose commands via low-level executor
  rewards.py         verified geometric seating, optional ablation penalties
  terminations.py    success/jam/out-of-range/timeout/missing data
  resets.py          valid standoff + grasp uncertainty, no object teleport during steps
  events.py          measured randomization: friction, camera, latency, light
  curriculum.py      progression C0-C5 and promotion criteria
  wrappers.py        common real/sim observation/action schema
  training/          rsl_rl or rl_games agent configs; configs pin seeds

experiments/
  real_vs_sim/       paired images, marker and depth error, step responses
  benchmark_envs/   1/8/16/32/64 throughput, VRAM and camera rates
  training/         loss, success, checkpoints, config snapshots, code commit
  evaluation/       held-out seeds, CSV trial reports, videos, failure labels
  deployment/       offline replay, policy adapter, safety gate, physical logs
```

**Isaac Lab design:** implement an `ManagerBasedRLEnv` task (or direct RL env if you need custom update order), with separate scene, actions, observations, rewards, terminations, events and curricula. Store each change in Git. Don't write long monolithic scripts that hard-code mesh, camera mount and reward values.

**Parallel rendering optimization:** use the supported tiled-camera system, keep rendering cadence at the needed observation rate rather than each physics step, avoid saving every frame for every environment, and benchmark both state-only and image-in-the-loop speeds. Train a marker-feature actor quickly from rendered/analytic feature extraction only if real and simulated features are shown to agree; verify with full RGB-D rollouts and the same detector. Performance is not a scientific excuse for training with perfect future-contact information.

## 7. Real-to-sim calibration / validation experiments

### V0 — Kinematics and control

- Compare simulated vs real joint angles/EE poses for ≥5 approved non-contact joint configurations; check joint order/sign, quaternion convention and same TCP.
- Run small free-space stepped/triangular motion commands along each controlled axis; compare measured pose trajectories, lag, overshoot and tracking error. Tune or randomize dynamics based on recorded response.
- Verify gripper closure range/contacts and a securely-held housing. If sim uses rigid fixed attachment, report it as a planned simplification; do **not** use rigid-attach physics to claim handling of slip.

### V1 — Camera and marker agreement

- Capture synchronized/simulated views at ≥5 approved configurations (home, high standoff, near alignment, oblique, pre-contact). Overlay marker locations and compare per-corner residual error in pixels.
- Compare available/in-bounds depth at board points, not just perfect simulated depth. Measure real depth invalid fraction and frame-to-frame variance at each viewpoint; inject appropriate distribution in sim.
- Check that the plug occludes exactly the same parts of the socket/marker layout at near contact. If EIH marker feedback is lost in the physical scene, your actor must be trained with loss of visual information.
- Define acceptance tolerances **after** measurement and target insertion clearance are known. For example, show actual pixel error percentiles; don't assert an arbitrary '2 pixel' value proves sub-mm alignment.

### V2 — Contact and clearance agreement

- Bench test with a sacrificial plug/socket and supervised ultra-low-speed approaches. Capture engaged/not-engaged depth, lateral/yaw range, ability to retract, plug tilt, and force **only if a validated load cell/F/T exists**.
- Repeat same boundary conditions in sim. Verify contact exists at the right geometry, no numerical tunneling, and that plausible aligned paths succeed while obvious jams fail.
- Calibrate friction/contact solver within the support of measured behavior, not for the highest RL score. Report uncertainty if no measured force available.

### V3 — Digital twin freeze

- Archive all asset hashes, calibration transforms, real/sim paired frames, simulation parameters and a gap report (`measured`, `estimated`, `unknown`). Block any 'high-fidelity' label if critical geometry or camera extrinsic remains unknown.

## 8. Deployment plan, safeguards and stage gates

1. **Offline replay:** run the exported actor on recorded real EIH streams and simulated replay trajectories. Check feature normalization, frames, action sign, missing-data behavior and predicted action bounds.
2. **Shadow mode:** physically move through **pre-contact safe space only** with policy predicting/logging corrections while the supervisor prevents any contact. Compare to Task A output.
3. **Pre-contact policy control:** allow small safe lateral/yaw corrections with watchdog, workspace filter and physical E-stop. Do not execute an insertion until visual/robot and seating-sensor gates are satisfied.
4. **Guarded insertion:** physical lower-level primitive—not the network—enforces the allowed axial motion, time, envelope, independent seating check, stop/retract policy, and human override.
5. **Supervised evaluation:** fixed evaluation sheet, paired task conditions, attempts counted, exact controller/checkpoint hashes, initial pose errors and completion labels.

**Gates:**

- **B-G0:** measured parts/frames & validated compute environment available.
- **B-G1:** correct joint, camera and gripper simulation; contact doesn't tunnel.
- **B-G2:** real-vs-sim kinematics/camera/contact evidence available with known residual gaps.
- **B-G3:** parallel env + reproducible train/evaluate pipeline.
- **B-G4:** simulation baseline, standalone RL, and residual/demo comparison on held-out seeds.
- **B-G5:** offline and shadow-mode deployment adapter safe.
- **B-G6:** physical supervised insertion with independent success and emergency stop. B-G6 is mandatory to claim **successful sim-to-real transfer**.

If B-G1/B-G2 fails due to geometry quality, do **not** bypass it by randomizing everything. Record `SIM_MODEL_NOT_VALIDATED`. If B-G6 is unavailable, deliver reproducible simulated results and realistic deployment limitations without misrepresenting physical performance.

## 9. Rolling first 10 days and longer research sequence

| Day | Task B work | Hard artifact | Dependency |
|---|---|---|---|
| 1 | Check RTX/RAM/driver, install pinned compatible Isaac Sim/Lab, run vendor sample | environment/version/GPU report | computer available |
| 2 | Import actual Gen3 Lite description, build USD/arm articulation | robot joint map, one joint-command smoke test | arm model from checklist |
| 3 | Import gripper, connector, socket, board and correct contact meshes | render/collision debug video | connector measurements |
| 4 | Match EIH mount, intrinsics, board colors/markers and optical frames | side-by-side camera views | real camera calibration |
| 5 | Test insertion contact and geometric success/fail oracle | aligned, offset, blocked unit test suite | collision geometry |
| 6 | Code Gym/Isaac Lab actions/observations/resets | deterministic env smoke test | real action schema |
| 7 | Scripted Task A controller in sim, run basic distribution | seeded baseline log | classical API agreement |
| 8 | Implement rewards, terminations, curricula; first PPO single/small batch | training sanity plot | validated success oracle |
| 9 | Benchmark parallel envs and camera memory/performance | `num_envs`/steps-s/VRAM table | suitable GPU |
| 10 | First RL training checkpoint, test on held-out seeds, model-gap review | report: successes/failures/blockers + updated sim config | gates B-G0...B-G3 |

**After Day 10:** complete curriculum, tune training only on training seeds, introduce measured domain randomization, demo/residual run, video-based real-vs-sim validation, offline/shadow safety test, and supervised physical evaluation. The next 5–10 days are a discovery/calibration window, **not** a promise of fully trained and transferred RL within 10 days.

## 10. Acceptance test matrix and mandatory failure investigation

| Test | Expected correct behavior | If not, fix before advancing |
|---|---|---|
| Import q/joint limits | exactly six actuated arm joints match real TF | wrong URDF, axis or articulation import |
| Collision cavity sanity | nominal plug enters; offset jams; no tunneling | collider topology/contact settings |
| Camera mount | real and sim viewpoints consistent at measured poses | `T_EC`, optical frame or intrinsics |
| Missing RGB / depth | actor sees validity mask, stops or documented recovery | observation preprocessing |
| Random actor | cannot command outside safe bounded action range | action adapter / supervisor |
| Reward anti-cheat | no success for mere proximity, lateral overlap, penetration | success oracle |
| Training convergence | held-out success rises and failures logged | reward sparsity, curriculum, action bounds |
| Parallel environment determinism | reproducible under fixed seed within backend limitations | reset, seeds, synchronization |
| Sim/real input equivalence | actor input shapes/units/order identical | adapter / normalization |
| Real safety | inference delay, command timeout, manual E-stop all controlled | real deployment blocker |

**Two mandatory negative controls:** a wrong-orientation keyed plug must *not* count as inserted; a plug commanded past the socket without actual seating must *not* count as success. In both cases, evaluate simulator geometric oracle and physical seating sensor separately.

## 11. Evaluation and reproducibility

Metrics for B0 scripted, B1 standalone RL and B2 residual/demo RL: simulated first-attempt and overall seated success, cumulative safe action cost, action count, final lateral/yaw error, number of collisions/jams and recovery attempts, sample efficiency (environment transitions to threshold), wall time, steps/s, VRAM peak, real seating outcomes, sim-to-real performance gap and failure categories.

Perform **three or more independent training seeds** where compute permits, and report mean/spread; for comparison, use ≥100 fixed held-out simulation episodes per distribution per policy. Record whether perception is true rendered RGB, camera-derived pixel/depth features, or state-based teacher. The **vision-conditioned** claim requires actual camera-equivalent observations at deployment.

Avoid invalid comparisons: policies must use identical initial pose/connector tolerances and verified success labels, the same lower-level command limits, and **the same observation information** (or explicitly mark a privileged teacher). RL may lose to a good deterministic controller when fixture repeatability is high: this is an interesting result, not a reason to move goalposts.

## 12. Change management (important for next 5–10 days)

Each change increments `setup_revision` and lists geometry, camera and control hashes. Use three levels:

- **Minor:** color values or lighting → update marker appearance and noise distributions, rerun camera comparisons.
- **Moderate:** camera mount or fixture placement → recalibrate `T_EC`/`T_BM`, reset desired image features, update USD, revalidate camera mapping, retrain/evaluate vision actor if observation distribution changed.
- **Major:** connector/keying, gripper or socket geometry → rebuild collision meshes, socket/plug transforms, success oracle, safe envelope and RL curriculum; old and new experiments belong to distinct domains.

Never overwrite the original evaluation set or relabel physical failures based on updated geometry. Run old policy on new setup as a **zero-shot transfer** experiment only after safety checks and explicitly label the distribution shift.

## 13. Primary references / official guides

**Literature (from user's provided review):** Schoettler et al. 2019 visual industrial insertion (https://arxiv.org/abs/1906.05841); Davchev et al. 2022 demo residual RL (https://arxiv.org/abs/2008.07682); IndustReal 2023 (https://research.nvidia.com/publication/2023-05_industreal-transferring-contact-rich-assembly-tasks-simulation-reality); Marougkas et al. 2025 residual policy (https://arxiv.org/abs/2505.11858); SHaRe-RL 2026 (https://arxiv.org/abs/2509.13949). **Most cited connector systems also use force/tactile; do not claim sensor equivalence.**

**Vendor/engineering references (verify release compatibility on installation date):**

- Isaac Lab install/requirements: https://docs.nvidia.com/isaaclab/isaac-lab/installation
- Isaac Sim requirements and compatibility checker: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- Isaac Lab manager-based scenes and RL: https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/2026-10.0/isaac-lab-basics/05-manager-based-workflows.html
- Isaac Lab camera/tiled rendering cost: https://docs.nvidia.com/isaaclab/concepts/sensors/camera
- Isaac Lab RL libraries / PPO/SAC choice: https://docs.nvidia.com/isaaclab/concepts/reinforcement-learning
- Isaac Sim URDF import and articulation tuning: https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html
- Isaac Sim robot import-to-pick workflow: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup_tutorials/tutorial_manipulator_workflow.html
- Isaac Lab simulated camera/noise/domain randomization guidance: https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/latest/transfer-policies-from-simulation-to-reality/index.html
- Kinova ROS 2 description and hardware controllers: https://github.com/Kinovarobotics/ros2_kortex

**Non-negotiable scientific rule:** A visually convincing rendering or high simulated success rate does not prove real contact physics, and a state-privileged trained actor is not a deployable vision policy. Measure both gaps explicitly.
