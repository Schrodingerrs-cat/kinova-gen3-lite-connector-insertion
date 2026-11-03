# Task A — Eye-in-Hand IBVS + Bounded Search for Cable-Connector Insertion

**Project:** Vision-Based Manipulation | **Status:** implementation specification v1.0 | **Updated:** 2026-11-02  
**Hardware target:** Kinova Gen3 Lite (6 DoF), Gen3 Lite two-finger gripper, one rigidly mounted eye-in-hand (EIH) RGB-D camera (D405 if confirmed), fixed plug cradle, fixed socket board, visual markers beside socket.  
**Single source of measured hardware truth:** [`SHARED_PRIMITIVES_CHECKLIST.md`](SHARED_PRIMITIVES_CHECKLIST.md). **Companion:** [`Task_B_Isaac_Sim_RL_PLAN.md`](Task_B_Isaac_Sim_RL_PLAN.md).

> **Frozen goal:** Autonomously grasp the **rigid plug housing**, align to a known socket using the EIH camera, perform guarded physical insertion, and independently classify success/failure. Exact connector, fixture, camera mount, and measured offsets may change; the goal, interfaces, experiment protocol, and evidence standard do not.

## 1. Firm deliverable and definition of done

**Deliverable A:** A reproducible ROS 2 application that runs one full *pickup → visual alignment → bounded retry/insert → physical seating verification → retract/stop* cycle on the Kinova Gen3 Lite, with video, timestamped logs, and side-by-side evaluation against a taught-pose baseline.

**Minimum acceptance (all required):**

- [ ] A saved configuration with identified connector, coordinate conventions, measured socket/plug/camera geometry, and verified robot-control limits. No unmeasured offsets disguised as constants.
- [ ] Repeatable **housing-only** pickup from the fixed cradle; logged gripper position and plug-tip repeatability. If repeatability is poor, add an approved regrasp/fixture change before insertion.
- [ ] Image features continuously tracked from EIH camera with confidence and occlusion reporting. At least one measured successful servo-alignment approach from a deliberately perturbed **pre-contact** robot pose; no contact while tuning.
- [ ] Explicit IBVS law minimizes **image-space** error for a documented set of controlled DoFs; any separate depth/pose component is labeled as hybrid, not called pure 6-DoF IBVS.
- [ ] Validated guarded attempt primitive. Stop/retract on marker loss when necessary, stale robot feedback, timeout, out-of-bounds pose, insufficient independent progress evidence, or other defined fault.
- [ ] Bounded candidate-offset search, with fixed trial count/time budget and no force-guided rubbing/search unless adequate contact sensing and compliance are actually validated.
- [ ] **Independent seating label** from an electrical low-voltage continuity check, seating switch, or another documented validated measurement; robot displacement or RGB-D depth alone is not accepted as ground truth.
- [ ] Run and archive a taught-pose baseline (B0), IBVS without search (A1), and IBVS+search (A2) using matched initial poses and the same safety/verification layer. Record successes *and* failures.
- [ ] Supply executable code, parameter manifest, calibration record, logs/CSV, at least one annotated execution video, plots, and a failure analysis explaining observed limits.

**Performance targets (goals, not pass/fail claims):** target >= 24/30 verified successes on nominal physical trials for A2, and improvement relative to A1 under safe prevalidated perturbations. Actual results may be lower; an accurately documented negative result still satisfies research reporting, but does **not** count as a successful hardware insertion demonstration. Pre-register the conditions before comparisons. If physical time is limited, report exact `n`, do not silently claim statistical confidence.

**Out of scope:** learned grasp policy, unstructured cable picking, routing a deformable cable, unguarded high-force push, in-contact continuous spiral guided by imaginary force feedback, 6-DoF force regulation without an F/T interface, multiple arbitrary sockets, and a second camera unless later justified experimentally.

## 2. Critical assumptions / no-go checks

1. Camera: EIH, lens can see marked board at reachable standoff; marker design stays visible through the **alignment phase**, not necessarily during final engagement.
2. Plug: rigid keyed housing is presented by a repeatable cradle; cable is not grasped, pinched or pulled at the strain relief.
3. Socket: fixed to a rigid board; known offset from visual markers. Fixture must withstand the attempted motions without yielding or shifting.
4. Contact: no wrist F/T is assumed. Robot-side current/following error may help detect anomalies **only after explicit validation**; not equivalent to calibrated axial force.
5. Physical contact: first insertion tests require supervisor approval, a hardware emergency stop, conservative measured limits, and a sacrificial/replaceable connector setup. Never rely on software stop alone.
6. Kortex: the official ROS 2 driver includes the Gen3 Lite description and a `twist_controller` for hardware, but this interface is **not** available in its mock/Gazebo simulation; validate the actual command topic, controller switching, reference frame and watchdog on the real arm.
7. Camera D405 (if used) has published depth range 7–50 cm. Measure actual depth reliability, minimum standoff, and occlusion; do not assume usable contact-depth readings.

## 3. Interfaces shared with Task B

Use the same timestamped data contract in both tracks. All poses in meters and quaternions; all angles in radians internally. Log `frame_id`, units, validity bits and calibration version.

**Frames:** `B` robot base, `E` end-effector/tool flange, `G` grasp tool center point (TCP), `C` optical camera, `M` marker board, `S` socket entry frame, `P` plug mating frame. Define insertion axis `+z_S` or `-z_S` **after measurement**; do not infer it from camera axes.

Core transforms:

```
T_BS = T_BE(q) @ T_EC @ T_CM @ T_MS
T_BP = T_BE(q) @ T_EP
T_SP = inverse(T_BS) @ T_BP
```

`T_EP` assumes stable grip; measure uncertainty (repeat grasps). `T_CM` may be estimated from marker geometry / camera model. If using pure IBVS, `T_BS` need not be estimated at each servo tick, but these transforms remain essential for initial taught pose, standoff, axial motion, safety checks and documentation.

**Shared APIs (suggested):**

```python
observation = perceive(timestamp)  # features_px, valid_mask, depth_m, joint_state, ee_pose, camera_ok
safe_grasp(config)                 # fixed fixture-based housing grasp
run_alignment(observation, goal)   # outputs bounded correction / status
attempt_insert(guarded_limits)     # separate short motion primitive; never assumes success
verify_seated()                    # independent success signal, confidence/validity
retract_to_clearance()             # guaranteed motion path confirmed in free space
abort(reason)                      # stop commands, state transition, log, human reset
```

**Data columns:** `run_id`, `timestamp_ns`, `controller`, `config_hash`, `connector_id`, `phase`, `q`, `ee_pose`, `camera_pose`, `features_px`, `desired_features_px`, `feature_confidence`, `depth`, `u_command`, `u_executed`, `contact_proxy_valid`, `verification_signal`, `verified_success`, `stop_reason`, `trial_seed/perturbation`, `attempt_index`.

## 4. Work packages and implementation details

### A0 — Characterize robot/camera/fixture (before closed loop)

- Export actual robot URDF/Xacro, live joint names, joint limits, controller interface and gripper control. Do not assume Gen3 7-DoF naming; Gen3 Lite has 6 arm joints.
- Inspect `ros2 control list_controllers` and identify the allowed transition between trajectory and twist control; test a tiny **non-contact** motion with emergency-stop supervisor, then stop publishing and verify watchdog behavior.
- Confirm camera RGB/depth modes, intrinsic calibration `K`, distortion parameters, optical-frame direction, FPS, image/depth alignment, timestamp behavior. Recheck the already-seen `device busy` issue and depth-color/pointcloud texture mismatch before depending on depth.
- Record full marker positions on board, board-to-socket transform, connector dimensions, safe approach clearance, and mechanical insertion path.
- Determine whether payload and camera bracket leave tool and connector within the real arm's rated load/torque capacities.

**Exit evidence:** calibration YAML, photograph/drawing, TF tree, controller-state log and short no-contact jog video.

### A1 — Deterministic grasp and taught-pose baseline (B0)

1. Home (safe, predefined), move to pre-grasp above cradle.
2. Position jaws around **housing side faces**, not the flexible cable; command a conservative validated gripper closure.
3. Lift to pre-inspection pose; verify grasp by appearance or a validated gripper-position cue. Do not infer successful grasp from commanded closure alone.
4. Record estimated `T_EP` and its variation across at least 20 repeated pickups.
5. Execute taught pre-insertion approach. Only attempt axial motion after collision envelope and independent success sensor are validated.
6. Save B0 results; if B0 is already robust, use measurable perturbations later so IBVS and RL are meaningful.

**Exit evidence:** repeat-pick plot, taught trajectory and baseline runs.

### A2 — Markers, tracking and hand–eye geometry

- Prefer 4 or more uniquely identifiable marker corners/centroids arranged around (not within) the opening; ensure view across the approach. Color blobs can work but need stable identities and illumination tests. AprilTag/ArUco corners offer a stronger alternative if color tracking fails.
- Calibrate camera intrinsics and fixed `T_EC`; verify through multiple non-contact wrist poses that the *stationary* board transforms to a consistent `T_BM`.
- Calibrate `T_MS` (board-marker to actual socket entry) using physical dimensions/fixture CAD and independent checking; a marker center is not the socket center.
- Validate RGB segmentation/detection on real recordings under different lighting, reflections, camera distances and wrist angles. Log dropped features and depth invalidity.
- Use fronto-parallel marker geometry or a known 3D layout as appropriate. Depth from the D405 can estimate feature range for an IBVS interaction matrix; invalid pixels require robust fallback/abort.

**Exit evidence:** detection unit tests, re-projection/pose consistency errors, demonstrated marker visibility at alignment standoff.

### A3 — Image-based servo control (A1)

**Controlled DoFs v1:** two lateral translations tangent to socket plane plus insertion-axis yaw. **Do not** promise full 6-DoF alignment from image centroids. Hold/slowly program insertion-axis standoff and roll/pitch separately from taught geometry unless you have well-observed rotational features.

For point features `s=[u1,v1,...,un,vn]^T`, desired features `s*`, error `e=s-s*`, and per-feature depth estimates `Z_i`:

```
xi_camera = -lambda * damped_pinv(L(s, Z)) @ W @ e
xi_tool   = transform_twist(C -> tool, xi_camera)  # use correct adjoint/convention
xi_safe   = project_to_allowed_axes_and_bound(xi_tool, limits, collision_envelope)
```

- Use calibrated **current and desired** image features at a known safe taught pre-contact camera viewpoint; do not blindly reuse desired pixels across camera re-mounting without recalibration.
- Confirm interaction-matrix units, sign, feature correspondence and coordinate conventions via **one-axis, no-plug/no-contact** experiments.
- Filter/clip image corrections and use a damped pseudoinverse with feature weights; monitor Jacobian conditioning and limit twist speed/acceleration.
- Enforce `N` consecutive in-tolerance frames before declaring convergence; require valid feature count, confidence, pose/workspace bounds and a timeout.
- When markers are lost, command zero / stop, retract if safe, reacquire from a known standoff. Never continue open-loop IBVS into contact due solely to last-seen features.

**Exit evidence:** plots of pixel error vs time; repeat alignment from safe signed initial perturbations; document which DoFs are controlled by image error vs fixture geometry.

### A4 — Guarded axial insertion

- Start at an approved **pre-contact** pose and standoff; approach axially in short, incremental, speed-limited steps, with a defined maximum path and stop conditions. Every limit comes from the signed primitive checklist, not from a generic example value.
- Check robot follow-through, position, controller health and independent seating evidence after each step. Without a real F/T sensor, this cannot reliably distinguish all jamming modes: stop on anomalies, never 'push harder until it goes in'.
- Validate success independently (preferred: electrically isolated low-voltage continuity plus physical seating cue, or an independently validated switch).
- Treat each of these as different states: `INSERTED_VERIFIED`, `NOT_INSERTED`, `UNKNOWN_SENSOR`, `FAULT`. Do not label `UNKNOWN_SENSOR` as failure or success without follow-up evidence.
- Retract along a prevalidated collision-free path if incomplete. Protect against grip slip or cable snagging; call human reset if safe recovery cannot be guaranteed.

**Exit evidence:** successful physical seating and negative tests (off-center plug / no plug / jam surrogate) showing no false success and controlled aborts.

### A5 — Bounded discrete search (A2)

Use **retract → candidate correction → reacquire/IBVS if visible → short guarded insertion → independent verify**. This is not a force-guided continuous spiral.

Search candidates are generated in the socket plane and optionally around yaw, with measured safe bounds:

```
candidates = [(0,0,0)] + ordered_grid_or_rectangular_spiral(
    dx=measured_dx, dy=measured_dy,
    max_x=validated_bound_x, max_y=validated_bound_y,
    yaw_offsets=approved_yaw_candidates
)
for candidate in candidates[:max_attempts]:
    safely_retract_to_search_standoff()
    apply_candidate_in_socket_frame(candidate)
    run_ibvs_or_verify_pose_at_standoff()
    result = guarded_short_insertion()
    if result == VERIFIED_SUCCESS: break
    if result in (UNSAFE, INDETERMINATE, SENSOR_FAULT): abort()
```

- Candidate order: start at nominal; prioritize offsets likely under **measured** uncertainty covariance. For a rectangular keyed plug, raster/rectangular spiral plus yaw perturbations is a better starting design than an unbounded circular spiral.
- `dx`, `dy`, yaw offsets, retries and search radius are **TBD until measured**. They must not exceed free-play/fixture/safety bounds. No arbitrary 1-mm search assumption.
- No sliding the connector across the board under sustained load; keep attempts short, isolated and retract between them. A fixture-mounted compliant entry guide can help, but characterize it and document it.

**Exit evidence:** 2D trial-offset map with verified success/failure/abort, no unsafe-contact incidents.

### A6 — Hardware integration and evaluation

**State machine:** `NOT_READY → CALIBRATE_CHECK → PICK → VERIFY_GRASP → PREVIEW → SERVO_ALIGN → GUARD_CHECK → ATTEMPT → VERIFY → (DONE | RETRACT → SEARCH_NEXT | ABORT)`. Every transition logs timestamp, data validity, reason and controller identity.

**Matched conditions:** (i) nominal, (ii) signed lateral errors, (iii) signed yaw errors, (iv) repeat-grasp variation, (v) partial marker dropout/lighting change where safe. Exact ranges come from calibration and approval, not from publication defaults.

**Pre-registered physical matrix (proposal):** 30 nominal trials for B0, A1 and A2; 30 safe perturbation trials per controller, paired or randomized by offset set. Split into multiple supervised sessions if necessary. Track connector wear and change/retire damaged parts; always report how many trials were actually completed.

Report: success on first attempt; eventual verified success; attempts; time; image error; false positives/negatives of success detector; sensor invalidity; robot motion fault; plug slippage; per-offset results; contact force **only if actually measured**; exact failure videos. Provide binomial confidence intervals for success rates.

## 5. 10-day initial execution plan (rolling dates)

| Day | Work | Concrete end-of-day artifact | Must be cleared before |
|---|---|---|---|
| 1 | Audit robot, connector, cameras, available contact/seating sensing | `hardware_snapshot.yaml`, setup photographs, control interface list | Robot motion |
| 2 | Define frames, measure fixture and connector, mark safe envelope | draft primitives checklist + frame diagram | Insertion motion |
| 3 | Fix camera streaming/permissions; collect RGB-D and marker recordings | 3–5 sample bags and intrinsics | IBVS |
| 4 | Hand–eye/board-to-socket calibration | calibration transform files + static-target test | Automated alignment |
| 5 | Taught housing grasp + 20 pickup repetitions | gripper logs + tip-repeatability data | Plug insertion |
| 6 | Marker detector and desired image reference | detector validation video/plots | Velocity control |
| 7 | IBVS simulation or no-contact, no-plug motion | signed-axis response and tracking log | Servo near socket |
| 8 | Guarded pose approach + seating sensor testing | controlled success/failure cases | Automated insertion |
| 9 | Search candidate generator and supervisor | dry-run retry sequence, abort test | Physical retries |
| 10 | Controlled demonstration if gates pass, otherwise blocker review | A1/A2 evidence + revised plan | Further trials |

This is a **gated work plan**, not a promise that contact experiments are safe by Day 8. If the setup changes on Day 5–10, update measurements and rerun the affected validation gates; retain the fixed scientific deliverable.

## 6. Test pyramid / failure modes

| Fault | Detection or test | Required response |
|---|---|---|
| Camera unavailable / `device busy` | RGB-D watchdog; camera start/stop test | No action; fix exclusivity |
| Marker color confusion / glare | visibility count, temporal ID consistency, confidence | Stop IBVS and reacquire |
| Calibration drift / camera re-mounted | stationary board transform consistency | Recalibrate `T_EC` and `s*` |
| Wrong twist sign/frame | single-axis free-space test | Disable twist until corrected |
| Controller-switch fault | ROS 2 controller state transition test | Stop and restore known state |
| Socket blocked / misalignment | insertion progress + independent sensor not met | Guarded stop and retract; do not force |
| Plug slip | repeated grasp measurement, visual/fixture check | Abort and regrasp only if safe |
| Cable snag | cable clearance test and supervisor | Stop, human reset |
| Depth invalid/occluded | depth validity mask + feature visibility | Use approved stage transition or abort |
| Seating sensor stuck true | empty-socket and partial-insertion negative tests | Disable success labeling |
| Search candidate out of envelope | geometric bounds, collision checking | Reject candidate, do not execute |

## 7. Repository artifacts and reproducibility

```text
vbm_insertion/
  configs/
    hardware_snapshot.yaml
    calibration_<date>.yaml
    socket_<id>.yaml
    ibvs_controller.yaml
    safety_limits.yaml
  ros2_ws/src/vbm_insertion/
    perception/markers.py
    geometry/frames.py
    pickup/fixed_grasp.py
    controllers/ibvs.py
    controllers/guarded_insert.py
    controllers/bounded_search.py
    runtime/state_machine.py
    safety/guards.py
    verification/seating_sensor.py
    logging/trial_logger.py
  tests/
    test_frames.py
    test_marker_tracking.py
    test_ibvs_sign_and_bounds.py
    test_search_candidates.py
    test_guarded_stop.py
    test_seating_detector.py
  experiments/{B0_taught,A1_ibvs,A2_ibvs_search}/
  results/plots/ and results/videos/
```

Minimum plots: feature error vs time; insertion depth vs time with measured signal validity; offset success map; per-controller success first/overall with confidence intervals; attempt count and duration distributions. Record software commit, ROS distro, camera firmware, driver revision, connector part ID, trial seed and calibration hash.

## 8. Acceptance gates (the work that cannot be skipped)

- **G0 – Physical readiness:** measured connector geometry, camera, controller, E-stop, fixture, independent success sensor, safety supervisor signed off.
- **G1 – Free-space control:** grasp and twist/trajectory execution validated away from socket.
- **G2 – Observability:** repeatable marker detection, stationary-target calibration test, alignable pose without contact.
- **G3 – Contact readiness:** guard limits verified, physical negative/positive success tests, sacrificial connector and controlled access.
- **G4 – Classical demonstration:** A1 and A2 runs with logged independent success.
- **G5 – Comparative evaluation:** matched B0/A1/A2 trials and honest failure analysis.

**If G3 does not pass:** do not attempt unattended insertion. Deliver a functioning EIH IBVS alignment subsystem, guarded insertion design, search simulation/dry run, and explicit blocker list. This is a **partial hardware deliverable**, not a claim of full physical insertion.

## 9. What can change without invalidating the project?

| Change | Keep | Revalidate |
|---|---|---|
| Connector shape/size | IBVS/search interfaces, logs, success methodology | CAD, contact envelope, gripping faces, `T_EP`, `T_MS`, offsets |
| Marker colors / marker family | Feature-error servo architecture | detector, correspondences, desired pixels, visibility |
| Camera bracket location | EIH architecture and tests | `T_EC`, intrinsics if changed, occlusion, `s*`, safety envelope |
| Socket board placement | Frame conventions and controller | `T_BM`, target standoff and collision zones |
| Add load cell or F/T | staged approach and search baseline | calibrated force limits and corresponding safety cases |
| Add second camera | EIH primary baseline remains | perception fusion + separate ablation; never silently change baseline |

## 10. Primary references and implementation guides

- **User's provided research master file:** `insertion_papers_master_review(1).md`; relevant entries V02 (Song 2017), V09 (Shen 2023), V10 (Zhang 2023), V04 (Haugaard 2021); important caution that vision-first systems may rely on force control at contact.
- Shen et al. (2023), *Learning-Based Visual Servoing for High-Precision Peg-in-Hole Assembly*: https://doi.org/10.3390/act12040144
- Song et al. (2017), *Electric Connector Assembly Based on Vision and Impedance Control*: https://doi.org/10.1007/s12206-017-1144-7
- Kinova official ROS 2 Kortex driver, controllers, URDF and Gen3 Lite bringup: https://github.com/Kinovarobotics/ros2_kortex
- Intel official D405 specifications: https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html

**Planning note:** Any numeric threshold not measured from your own connector/robot appears as `TBD` or a clearly marked experimental target. Do not turn simulation-only force contact into a real-robot guarantee.
