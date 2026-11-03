# Shared Primitives & Measurements Checklist — Gen3 Lite Vision-Based Insertion

**Status:** canonical inputs; incomplete until measured and reviewed. **Updated:** 2026-11-02.  
**Used by:** [Task A — IBVS/Search](Task_A_IBVS_Search_PLAN.md) · [Task B — Isaac Sim/RL](Task_B_Isaac_Sim_RL_PLAN.md).

> **Stop rule:** No unmeasured dimension, camera transform, angle, force limit, or insertion depth may become an executable real-robot constant. An unset value remains `TBD`/`null` and blocks the corresponding gate. This file asks for both **geometric parameters** and executable **motion primitives**; they are different things.

## 0. Priority system and setup change record

- **P0 / START:** needed to start meaningful robot/simulation integration without guessing hardware interfaces. Ideally collected Days 1–2.
- **P1 / CONTACT:** required **before any physical socket contact or closed-loop EIH approach near the fixture**.
- **P2 / TRANSFER:** needed to claim realistic simulation, sim-to-real transfer or quantitative comparison.
- **Optional:** useful extension, not required for minimum demonstrator.

Record `setup_revision`, date, photo, serial/model or part-number where appropriate, source (CAD/caliper/ROS/real measurement/estimated), unit, uncertainty and reviewer. Change status from `[ ]` to `[x]` **only when a measurement or test artifact exists**, not when someone has guessed a value.

**Revision log** (copy one row each change):

| Revision | Date | Hardware changed | Affected transforms/configs | Revalidated by | Evidence |
|---|---|---|---|---|---|
| setup_r0 | TBD | Initial physical configuration | All | TBD | TBD |
| setup_r1 | TBD | TBD | TBD | TBD | TBD |

## 1. P0: Object and fixture identity (must provide first)

- [ ] **P0.01 Connector model:** manufacturer / part number / type / rev / datasheet URL or photo; nominal insertion direction and whether keyed/symmetric. `connector_id = TBD`.
- [ ] **P0.02 Plug body CAD or dimensioned drawing:** STEP/STL preferred; otherwise calibrated orthographic photos and caliper readings. Supply **rigid housing** width/height/depth; nose dimensions; outside corners/chamfers; orientation notch; terminal/pin positions if relevant. Units: mm. Values `TBD`.
- [ ] **P0.03 Socket CAD or dimensioned drawing:** opening width/height, internal clearance, lead-in/chamfer, insertion axis, full mechanical seating depth, flange location and mounting stiffness. Units: mm. Values `TBD`.
- [ ] **P0.04 Connector insertion tolerance:** minimum/nominal/maximum radial or per-axis free clearance; acceptable yaw/pitch/roll tolerances; must come from geometry/datasheet/test, **not** arbitrary search step. `clearance_x_mm/y_mm`, `max_alignment_error_deg = TBD`.
- [ ] **P0.05 Socket board location:** photograph + metric reference; rigid board size/thickness, orientation in robot workspace, mounting points and free cable path. Metric or CAD file.
- [ ] **P0.06 Fixed pickup cradle:** CAD/photo, cradle datum, housing resting pose, clamp/release mechanics, accessible gripper faces, plug presentation repeatability and cable exit direction.
- [ ] **P0.07 Robot version:** verify model `gen3_lite`, serial/firmware (store privately if necessary), 6-arm-joint ROS names, gripper model, robot description package revision, controller/launch configuration.
- [ ] **P0.08 Camera identity:** exact RealSense model (D405 **only if confirmed**), mount type, attached link, USB connection, RGB/depth streams, supported resolutions and rates, driver version, serial stored privately.
- [ ] **P0.09 Host compute:** physical ROS PC CPU/RAM/OS/ROS distro; intended **Isaac training host** RTX model/VRAM/system RAM/driver. Confirm whether these are separate machines; do not assume the physical-robot PC can run camera-parallel Isaac Sim.
- [ ] **P0.10 One overview photograph/diagram with dimensions:** label robot base, board, cradle, socket, camera, cable, markers; include scale and a coordinate-axis sketch.

**P0 output:** `setup_r0/hardware_manifest.yaml`, `photos/`, `CAD/`, `ros_topics.txt`, `frame_diagram.png` and `gpu_compatibility.txt`. **P0 STOP:** If exact connector geometry is unknown, initial Isaac training can study generic insertion *only*, not claim a high-fidelity physical twin.

## 2. Coordinate frames and transforms — shared language for both tasks

**Frame contract:** `B` base; `E` flange/tool frame; `G` gripper TCP; `C` camera **optical** frame; `M` marker-board frame; `S` socket mouth center; `P` mating plug frame; `R` cradle datum.

**Transform notation:** `T_XY` maps points expressed in **Y** into **X**. Save 3D translation in **meters** and orientation as a **normalized quaternion with documented component order** (e.g., xyzw). Also attach source, uncertainty, timestamp, and `setup_revision`.

- [ ] **P0.11 Frame axes convention** documented: `+x_S`, `+y_S` lie in socket entrance plane; `+z_S` chosen insertion direction; separate the incoming/outgoing signs. `+z_S` is **not automatically +z camera**.
- [ ] **P0.12 `T_BE(q)`** forward kinematics read from actual `/joint_states` and robot description; verified against live TF at ≥5 non-contact poses.
- [ ] **P1.01 `T_EC`: tool to camera optical frame**, 6-DoF hand-eye transform from calibration or measured mount plus verification; translation [m], quaternion, uncertainty.
- [ ] **P1.02 `T_EM` or observed `T_CM`:** actual camera observation of marker board with pose/corner association and validity; define which transform detector publishes.
- [ ] **P1.03 `T_MS`: marker frame → socket entry center**, position xyz [m], angle [rad], uncertainty; measured from actual board, *not* marker centroid assumption.
- [ ] **P1.04 `T_BR`: robot base → cradle datum**, measured/taught 6D pose and uncertainty.
- [ ] **P1.05 `T_EP`: end-effector → plug mating frame after grasp**, measured from grasp/calibration; repeatability distribution from multiple picks. This is the major unobserved uncertainty if EIH camera sees only socket.
- [ ] **P1.06 `T_BG`: base → gripper TCP / TCP definition**; true tool center, fingertips reference, approach normal, gripper closing axis.
- [ ] **P1.07 `T_BS`: base → socket** computed and checked across several wrist poses with socket stationary; variation reported.
- [ ] **P1.08 Correct twist/pose frame mapping:** command is in which robot frame? Test signed `+x,+y,+z,+yaw` free-space and validate direction **before closed-loop control**.

**Must provide:** an image or diagram with `B,E,G,C,M,S,P,R` axes and a versioned calibration file. **P1 STOP:** uncertain transform sign or unknown socket axis blocks contact.

## 3. Camera & visual-feature primitives

- [ ] **P0.13 RGB camera topic** and active frame ID; image size `[width,height]`, FPS, encoding, optical frame convention and exposure settings.
- [ ] **P0.14 Depth topic** and alignment to RGB; depth scale [m/unit], valid range, frame ID, FPS, valid-pixel mask; verify device not busy and correct texture/stream choice.
- [ ] **P1.09 Intrinsic matrix `K`**, `fx,fy,cx,cy`, distortion coefficients/model, image scaling/undistortion method; link to camera-info file.
- [ ] **P1.10 Marker board design:** feature type (`color_blob` / ArUco / AprilTag), unique marker IDs/colors, physical positions in `M` [m], size [mm], spacing, and positions relative to socket; include 4+ reliable features where possible.
- [ ] **P1.11 Marker tracking thresholds:** HSV/color ranges if blobs, area bounds, morphology/filter parameters, minimum confidence, allowed missing markers, pixel tracking jitter measured across 100+ stationary frames.
- [ ] **P1.12 `s*` desired image features:** capture at a correct **safe pre-contact** view; save per-marker target pixel center/corners and corresponding standoff/gripper pose. Change when camera mount/part changes.
- [ ] **P1.13 Visibility envelope:** for each of `pick_exit`, `initial_approach`, `servo_standoff`, `near_contact`, record #visible markers, depth validity, socket occlusion, image of each pose and physical clearance.
- [ ] **P1.14 Depth accuracy audit:** at 3–5 known camera-to-board distances, log measured median depth, absolute error, invalid fraction, frame variance, lighting sensitivity. Do not assume pixel-to-mm conversion is invariant with depth.
- [ ] **P2.01 Sim camera match:** actual `T_EC`, intrinsics, resolution, optical axes, camera period, clipping, RGB marker colors/lighting, depth noise/dropout, occlusion and timestamp delay; paired real/sim images.

**What to supply:** bag/recording at 3 or more fixed wrist poses, camera info, sample annotated images and validation report. **P1 STOP:** a marker that disappears before servo convergence blocks the planned IBVS phase.

## 4. Grasp primitives (housing, not cable)

- [ ] **P0.15 Approved gripping faces:** draw housing ROI / opposite planar faces; mark areas not to contact (pins, latch, cable, strain relief, fragile clips).
- [ ] **P1.15 `PRE_GRASP` pose:** named safe pose in B frame, max approach speed and tool direction; sufficient cable/jaw clearance.
- [ ] **P1.16 `GRASP` pose:** center/TCP, gripper jaw normal, wrist orientation, relation to cradle, taught waypoint(s).
- [ ] **P1.17 Gripper open/close commands:** command interface, legal range, units, speed, closed position around housing, validated grip force if *measurable/available*, timeout.
- [ ] **P1.18 `LIFT_CLEAR` pose:** direction, lift distance [m], cable exit/sag check; clamp cannot catch cradle.
- [ ] **P1.19 Repeatability dataset:** ≥20 complete fixed-fixture pickups; record `T_EP` variation, jaw position, slip events, failed grasps, mechanical wear; if `T_EP` cannot be observed directly, define and record a physical measurement fixture/test.
- [ ] **P1.20 Failure-handling:** no-object detected, wrong housing orientation, loose grip, plug slip, cable tangling → aborted/regrasp with human confirmation.
- [ ] **P2.02 Isaac gripper match:** finger geometry, joints, jaw range, closure time, mass/inertia, housing friction/contact properties; if model uses fixed plug joint, flag it as a simplification and randomize measured pose offset.

**Grasp interface contract:** `go_pregrasp() -> close_housing() -> lift_clear() -> verify_grasp() -> {success,abort}`. No general grasp detector/model needed while cradle pose is fixed.

## 5. Approach and insertion waypoints / offsets (most important requested parameters)

Record **each waypoint as a 6D pose relative to a named frame**, not only as a one-dimensional distance. Define safe approach/retraction paths, robot joint configuration and cable envelope. Provide a CAD sketch with dimensions and the selected sign of the insertion axis.

| ID / primitive | Required measurement | Units/frame | Priority | Provided? |
|---|---|---|---|---|
| `HOME` | safe known robot joint configuration | 6 joints [rad] | P0 | [ ] |
| `PRE_GRASP` | pose before jaw entry and safe approach corridor | `T_BGraspPre` [m, quat] | P1 | [ ] |
| `GRASP` | tool pose at housing closing | `T_BG` [m, quat] | P1 | [ ] |
| `LIFT_CLEAR` | unobstructed clearance pose and lift vector | B/R frame [m] | P1 | [ ] |
| `INSPECTION` | non-contact camera view seeing board markers | pose in B [m, quat] | P1 | [ ] |
| `SERVO_START` | max safe offset/standoff from board for IBVS to begin | socket frame [m, rad] | P1 | [ ] |
| `SERVO_GOAL` | target EIH image/pose at aligned **pre-contact** standoff | `s*`, `T_SE`, [px,m,rad] | P1 | [ ] |
| `SOCKET_ENTRY` | mouth center and actual axis | `T_BS`, [m, quat] | P1 | [ ] |
| `PLUG_TIP` | mating feature on plug, not gripper flange | `T_EP`, [m, quat] | P1 | [ ] |
| `FIRST_CONTACT` | expected plug-tip position at first socket lip contact; measured conservative boundary | in S [m] | P1 | [ ] |
| `FULL_SEAT` | correct final plug depth/orientation and independent sensor activation | in S [m,rad] | P1 | [ ] |
| `RETRACT_STANDOFF` | reverse path and clearance to reset a failed attempt | in S [m] | P1 | [ ] |
| `SEARCH_CENTER` | nominal alignment center; uncertainty/covariance | in S [m,rad] | P1 | [ ] |
| `SEARCH_STEP_X/Y` | bounded grid offset derived from clearance and calibration error | in S [m] | P1 | [ ] |
| `SEARCH_STEP_YAW` | legal angular candidate increment | around +z_S [rad] | P1 | [ ] |
| `MAX_SEARCH_X/Y/YAW` | maximum tested safe lateral/rotational correction | in S [m,rad] | P1 | [ ] |
| `MAX_AXIAL_TRAVEL` | maximum allowed guarded penetration, **not** beyond known seat geometry | along z_S [m] | P1 | [ ] |
| `MAX_ATTEMPTS` | retry budget and timeout | count / s | P1 | [ ] |
| `CABLE_CLEARANCE` | envelope that must not snag board, tool, cradle | geometry [m] | P1 | [ ] |
| `RECOVERY_PATH` | verified path after jam/failed insertion (or human-reset criterion) | waypoint array [B/S] | P1 | [ ] |

**Important distinctions to document:**

- `socket_entry_to_full_seat` is **not** the same as robot tool travel if `T_EP` changes, grip slips, or fixture flexes.
- `servo_standoff` is chosen by camera visibility, focus/range and physical collision clearance, **not** by an arbitrary default distance.
- `search step` must be much smaller than the validated safe free-play envelope; however no universally safe numeric value exists for a keyed connector. Record clearance and test incrementally in the actual part.
- `yaw` is around the *socket insertion axis*. It is not necessarily robot wrist joint 6.
- Spiral/raster offsets must not turn into uncontrolled lateral sliding at the face of the socket; retract before each candidate if contact sensing/compliance is insufficient.

### Explicit waypoint measurement worksheet (fill in)

```text
SOCKET_ID: TBD                 SETUP_REVISION: TBD
Insertion axis +z_S: [sign/direction description] TBD
Socket mouth frame T_BS:       xyz=[TBD,TBD,TBD] m; q_xyzw=[TBD,TBD,TBD,TBD]
Plug frame T_EP:               xyz=[TBD,TBD,TBD] m; q_xyzw=[TBD,TBD,TBD,TBD]
Servo goal TCP rel to socket:  xyz=[TBD,TBD,TBD] m; q_xyzw=[TBD,TBD,TBD,TBD]
Expected first contact depth:  TBD m
Confirmed full seat depth:    TBD m
Safe retract standoff:        TBD m
Search dx / dy / dyaw:        TBD m / TBD m / TBD rad
Search max abs x/y/yaw:       TBD m / TBD m / TBD rad
Max guarded axial travel:     TBD m
Max retries / phase timeout:  TBD / TBD s
Cable/fixture intrusion envelope: TBD (CAD or measured bounding shape)
Approved by / date:           TBD
```

## 6. Controller primitives and runtime limits

- [ ] **P0.16 ROS controller state:** actual available `joint_trajectory_controller`, `twist_controller`, gripper controller, their active/inactive states and switching service; check names against current launch, not generic examples.
- [ ] **P1.21 Command frame/unit:** document exact ROS `Twist` semantics, angular and linear axis order, publisher rate, expected watchdog behavior; test stop on controller cancellation/connection loss.
- [ ] **P1.22 Maximum free-space speed/acceleration/jerk:** axis-specific allowed bounds from manufacturer + supervised validation and safe workspace; values `TBD`. Reduce further near socket.
- [ ] **P1.23 Maximum near-contact speed:** axial and lateral and yaw separately, measured/approved values `TBD`; no default invented.
- [ ] **P1.24 End-effector position/orientation and joint limits:** safe geofenced workspace, singularity handling, collision margins and reachable candidate offsets.
- [ ] **P1.25 IBVS gains `lambda`, damping, update period, max pixel error, feature confidence, convergence persistence:** tuned free-space, documented and bounded; tune at safe standoff only.
- [ ] **P1.26 Missing-frame and stale-state timeouts:** camera and joint state timestamp thresholds; zero/stop behavior.
- [ ] **P1.27 Search candidate ordering:** offset grid (0, ±dx, ±dy ...), yaw variations, maximum attempts and total allowed time, collision checked per candidate.
- [ ] **P1.28 Axial step length and per-step feedback checks:** only use after measuring insertion depth and independent sensor, never command arbitrary continuous push.
- [ ] **P1.29 `STOP/ABORT` and `RETRACT` implementation:** emergency stop access, stopping distance observed, controller recovery procedure and human reset condition.
- [ ] **P2.03 Sim/real actuator action equivalence:** action scaling, command delays, saturation, period, control mode, observation normalization and sign.
- [ ] **P2.04 RL residual action limits:** `delta_x/y/z/yaw`, rate and cumulative offset cap, robot-side safety filter; all approved from same mechanical envelope.

**Required atomic motion primitives** (implement as independently testable operations; report success/status/timeout and log all commands):

| Primitive API | Precondition | Effect | Postcondition / failure |
|---|---|---|---|
| `home()` | unoccupied safe zone | move to known robot joint pose | feedback within tolerance |
| `open_gripper()` | no collision risk | command jaw open | jaw state verified or timeout |
| `move_pregrasp()` | camera/robot valid | move to cradle approach | pose bounds verified |
| `grasp_housing()` | plug in cradle | close on housing faces | slip/closure test |
| `lift_clear()` | grasp verified | move away from cradle/cable | clearance envelope verified |
| `move_inspection()` | plug lifted | move to camera inspection pose | board features observed |
| `servo_align()` | features & depth valid, safe standoff | iterative image-space correction | image error threshold or abort |
| `short_axial_approach()` | pre-contact guard OK | one bounded axial step | measured progress + safe state |
| `retract()` | recovery path safe | back away to clearance | confirmed standoff or manual stop |
| `apply_search_offset()` | retracted | bounded x/y/yaw correction | collision-checked target |
| `verify_seated()` | sensor alive | physical success query | `YES / NO / UNKNOWN` |
| `abort_and_log()` | any fault | stop command, record cause, park if safe | no further automatic contact |

**Gate:** no RL policy or IBVS controller may bypass these primitives' hard safety checks.

## 7. Contact, independent success and supervision (P1 contact gate)

- [ ] **P1.30 Socket seating sensor:** preferred low-voltage continuity (isolated) or mechanical microswitch/limit sensor; provide exact wiring, voltage, GPIO/serial/ROS readout, sample rate and expected response.
- [ ] **P1.31 Positive-control validation:** manually fully seat plug (robot disconnected/isolated as appropriate); show sensor registers `YES` and remains stable for an agreed dwell.
- [ ] **P1.32 Negative-control validation:** empty socket, wrong plug orientation, shallow partial seat and blocked entry must **not** trigger success; record false positive/negative cases.
- [ ] **P1.33 Contact detection or safe progression:** with no F/T, the actual procedure can only be guarded by small motion, workspace bounds, motor/encoder feedback validity and independent result (these do **not** measure contact force). Explicitly document what cannot be observed.
- [ ] **P1.34 Optional load cell / F/T:** if available, axis, calibration curve, sample rate, safe load range, force threshold and reaction delay **measured**, not inferred from joint current.
- [ ] **P1.35 E-stop/supervisor:** physical stop reachable and tested, clear line of sight, slow-speed mode, approved operator present during contact, circuit electrical safety and cable snag risk accounted for.
- [ ] **P1.36 Failure reset plan:** who/how returns fixture/plug to safe starting condition after jam/slip/part wear; replacement connector available.
- [ ] **P1.37 Minimum contact checklist signed:** acceptable geometries, protected region, camera visibility, validated retraction, no unresolved controller faults.

**Absolute contact NO-GO:** no validated independently observed seating, unstable controller/watchdog, uncertain insertion axis, invalid cavity geometry, unrestrained socket, excessive or unknown collision envelope, or no person able to stop first-contact tests.

## 8. P2: Simulation calibration and RL-specific requirements

- [ ] **P2.05 Versioned USD from real Gen3 Lite URDF/Xacro**, imported with correct collision links, actuation modes, joint gains/limits, tool and camera frame; record source hash.
- [ ] **P2.06 Collision geometry quality:** correct keyed plug and **open** socket cavity, body mass/inertia and non-penetrating contact; test nominal insertion, obvious misalignment and blocked cavity.
- [ ] **P2.07 Contact model:** friction/restitution, gravity, contact offsets/solver substeps/time step, fixture stiffness, jaw grip/slip approximation, cable disturbance; every value source and uncertainty tagged.
- [ ] **P2.08 Camera noise model:** real-versus-sim sample paired images, detector jitter distribution, depth dropout and near-range invalidity, variable exposure/light, timestamp jitter.
- [ ] **P2.09 Initial condition distributions:** measured ranges for `T_EP` repeat grasp, socket pose uncertainty, starting offset, camera calibration residuals; pre-register train/test bounds separately.
- [ ] **P2.10 Same actor observation fields:** camera marker pixels/validity, depth valid summaries, q/dq/EE feedback, gripper state, previous action, history if needed; no ground-truth sim contact wrench or exact socket/plug pose in actor unless physical equivalent exists.
- [ ] **P2.11 Action scaling and delay:** incremental x/y/z/yaw max and nominal policy period; mimic real saturation and interpolation; explicit low-level guard.
- [ ] **P2.12 Geometric success oracle:** full plug engagement, positive final seating depth, angular/lateral tolerance, stable dwell, no invalid penetration/force; unit test positive/negative cases before reward training.
- [ ] **P2.13 RL reset and curriculum:** fixed `PRE_INSERT` starts, phase C0–C5, seed assignment, promotion thresholds, success/reward variant (sparse vs shaped warm-up), episode max duration.
- [ ] **P2.14 Isaac workstation compatibility and camera throughput:** RTX, driver, VRAM/RAM, Isaac Sim/Lab pinned versions; measured 1/8/16/32/... env benchmarks, no magic assumption about parallel RGB-D capacity.
- [ ] **P2.15 Deployment safety adapter:** verify normalization, input-order, action semantics with offline real logs; shadow mode without contact; rollback/checkpoint procedure.
- [ ] **P2.16 Held-out evaluation cases:** nominal, bounded x/y offsets, bounded yaw, repeat-grasp error, occlusions, camera noise, friction/cable drag variations; same cases for scripted and RL baselines.

## 9. What specifically to request from the hardware team (copy/paste handoff)

**First batch: enough to start**

- [ ] Front/side photos of plug, socket, mounted board and cradle with ruler/caliper reference.
- [ ] Plug/socket manufacturer/part ID, STEP/STL/CAD/drawings if available; measured opening, plug head, keyed geometry and insertion depth.
- [ ] Robot and gripper model, current ROS 2 controller list, actual `/joint_states` names, robot Xacro/URDF location.
- [ ] RealSense exact model and RGB/depth info; camera bracket photo, camera link attachment and whether D405 is operational.
- [ ] Socket and pickup approximate poses relative to robot base, with explicit frame axes (coarse only until calibrated).
- [ ] GPU machine specs for Isaac Sim training and access plan.

**Second batch: required before contact**

- [ ] Four marker metric positions and identities, socket-to-marker offset, calibration captures from 3–5 viewpoints.
- [ ] Measured `T_EC`, `T_EP` (repeat grasp), `T_MS`, and robot-base cradle/socket positions.
- [ ] Named `HOME`, `PRE_GRASP`, `GRASP`, `LIFT_CLEAR`, `INSPECTION`, `SERVO_GOAL`, `RETRACT` poses.
- [ ] Actual first-contact distance, full seating depth, legal search x/y/yaw ranges, safe offset step based on connector clearance, axis-specific speed caps and max guarded travel.
- [ ] Independently observed seating circuit/switch and both positive/negative test evidence, E-stop/observer and retraction plan.

**Third batch: needed to defend sim-to-real realism**

- [ ] Connector/cavity collision geometry and photo/CAD correspondence.
- [ ] Joint response/logging, gripper slip/error, cable drag estimate, latency distribution.
- [ ] Real/sim camera comparison at multiple poses; actual camera depth noise and feature jitter statistics.
- [ ] Solver/contact sanity test evidence and held-out initial-condition set.

## 10. Example machine-readable parameter file (populate; never execute with null limits)

Save as `configs/setup_r0.yaml`. This is a **schema example**, not a usable live-robot configuration.

```yaml
schema_version: 1
setup_revision: setup_r0
status: UNCALIBRATED  # prevents physical execution
connector:
  id: null
  plug_cad_file: null
  socket_cad_file: null
  plug_housing_xyz_m: null
  socket_opening_xyz_m: null
  clearance_xy_m: null
  chamfer_geometry_file: null
  insertion_depth_m: null
  verified_seated_depth_m: null
robot:
  model: gen3_lite
  dof_arm: 6
  gripper: gen3_lite_2f
  urdf_source: null
  arm_joint_names: null
  ros_joint_controller: null
  ros_twist_controller: null
  command_frame_id: null
  joint_limits_rad: null
camera:
  model: null  # confirm D405
  rgb_topic: null
  depth_topic: null
  optical_frame: null
  rgb_width_px: null
  rgb_height_px: null
  fps: null
  K: null  # 3x3, row-major
  distortion: null
  depth_scale_m_per_unit: null
  T_EC: {xyz_m: null, quat_xyzw: null, uncertainty: null}
markers:
  family: null # color_blob / ArUco / AprilTag
  positions_M_m: null  # id -> [x,y,z]
  target_pixels_uv: null # id -> [u,v] at correct standoff
  feature_confidence_min: null
  feature_max_missing_frames: null
frames:
  socket_insertion_axis_in_S: [0, 0, 1] # CONVENTION ONLY; must validate physical direction
  T_MS: {xyz_m: null, quat_xyzw: null}
  T_BR: {xyz_m: null, quat_xyzw: null}
  T_EP: {xyz_m: null, quat_xyzw: null, covariance: null}
  T_BS_check: {xyz_m: null, quat_xyzw: null}
waypoints:
  home_joint_rad: null
  pre_grasp_B: null
  grasp_B: null
  lift_clear_B: null
  inspection_B: null
  servo_start_B: null
  servo_goal_B: null
  first_contact_S: null
  full_seat_S: null
  retract_standoff_S: null
search:
  step_x_m: null
  step_y_m: null
  step_yaw_rad: null
  max_abs_x_m: null
  max_abs_y_m: null
  max_abs_yaw_rad: null
  max_attempts: null
  max_total_time_s: null
motion_limits:
  max_free_linear_m_s: null
  max_free_angular_rad_s: null
  max_contact_axial_m_s: null
  max_contact_lateral_m_s: null
  max_acceleration_m_s2: null
  max_axial_travel_m: null
  watchdog_timeout_s: null
  camera_stale_timeout_s: null
verification:
  type: null  # continuity / seating_switch / validated_other
  ros_topic_or_device: null
  success_value: null
  stable_dwell_s: null
  positive_control_passed: false
  negative_control_passed: false
safety:
  supervisor_approved: false
  e_stop_tested: false
  clear_retract_path_tested: false
  connector_safe_voltage_verified: false
sim:
  isaac_sim_version: null
  isaac_lab_version: null
  gpu_model: null
  gpu_vram_gb: null
  actuator_delay_s: null
  contact_parameter_source: null
  camera_noise_profile_path: null
  policy_dt_s: null
  num_envs_benchmarked: null
  heldout_eval_seed_file: null
```

**Validation code MUST reject** `status: UNCALIBRATED`, null motion limits/waypoints, stale calibration hashes and false safety flags for any real-robot contact command. Simulation may use marked estimated values for prototyping, but those estimates must remain distinguishable in logs and cannot certify the digital twin.

## 11. Final pre-experiment gate checklist

### Begin A non-contact visual servo
- [ ] P0 identification, real robot controller, robot frame mapping complete.
- [ ] EIH camera stable; markers readable; `K`, `T_EC`, `T_MS` validated at real standoff.
- [ ] Desired image reference acquired; wrong-axis/sign velocity experiment passed in clear free space.
- [ ] Bounded speed/workspace/abort and emergency-stop safeguards tested.

### Begin A physical insertion / search
- [ ] All applicable P1 complete and signed by supervisor.
- [ ] Grasp reproducible, plug seating geometry and full mechanical depth measured.
- [ ] Positive and negative seating detector tests passed.
- [ ] Independent progress and fail-safe guarded axial step/retraction validated.
- [ ] Search envelope and each offset tested for collision-free **retracted** movements.

### Begin B high-fidelity parallel training
- [ ] P0 geometry/model and P2 simulation toolchain/RTX availability documented.
- [ ] Collision cavity physically plausible; contact, success oracle and reset tests pass.
- [ ] At least five real/sim camera-view comparisons exist, with measured errors and known limitations.
- [ ] Actor features are identical in type/order/units to planned deployment interface.
- [ ] Single environment validated, then each new parallel-env count benchmarked.

### Claim B real sim-to-real success
- [ ] Exported policy frozen; offline real log replay and shadow tests passed.
- [ ] P1 real contact gates complete, independent physical seating working.
- [ ] Controlled supervised physical trials performed, failures not discarded, all artifacts archived.
- [ ] Real-vs-sim success gap and constraints quantified.

## 12. References and verification

- Uploaded literature master file: `insertion_papers_master_review(1).md`, especially its Task/Method selection Section C; distinguishes camera-only alignment from force-enabled contact search and warns about plug-to-gripper errors.
- Official Kortex ROS 2 interface: https://github.com/Kinovarobotics/ros2_kortex
- Official Intel D405 range/specifications: https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html
- Isaac Sim URDF importing and model setup: https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html
- Isaac Lab camera rendering/limitations: https://docs.nvidia.com/isaaclab/concepts/sensors/camera
- Isaac Lab installation and GPU requirements: https://docs.nvidia.com/isaaclab/isaac-lab/installation

**Purpose of this checklist:** make every assumption observable, measurable, versioned and replaceable while keeping the overall robot insertion objective stable.
