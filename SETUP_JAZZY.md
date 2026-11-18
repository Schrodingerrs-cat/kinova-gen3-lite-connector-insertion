# This laptop: ROS 2 Jazzy

This checkout is adapted for Jay's Ubuntu / ROS 2 Jazzy laptop. The shared environment loads the adjacent `kinova_ws/install/setup.bash`, the project `.venv`, ROS domain 42, localhost discovery and Cyclone DDS. It does not edit `.bashrc`.

## Start with no pipeline motion

Stop previous robot/camera launch terminals first. Connect the robot by USB and the D405 with a USB 3 data cable. From a new terminal:

```bash
cd "/home/jay-patil/Peg insertion project/kinova-gen3-lite-connector-insertion/Code/Grasp"
./run_ros.sh dry_run:=true
```

This starts one real driver/controller manager through `robot_jazzy.launch.py`, MoveIt, synchronized camera streams, the grasp node in dry-run mode, and two RViz windows. Add `rviz:=false` for terminal-only startup. Ctrl+C stops the processes owned by this launcher. The wrapper refuses to start over a detected existing stack.

In another terminal:

```bash
cd "/home/jay-patil/Peg insertion project/kinova-gen3-lite-connector-insertion/Code/Grasp"
source ros_env.sh
python3 ros2_iface/smoke_test.py
ros2 param get /grasp_node dry_run
```

The smoke test reads joint states, TF and camera frames and requests IK; it never commands motion. A printed `IK ... None` requires investigation before scanning. The upstream smoke test does not set a failing exit code for every failed check, so inspect its output.

## What changed locally

- Shared Jazzy environment and project-local Python dependencies.
- One driver/controller manager; removed the separate duplicate driver launch.
- Local copy of Kinova's Jazzy launch file selects the Kortex robot description without the extra FakeSystem in the MoveIt config wrapper.
- RealSense `enable_sync:=true`, verified necessary on this laptop earlier.
- Internal robot node ignores the launch-wide name remap, preventing duplicate grasp-node names and parameter services.
- IBVS controller switching supports Jazzy's activate/deactivate fields.
- Scan geometry checker discovers the installed Kortex package instead of assuming `/opt/ros/humble`.

## Before scan or grasp

The repository includes saved joint poses, hand-eye calibration, TCP offset, gripper geometry and disabled collision checks from the team setup. Confirm their applicability to the current mounting, table and objects. Dry-run startup is not proof that a scan trajectory is clear. Do not widen robot joint limits to force IK to accept an out-of-range measured state.

The runner's “perception only” mode still moves home, scans and moves to a close view when the node is live. Omitting `--grasp` does not mean no robot movement.

## Insertion status

`Code/IBVS/config/ibvs.yaml` intentionally remains UNCALIBRATED with null measurements. Full insertion cannot run until camera/marker calibration, socket axes, controller frame, bounds, taught waypoints, and independent seating verification are measured and checked. Software tests do not validate physical insertion. No calibration flags or motion gates were bypassed.

## Dependencies and tests

The `.venv` uses system Python/ROS packages plus `requirements-jazzy.txt`. Existing geometry tests run without the automatically loaded ROS pytest plugins:

```bash
source Code/Grasp/ros_env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest Code/Grasp/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest Code/IBVS/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest Code/Calibration/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest Code/Teleop/test_teleop.py -q
```

Run these test commands from the repository root. Runtime startup logs are under `Code/Grasp/logs/jazzy/`.

## Verification on 2026-11-12

69 existing tests passed (33 grasp, 22 IBVS, 6 calibration, 8 teleop). Live read-only checks received arm/gripper action servers, measured joints, TF, color/depth images and both planning services. The corrected launch exposes only KortexMultiInterfaceHardware, and the grasp node reports dry_run=True.

Outstanding hardware observations: the D405 negotiated USB 2 (480 Mbps) and opened 848x480 color/depth at 10 FPS. Joint 3 reported about 161 degrees, beyond the installed URDF range of +/-2.61 radians; current-pose IK returned None. The driver also logged a Kortex control-mode protocol error at activation, despite the controllers reporting active. Motion has not been validated in this setup. No robot motion or full pipeline was commanded. The IBVS configuration remains uncalibrated.

## Driver repair and verification on 2026-11-13

The current-pose IK check now passes after the arm was repositioned. The actuator control-mode protocol error was traced to the upstream driver's assumption that actuator routing IDs are `1..actuator_count`. A read-only `DeviceManager.ReadAllDevices` query identified this robot's arm actuator IDs as `1,2,3,4,5,7`; ID `6` is the gripper. The old loop sent an ActuatorConfig RPC to the gripper and skipped the last arm actuator.

The adjacent `kinova_ws/src/ros2_kortex/kortex_driver` now discovers and filters actuator device handles, validates the count and unique nonzero IDs before setting modes, and propagates mode-setting failures. The patch does not change joint indices or cyclic command order. Its regression test covers the Gen3 Lite topology, contiguous seven-actuator IDs, and malformed device lists.

The driver was rebuilt and its CTest regression passed. Live startup now logs `Set all actuators to POSITION control mode` with no control-mode protocol error. Arm/gripper controllers are active; joint feedback, camera frames and current-pose IK pass the read-only smoke test. The grasp node remains `dry_run=True`; no motion trajectory was commanded. Joint velocity control and Cartesian motion have not been exercised by this check.

Remaining separate issues: the camera still negotiates USB 2.1 and 10 FPS, and MoveIt reports no 3D sensor plugin configured for octomap updates. Full insertion remains uncalibrated. Preserve the local driver patch when updating the Kortex source checkout.
