#!/usr/bin/env python3
"""Automated hand-eye calibration: the arm visits poses around an ArUco marker and captures it, safety checks on every move.

WHAT IT DOES
  Finds the marker from the home pose, plans ~20 camera poses on a dome around it (distance 0.18-0.24 m, tilt 0-45 deg, several azimuths and rolls), keeps only the ones that pass
  every safety check, visits them slowly, takes a stationary burst of 8 frames at each (method 2's burst_capture: subpixel corners, robot TF at the image time, reprojection <= 1 px),
  solves the hand-eye (Park-Martin, method 2's calib_lib), checks it on held-out poses (every 4th sample, not used in the fit) against the CURRENT hand-eye, and returns home.
  It never edits any config. It prints the two lines to paste, and you decide.

BEFORE YOU RUN (every time)
  1. Stack up, nothing else moving the arm:  ~/kinova/work/tools/run_stack_method2.sh rviz:=false     (arm link, driver, D405 with enable_sync, MoveIt, method 2's grasp node in dry run)
  2. Marker: ArUco DICT_4X4_50, id 0, BLACK square 100 mm (ruler it; value is aruco.marker_size_m in method 2's Code/Calibration/calibration.yaml), flat on the table inside the
     camera view from the home pose, about x 0.3-0.45 m in front of the base. Do NOT move it during the run. Clear the area around it; empty gripper (no plug).
  3. Camera on USB 3 (rs-enumerate-devices | grep "Usb Type" must say 3.x).
  4. Arm at project home (within 2 deg), else the script refuses:
       cd ~/kinova/ins/method2_src/Code/Grasp && python3 ros2_iface/move_home_once.py --execute
  5. In the terminal you run this from:  source ~/kinova/ins/repo/Code/Grasp/ros_env.sh

RUN
  python3 ~/kinova/work/tools/auto_hand_eye.py             PLAN ONLY, nothing moves. Prints the marker position and a table of every pose: joints, largest joint step, lowest clearance, OK or SKIP + reason.
                                                         Need at least 14 OK poses (it refuses to execute with fewer). Skipped poses (no IK in range, too low, outside the box) are normal.
  python3 ~/kinova/work/tools/auto_hand_eye.py --execute  MOVES THE ARM (about 2-3 minutes). E-stop in reach, keep hands and cables out of the way. Ctrl-C stops it and still saves what it has.
  --out DIR                                              choose the output folder (default ~/kinova/work/calibration/<time>_marker_hand_eye)

SAFETY (all values are constants near the top of this file)
  - every target joint inside the robot-description limit minus MARGIN (6 deg); no pose more than 120 deg from home on any joint, no move longer than 90 deg on any joint
  - along every move (straight joint interpolation, what the controller runs) forward kinematics is checked every ~3 deg: fingertip >= TCP_ABOVE (50 mm) and camera >= CAM_ABOVE (80 mm)
    above the marker plane, tool and camera inside BOX_X/BOX_Y, and outside the base keep-out zone (BASE_ZONE)
  - average speed SPEED = 8 deg/s (quintic profile, zero speed at both ends), at least 2.5 s per move, 1.5 s settle before each capture
  - before every move: arm fault flag must be false. After every move: every joint within 1.5 deg of its target, else the run stops (a faulted arm ignores commands but the controller still says success)
  - no collision model of the table fixtures or the cable: clearance is only measured against the marker plane. Keep the workspace clear.
  - DESIGN (below) lists the poses: (distance m, tilt from vertical deg, azimuth deg from base +x, roll deg). The image keeps a fixed orientation relative to the base at every tilt (no wrist spin).
    Azimuths 150-240 deg (camera on the robot-base side) have no reachable IK and are skipped.

OUTPUT (folder ~/kinova/work/calibration/<time>_marker_hand_eye)
  plan.json (every pose and why it was kept or skipped), samples.json (accepted samples), solve.json (new hand-eye, spreads, held-out errors), console.log, attempts/attempt_NNN/ (frames, corners, quality per burst)

READING THE RESULT
  "marker spread" = where the FIXED marker lands in the base frame for every sample using hand-eye X: 0 for a perfect calibration. Compare NEW and CURRENT on the same samples and on the held-out poses.
  Apply the printed hand_eye_xyz_m / hand_eye_quat_xyzw to Code/Grasp/config/grasp.yaml (camera:) only if the held-out error is clearly lower than the current one (the tool's own rule), then confirm with a
  scan (python3 calib_scan.py in Code/Calibration) or a CAD-fit run. The method 2 and method 1 pipelines also read the camera pose from calibration files: keep them in step.
  Straight-down (tilt 0) captures are usually rejected (planar marker pose ambiguity, reprojection > 1 px): normal.

REFERENCE RESULT 2026-11-19 (calibration/test1_marker_hand_eye): 18 poses, 16 samples, no stop. New marker spread std 3.4/3.3/1.4 mm (max 7.8) vs current 3.8/4.1/1.4 (max 8.9); held-out 4.1 vs 4.9 mm; the two hand-eyes
  differ by 11.4 mm / 2.7 deg. Not applied. The leftover error flips sign with the camera azimuth (5-8 mm at 270 and 90-120 deg): not explained by a rigid hand-eye (intrinsics, arm kinematics, depth bias untested).

NEEDS  method 2's calibration modules (burst_capture, capture_quality, calib_lib, robot_ros), default ~/kinova/ins/method2_src/Code/Calibration, or set CAL_DIR=/other/path. MoveIt /compute_ik and /compute_fk running.
"""
import argparse
import json
import os
import pathlib
import sys
import time

import numpy as np
import rclpy
import yaml
from example_interfaces.msg import Bool
from moveit_msgs.srv import GetPositionFK
from scipy.spatial.transform import Rotation as Rot

CAL = pathlib.Path(os.environ.get("CAL_DIR", pathlib.Path.home() / "kinova/ins/method2_src/Code/Calibration"))
sys.path.insert(0, str(CAL))
import calib_lib as cl  # noqa: E402
from burst_capture import capture, setup  # noqa: E402
from capture_quality import corners_refined, square_pose  # noqa: E402
from robot_ros import BASE, JOINTS, TOOL, Robot, matrix  # noqa: E402

LIMITS = np.array([153.6, 149.5, 149.5, 149.0, 145.0, 149.0])     # robot description, deg
MARGIN = 6.0                                                        # stay this far inside
HOME = (np.array([3.0266, 356.8712, 93.4037, 271.4634, 272.3364, 277.3673]) + 180) % 360 - 180
SPEED = 8.0                                                         # deg/s, average of the largest joint
MIN_MOVE_S, SETTLE_S, REACH_TOL = 2.5, 1.5, 1.5
TCP_ABOVE, CAM_ABOVE = 0.05, 0.08                                   # fingertip / camera height over the marker plane, m
BOX_X, BOX_Y, BASE_ZONE = (0.12, 0.55), (-0.30, 0.30), 0.20        # tool and camera must stay in this box and out of the base zone
TCP_IN_TOOL = np.array([0.0, 0.0, 0.035])
# distance (m), tilt from vertical (deg), azimuth (deg), roll about the optical axis (deg)
DESIGN = [(0.21, 0, 0, -30), (0.21, 0, 0, 0), (0.21, 0, 0, 30),
          (0.18, 20, 0, 0), (0.24, 20, 90, 0), (0.18, 20, 180, 0), (0.24, 20, 270, 0),
          (0.20, 35, 30, 0), (0.24, 35, 90, 15), (0.18, 35, 150, 0), (0.22, 35, 210, -15), (0.18, 35, 270, 0), (0.24, 35, 330, 0),
          (0.22, 45, 0, 0), (0.18, 45, 60, 0), (0.24, 45, 120, -15), (0.22, 45, 180, 0), (0.18, 45, 240, 15), (0.24, 45, 300, 0),
          (0.22, 30, 0, 20), (0.20, 40, 30, -20), (0.24, 40, 60, 0), (0.20, 30, 120, 10), (0.22, 40, 90, 0), (0.22, 30, 270, -15), (0.18, 25, 45, 0), (0.24, 40, 270, 15)]

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--execute", action="store_true")
ap.add_argument("--out", type=pathlib.Path)
args = ap.parse_args()

cfg = yaml.safe_load(open(CAL / "calibration.yaml"))
aruco = cfg["aruco"]
X0 = cl.to_T(cfg["camera"]["hand_eye_xyz_m"], cfg["camera"]["hand_eye_quat_xyzw"])
wrap = lambda q: (np.asarray(q, float) + 180) % 360 - 180

rclpy.init()
robot = Robot("auto_hand_eye", background=False)
if not robot.wait_ready(10):
    sys.exit("no driver")
sub = setup(robot)
fault = {"v": False}
robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("v", bool(m.data)), 10)
fk_cli = robot.create_client(GetPositionFK, "/compute_fk")
robot.spin_until(lambda: hasattr(robot, "color_info"), 5)
if not fk_cli.wait_for_service(10):
    sys.exit("no /compute_fk (MoveIt up?)")


def fk(q_deg):
    req = GetPositionFK.Request()
    req.header.frame_id, req.fk_link_names = BASE, [TOOL]
    req.robot_state.joint_state.name = JOINTS
    req.robot_state.joint_state.position = [float(v) for v in np.radians(q_deg)]
    fut = fk_cli.call_async(req)
    robot.spin_until(fut.done, 5)
    res = fut.result()
    if res is None or res.error_code.val != 1 or not res.pose_stamped:
        raise RuntimeError("FK failed")
    p = res.pose_stamped[0].pose
    return matrix([p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def limit_error(q):
    q = wrap(q)
    bad = [i for i in range(6) if abs(q[i]) > LIMITS[i] - MARGIN]
    return "; ".join("joint %d at %.1f (usable +-%.1f)" % (i + 1, q[i], LIMITS[i] - MARGIN) for i in bad) or None


def clearance_error(T_tool, z_marker):
    tcp, cam = (T_tool @ np.r_[TCP_IN_TOOL, 1])[:3], (T_tool @ X0)[:3, 3]
    for name, p, above in (("fingertip", tcp, TCP_ABOVE), ("camera", cam, CAM_ABOVE)):
        if p[2] < z_marker + above:
            return "%s %.0f mm above the marker plane (need %.0f)" % (name, (p[2] - z_marker) * 1e3, above * 1e3), p[2] - z_marker
        if not (BOX_X[0] <= p[0] <= BOX_X[1] and BOX_Y[0] <= p[1] <= BOX_Y[1]):
            return "%s outside the workspace box at x %.2f y %.2f" % (name, p[0], p[1]), p[2] - z_marker
        if np.hypot(p[0] + 0.05, p[1]) < BASE_ZONE:
            return "%s inside the base zone" % name, p[2] - z_marker
    return None, min(tcp[2], cam[2]) - z_marker


def path_report(q_from, q_to, z_marker):
    """Straight joint interpolation (what the controller runs): limits and clearance at every ~3 deg."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    step = float(np.abs(q_to - q_from).max())
    n = max(2, int(np.ceil(step / 3.0)) + 1)
    worst = np.inf
    for s in np.linspace(0, 1, n):
        q = q_from + (q_to - q_from) * s
        err = limit_error(q)
        if err:
            return False, err, step, worst
        err, clear = clearance_error(fk(q), z_marker)
        worst = min(worst, clear)
        if err:
            return False, "at %.0f%% of the move: %s" % (100 * s, err), step, worst
    return True, "ok", step, worst


def look_at(c, n, d, theta, phi, roll):
    e1 = np.array([1.0, 0, 0]) - n * n[0]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    t, p = np.radians(theta), np.radians(phi)
    pos = c + d * (np.cos(t) * n + np.sin(t) * (np.cos(p) * e1 + np.sin(p) * e2))
    z = (c - pos) / np.linalg.norm(c - pos)
    y = -(e1 - (e1 @ z) * z)                      # image 'down' keeps its direction relative to the base at every tilt and azimuth: no wrist spin
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    Rc = np.column_stack([x, y, z])
    Rc = Rc @ Rot.from_euler("z", roll, degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rc, pos
    return T


# ---- where is the marker (current hand-eye, good to a centimetre) ----
color, _, K = robot.grab()
dist = np.array(robot.color_info.d)
uv = corners_refined(color, aruco["dictionary"], aruco["marker_id"])
T_cm, _ = square_pose(uv, K, dist, aruco["marker_size_m"])
T_bm = robot.tool_matrix() @ X0 @ T_cm
c, n = T_bm[:3, 3], T_bm[:3, 2] * np.sign(T_bm[2, 2])
print("marker centre in base (m): %s | plane normal %s (%.1f deg from vertical)" % (np.round(c, 3), np.round(n, 3), np.degrees(np.arccos(n[2]))), flush=True)
q_home_now = np.degrees(robot.q)
if np.abs(wrap(q_home_now) - HOME).max() > 2:
    sys.exit("arm is %.1f deg from project home: move it home first" % np.abs(wrap(q_home_now) - HOME).max())

# ---- plan ----
plan, rows, q_prev, T_all = [], [], wrap(q_home_now), []
remaining = [(k, par, look_at(c, n, par[0], par[1], par[2], par[3]) @ np.linalg.inv(X0)) for k, par in enumerate(DESIGN)]
MAX_STEP, MAX_FROM_HOME = 90.0, 120.0
while remaining:
    best = None
    for k, par, T_tool in remaining:
        for seed in (q_prev, HOME):
            q = robot.ik(T_tool, seed_deg=seed)
            if q is None:
                continue
            q = wrap(q)
            if limit_error(q) or np.abs(q - HOME).max() > MAX_FROM_HOME:
                continue
            step = np.abs(q - q_prev).max()
            if step <= MAX_STEP and (best is None or step < best[0]):
                best = (step, k, par, T_tool, q)
    if best is None:
        break
    _, k, par, T_tool, q = best
    remaining = [x for x in remaining if x[0] != k]
    ok, why, step, clear = path_report(q_prev, q, c[2])
    rows.append(dict(index=k, design=par, joints_deg=np.round(q, 1).tolist(), step_deg=round(step, 1), min_clearance_mm=round(clear * 1e3), ok=ok, why=why))
    if ok:
        plan.append(dict(index=k, design=par, q_deg=q.tolist(), T_tool=T_tool.tolist()))
        q_prev = q
for k, par, T_tool in remaining:
    rows.append(dict(index=k, design=par, joints_deg=None, ok=False, why="no IK solution within the limits, 90 deg of the previous pose and 120 deg of home"))
print("\n pose  (dist, tilt, azim, roll)        joints (deg)                          step  clearance  result")
for r in rows:
    print(" %3d  %-24s %-42s %5s %8s   %s" % (r["index"], r["design"], r["joints_deg"], r.get("step_deg", "-"), r.get("min_clearance_mm", "-"), "OK" if r["ok"] else "SKIP: " + r["why"]), flush=True)
Rs = [Rot.from_matrix(np.array(p["T_tool"])[:3, :3]) for p in plan]
spread = max((a * b.inv()).magnitude() for a in Rs for b in Rs) if Rs else 0
print("\n%d of %d poses pass every check; widest tool rotation between two poses %.0f deg" % (len(plan), len(DESIGN), np.degrees(spread)), flush=True)
if not args.execute:
    print("PLAN ONLY, nothing moved.")
    robot.destroy_node(); rclpy.shutdown(); sys.exit(0)
if len(plan) < 14:
    sys.exit("fewer than 14 safe poses: not executing")

# ---- execute ----
run = args.out or (pathlib.Path.home() / "kinova/work/calibration" / time.strftime("%Y%m%d_%H%M%S_marker_hand_eye"))
(run / "attempts").mkdir(parents=True)
(run / "plan.json").write_text(json.dumps(dict(marker_centre_m=c.tolist(), marker_normal=n.tolist(), rows=rows, limits=LIMITS.tolist(), margin_deg=MARGIN, speed_deg_s=SPEED), indent=1))
log = open(run / "console.log", "w")


def say(msg):
    line = "[%6.1f s] %s" % (time.time() - t0, msg)
    print(line, flush=True)
    log.write(line + "\n")
    log.flush()


t0 = time.time()
T_bt, T_cm_list, meta, attempts = [], [], [], 0


def safe_move(q_to, label):
    robot.spin_until(lambda: False, 0.3)
    q_now = wrap(np.degrees(robot.q))
    if fault["v"]:
        raise RuntimeError("arm in fault")
    ok, why, step, clear = path_report(q_now, q_to, c[2])
    if not ok:
        raise RuntimeError("path to %s rejected: %s" % (label, why))
    robot.move_joints(q_to, max(MIN_MOVE_S, step / SPEED))
    time.sleep(SETTLE_S)
    robot.spin_until(lambda: False, 0.3)
    miss = np.abs(wrap(np.degrees(robot.q)) - wrap(q_to)).max()
    if fault["v"] or miss > REACH_TOL:
        raise RuntimeError("stopped short at %s (fault %s, %.1f deg off)" % (label, fault["v"], miss))


try:
    for i, p in enumerate(plan):
        q = np.array(p["q_deg"])
        say("pose %d/%d design %s" % (i + 1, len(plan), p["design"]))
        safe_move(q, "pose %d" % (i + 1))
        for tries in range(2):
            attempts += 1
            try:
                BT, CM, quality = capture(robot, aruco, run / "attempts" / ("attempt_%03d" % attempts))
                T_bt.append(BT); T_cm_list.append(CM)
                meta.append(dict(attempt=attempts, pose=i, design=p["design"], quality=quality))
                say("  sample %d: reprojection %.2f px, corner std %.2f px, marker at %.0f mm" % (len(T_bt), quality["reprojection_rms_px"], quality["corner_std_max_px"], CM[2, 3] * 1e3))
                break
            except Exception as e:
                say("  capture rejected: %s" % e)
                time.sleep(1.0)
    say("back to home")
    safe_move(wrap(HOME), "home")
except BaseException as e:
    say("STOPPED: %r" % (e,))
    try:
        robot.stop()
    except Exception:
        pass
finally:
    (run / "samples.json").write_text(json.dumps(dict(aruco=aruco, captures=meta, T_base_tool=[t.tolist() for t in T_bt], T_cam_marker=[t.tolist() for t in T_cm_list]), indent=1))

# ---- solve and validate ----
n_s = len(T_bt)
say("%d samples" % n_s)
if n_s >= 10:
    val = list(range(3, n_s, 4))
    train = [i for i in range(n_s) if i not in val]
    sel = lambda ids: ([T_bt[i] for i in ids], [T_cm_list[i] for i in ids])
    marker_in_base = lambda X, ids: np.array([(T_bt[i] @ X @ T_cm_list[i])[:3, 3] for i in ids])
    Xt = cl.hand_eye(*sel(train))
    dev = np.linalg.norm(marker_in_base(Xt, train) - marker_in_base(Xt, train).mean(0), axis=1) * 1e3
    keep = [i for i, d_ in zip(train, dev) if d_ <= max(3.0, 2.5 * np.median(dev))]
    if len(keep) < len(train):
        say("dropped %d training outliers (marker %.1f mm from the mean): %s" % (len(train) - len(keep), dev.max(), [i for i in train if i not in keep]))
        Xt = cl.hand_eye(*sel(keep))
    mean_t = marker_in_base(Xt, keep).mean(0)
    held = np.linalg.norm(marker_in_base(Xt, val) - mean_t, axis=1) * 1e3
    held0 = np.linalg.norm(marker_in_base(X0, val) - marker_in_base(X0, keep).mean(0), axis=1) * 1e3
    Xa = cl.hand_eye(*sel(keep + val))
    res = dict(n_samples=n_s, train=keep, held_out=val,
               train_spread=cl.hand_eye_residual(*sel(keep), Xt), current_spread_on_train=cl.hand_eye_residual(*sel(keep), X0),
               held_out_error_mm=dict(new=np.round(held, 2).tolist(), current=np.round(held0, 2).tolist(), new_mean=float(held.mean()), current_mean=float(held0.mean())),
               difference_new_vs_current_mm_deg=cl.pose_difference(Xt, X0),
               frozen_train_hand_eye=dict(xyz=cl.xyz_quat(Xt)[0], quat=cl.xyz_quat(Xt)[1]), all_samples_hand_eye=dict(xyz=cl.xyz_quat(Xa)[0], quat=cl.xyz_quat(Xa)[1]),
               all_samples_spread=cl.hand_eye_residual(*sel(keep + val), Xa))
    (run / "solve.json").write_text(json.dumps(res, indent=1))
    say("NEW  (train only) marker spread: std %s mm, max %.1f mm, %.2f deg" % (np.round(res["train_spread"]["std_mm"], 2), res["train_spread"]["max_dev_mm"], res["train_spread"]["max_angle_deg"]))
    say("CURRENT hand-eye on the same samples: std %s mm, max %.1f mm" % (np.round(res["current_spread_on_train"]["std_mm"], 2), res["current_spread_on_train"]["max_dev_mm"]))
    say("HELD-OUT poses (not used in the fit): new %.1f mm mean error, current %.1f mm" % (held.mean(), held0.mean()))
    say("new vs current hand-eye differ by %.1f mm, %.2f deg" % tuple(res["difference_new_vs_current_mm_deg"]))
    say("hand_eye_xyz_m: %s" % [round(v, 7) for v in res["all_samples_hand_eye"]["xyz"]])
    say("hand_eye_quat_xyzw: %s" % [round(v, 7) for v in res["all_samples_hand_eye"]["quat"]])
else:
    say("too few samples to solve")
log.close()
robot.destroy_subscription(sub)
robot.destroy_node()
rclpy.shutdown()
