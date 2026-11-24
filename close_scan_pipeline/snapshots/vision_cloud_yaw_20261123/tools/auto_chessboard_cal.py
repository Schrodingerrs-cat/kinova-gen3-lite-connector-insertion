#!/usr/bin/env python3
"""Automated chessboard calibration: camera intrinsics check AND hand-eye from the board, arm moves on its own with safety checks on every move.

WHAT IT DOES
  Finds the board from the home pose, plans ~20 camera poses around it (distance 0.26-0.34 m, tilt 0-40 deg, several azimuths, rolls, and aim offsets so the board also appears near
  the image edges), keeps only the poses that pass every safety check and show the WHOLE board, visits them slowly, takes a stationary burst of 8 frames at each (robot TF at the
  image time) and then:
   1. INTRINSICS: fits K (and K + k1,k2) with cv2.calibrateCamera on the training views and compares them with the factory camera_info on the held-out views (reprojection error in px).
   2. HAND-EYE: solves tool->camera from the board poses (Park-Martin, method 2's calib_lib), once with the factory intrinsics and once with the best calibrated ones, and reports the board
      position spread, the held-out error, and the difference to the CURRENT hand-eye and to the marker-based result of 2026-11-19.
  It never edits any config. It prints the lines to paste and you decide.

BOARD  8 x 6 squares of 30 mm = 7 x 5 INNER corners (PATTERN / SQUARE below). Flat on the table, fixed, never moved during the run. A 7x5 board looks the same rotated by 180 deg: the
  script resolves that for each view with the arm's own pose (needs the current hand-eye to be right to a few degrees, which it is).

BEFORE YOU RUN (every time)
  1. Stack up:  ~/kinova/work/tools/run_stack_method2.sh rviz:=false      (arm link, driver, D405 with enable_sync, MoveIt)
  2. Board in view from the home pose (check: python3 auto_chessboard_cal.py   prints whether it is found), nothing else moving the arm, empty gripper, table around it clear.
  3. Camera on USB 3.  Arm at project home:  cd ~/kinova/ins/method2_src/Code/Grasp && python3 ros2_iface/move_home_once.py --execute
  4. source ~/kinova/ins/repo/Code/Grasp/ros_env.sh

RUN
  python3 ~/kinova/work/tools/auto_chessboard_cal.py              PLAN ONLY, nothing moves: board position, a table of every pose (OK / SKIP and why). Needs >= 12 OK poses to execute.
  python3 ~/kinova/work/tools/auto_chessboard_cal.py --execute    MOVES THE ARM (about 3-4 minutes). E-stop in reach. Ctrl-C stops it and still solves what it has.
  --out DIR                                                     output folder (default ~/kinova/work/calibration/<time>_chessboard)

SAFETY (constants below; same framework as auto_hand_eye.py)
  joint targets inside the robot limit minus MARGIN (6 deg), <= 120 deg from home, <= 90 deg per move; forward kinematics checked every ~3 deg of every move: fingertip >= 50 mm and camera >= 80 mm above
  the board plane, tool and camera inside the workspace box and outside the base zone; 8 deg/s average; fault flag checked before every move and joint arrival (1.5 deg) after every move.
  No collision model of fixtures or cables: keep the workspace clear.

OUTPUT (~/kinova/work/calibration/<time>_chessboard)
  plan.json, captures.json (median corners, poses, quality per pose), frames/ (middle frame of every burst), solve.json (intrinsics and hand-eye results), console.log

READING THE RESULT
  Intrinsics: held-out reprojection RMS below ~0.5 px is good. If the factory value is about as good as the calibrated one, the factory model is fine and the azimuth-dependent error seen with the marker
  is not an intrinsics problem. Hand-eye: lower board spread / held-out error than the current hand-eye = better. Paste hand_eye_xyz_m / hand_eye_quat_xyzw into Code/Grasp/config/grasp.yaml (camera:) only if clearly
  better and confirmed on a scan or CAD-fit run. Calibrated intrinsics would have to be applied to the camera driver / pipeline too: not done by this script.

NEEDS  method 2's calibration modules (default ~/kinova/ins/method2_src/Code/Calibration, or CAL_DIR=...), MoveIt /compute_ik and /compute_fk.
"""
import argparse
import json
import os
import pathlib
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from example_interfaces.msg import Bool
from moveit_msgs.srv import GetPositionFK
from scipy.spatial.transform import Rotation as Rot

CAL = pathlib.Path(os.environ.get("CAL_DIR", pathlib.Path.home() / "kinova/ins/method2_src/Code/Calibration"))
sys.path.insert(0, str(CAL))
import calib_lib as cl  # noqa: E402
from burst_capture import setup  # noqa: E402
from capture_quality import stationary  # noqa: E402
from robot_ros import BASE, JOINTS, TOOL, Robot, matrix  # noqa: E402

PATTERN, SQUARE = (7, 5), 0.030                                     # inner corners (columns, rows), square side in m
LIMITS = np.array([153.6, 149.5, 149.5, 149.0, 145.0, 149.0])
MARGIN = 6.0
HOME = (np.array([3.0266, 356.8712, 93.4037, 271.4634, 272.3364, 277.3673]) + 180) % 360 - 180
SPEED, MIN_MOVE_S, SETTLE_S, REACH_TOL = 8.0, 2.5, 1.5, 1.5
TCP_ABOVE, CAM_ABOVE = 0.05, 0.08
BOX_X, BOX_Y, BASE_ZONE = (0.12, 0.55), (-0.30, 0.30), 0.20
TCP_IN_TOOL = np.array([0.0, 0.0, 0.035])
IMG_MARGIN_PX = 40                                                  # every corner this far from the image border
MAX_STEP, MAX_FROM_HOME = 90.0, 120.0
# (camera distance to the aim point m, tilt from vertical deg, azimuth deg from base +x, roll deg, aim offset along base x m, along base y m)
DESIGN, _k = [], 0
_dists, _rolls = (0.26, 0.30, 0.34), (0, 15, -15, 25, -25)
_offs = ((0, 0), (0.03, 0.08), (-0.03, -0.08), (0.05, -0.06), (-0.05, 0.06), (0.0, 0.10), (0.0, -0.10))
for _tilt in (0, 15, 30, 40):
    for _az in ([0, 0, 0] if _tilt == 0 else [0, 60, 120, 270, 300, 330]):
        DESIGN.append((_dists[_k % 3], _tilt, _az, _rolls[_k % 5], *_offs[_k % 7]))
        _k += 1
DESIGN += [(0.30, 25, 60, 0, 0, 0.04), (0.34, 25, 300, 10, 0, -0.04), (0.30, 20, 120, -10, 0.02, 0.04), (0.36, 20, 0, 15, -0.02, 0.0),
           (0.32, 30, 330, -10, 0.0, 0.05), (0.34, 25, 270, 0, 0.03, 0.0), (0.28, 20, 60, 20, -0.03, -0.04), (0.32, 35, 120, 0, 0.0, 0.0)]

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--execute", action="store_true")
ap.add_argument("--out", type=pathlib.Path)
args = ap.parse_args()

cfg = yaml.safe_load(open(CAL / "calibration.yaml"))
X0 = cl.to_T(cfg["camera"]["hand_eye_xyz_m"], cfg["camera"]["hand_eye_quat_xyzw"])
OBJ = cl.chessboard_points(PATTERN[0], PATTERN[1], SQUARE).astype(np.float64)
CENTRE_OBJ = np.array([(PATTERN[0] - 1) / 2 * SQUARE, (PATTERN[1] - 1) / 2 * SQUARE, 0.0, 1.0])
wrap = lambda q: (np.asarray(q, float) + 180) % 360 - 180

rclpy.init()
robot = Robot("auto_chessboard", background=False)
if not robot.wait_ready(10):
    sys.exit("no driver")
sub = setup(robot)
fault = {"v": False}
robot.create_subscription(Bool, "/fault_controller/internal_fault", lambda m: fault.__setitem__("v", bool(m.data)), 10)
fk_cli = robot.create_client(GetPositionFK, "/compute_fk")
robot.spin_until(lambda: hasattr(robot, "color_info"), 5)
if not fk_cli.wait_for_service(10):
    sys.exit("no /compute_fk (MoveIt up?)")


def find_board(color):
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCornersSB(gray, PATTERN, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return c.reshape(-1, 2).astype(np.float64) if ok else None


def pnp(corners, K, dist):
    ok, rv, tv = cv2.solvePnP(OBJ, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    proj = cv2.projectPoints(OBJ, rv, tv, K, dist)[0].reshape(-1, 2)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
    return T, float(np.sqrt(np.mean(np.sum((proj - corners) ** 2, axis=1))))


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


def clearance_error(T_tool, z_plane):
    tcp, cam = (T_tool @ np.r_[TCP_IN_TOOL, 1])[:3], (T_tool @ X0)[:3, 3]
    for name, p, above in (("fingertip", tcp, TCP_ABOVE), ("camera", cam, CAM_ABOVE)):
        if p[2] < z_plane + above:
            return "%s %.0f mm above the board plane (need %.0f)" % (name, (p[2] - z_plane) * 1e3, above * 1e3), p[2] - z_plane
        if not (BOX_X[0] <= p[0] <= BOX_X[1] and BOX_Y[0] <= p[1] <= BOX_Y[1]):
            return "%s outside the workspace box at x %.2f y %.2f" % (name, p[0], p[1]), p[2] - z_plane
        if np.hypot(p[0] + 0.05, p[1]) < BASE_ZONE:
            return "%s inside the base zone" % name, p[2] - z_plane
    return None, min(tcp[2], cam[2]) - z_plane


def path_report(q_from, q_to, z_plane):
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    step = float(np.abs(q_to - q_from).max())
    worst = np.inf
    for s in np.linspace(0, 1, max(2, int(np.ceil(step / 3.0)) + 1)):
        q = q_from + (q_to - q_from) * s
        err = limit_error(q)
        if err:
            return False, err, step, worst
        err, clear = clearance_error(fk(q), z_plane)
        worst = min(worst, clear)
        if err:
            return False, "at %.0f%% of the move: %s" % (100 * s, err), step, worst
    return True, "ok", step, worst


def look_at(aim, n, d, theta, phi, roll):
    e1 = np.array([1.0, 0, 0]) - n * n[0]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    t, p = np.radians(theta), np.radians(phi)
    pos = aim + d * (np.cos(t) * n + np.sin(t) * (np.cos(p) * e1 + np.sin(p) * e2))
    z = (aim - pos) / np.linalg.norm(aim - pos)
    y = -(e1 - (e1 @ z) * z)
    y /= np.linalg.norm(y)
    Rc = np.column_stack([np.cross(y, z), y, z]) @ Rot.from_euler("z", roll, degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rc, pos
    return T


def corners_visible(T_cam_world, K, board_to_world):
    """All board corners inside the image with IMG_MARGIN_PX, seen from the planned camera pose."""
    P = (np.linalg.inv(T_cam_world) @ board_to_world @ np.c_[OBJ, np.ones(len(OBJ))].T)[:3]
    if (P[2] <= 0.05).any():
        return False
    uv = (K @ P)[:2] / P[2]
    return bool((uv[0] > IMG_MARGIN_PX).all() and (uv[0] < 848 - IMG_MARGIN_PX).all() and (uv[1] > IMG_MARGIN_PX).all() and (uv[1] < 480 - IMG_MARGIN_PX).all())


# ---- board position from the home view (current hand-eye, centimetre level) ----
color, _, K0 = robot.grab()
dist0 = np.array(robot.color_info.d)
if len(dist0) == 0:
    dist0 = np.zeros(5)
corners = find_board(color)
if corners is None:
    sys.exit("chessboard %dx%d inner corners NOT found from this pose: place it in view first" % PATTERN)
T_cb, rms = pnp(corners, K0, dist0)
T_bb0 = robot.tool_matrix() @ X0 @ T_cb
c, n = (T_bb0 @ CENTRE_OBJ)[:3], T_bb0[:3, 2] * np.sign(T_bb0[2, 2])   # n: board plane normal pointing up
print("board FOUND (%dx%d inner corners, PnP reprojection %.2f px). Centre in base (m): %s | plane normal %.1f deg from vertical | distance from the camera %.2f m" % (*PATTERN, rms, np.round(c, 3), np.degrees(np.arccos(n[2])), T_cb[2, 3]), flush=True)
q_now = wrap(np.degrees(robot.q))
if np.abs(q_now - HOME).max() > 2:
    sys.exit("arm is %.1f deg from project home: move it home first" % np.abs(q_now - HOME).max())

# ---- plan ----
ex = np.array([1.0, 0, 0]) - n * n[0]
ex /= np.linalg.norm(ex)
ey = np.cross(n, ex)
remaining = [(k, par, look_at(c + par[4] * ex + par[5] * ey, n, par[0], par[1], par[2], par[3])) for k, par in enumerate(DESIGN)]
plan, rows, q_prev = [], [], q_now
while remaining:
    best = None
    for k, par, T_cam in remaining:
        T_tool = T_cam @ np.linalg.inv(X0)
        if not corners_visible(T_cam, K0, T_bb0):
            continue
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
for k, par, T_cam in remaining:
    rows.append(dict(index=k, design=par, joints_deg=None, ok=False, why="board not fully visible in the planned view, or no IK in range"))
print("\n pose  (dist, tilt, azim, roll, aim dx, dy)         joints (deg)                          step  clearance  result")
for r in rows:
    print(" %3d  %-38s %-42s %5s %8s   %s" % (r["index"], tuple(round(v, 2) for v in r["design"]), r["joints_deg"], r.get("step_deg", "-"), r.get("min_clearance_mm", "-"), "OK" if r["ok"] else "SKIP: " + r["why"]), flush=True)
Rs = [Rot.from_matrix(np.array(p["T_tool"])[:3, :3]) for p in plan]
spread = max((a * b.inv()).magnitude() for a in Rs for b in Rs) if Rs else 0
print("\n%d of %d poses pass every check; widest tool rotation between two poses %.0f deg" % (len(plan), len(DESIGN), np.degrees(spread)), flush=True)
if not args.execute:
    print("PLAN ONLY, nothing moved.")
    robot.destroy_node(); rclpy.shutdown(); sys.exit(0)
if len(plan) < 12:
    sys.exit("fewer than 12 safe poses: not executing")

# ---- execute ----
run = args.out or (pathlib.Path.home() / "kinova/work/calibration" / time.strftime("%Y%m%d_%H%M%S_chessboard"))
(run / "frames").mkdir(parents=True)
(run / "plan.json").write_text(json.dumps(dict(board_centre_m=c.tolist(), board_normal=n.tolist(), pattern=PATTERN, square_m=SQUARE, rows=rows, limits=LIMITS.tolist(), margin_deg=MARGIN, speed_deg_s=SPEED), indent=1))
log = open(run / "console.log", "w")
t0 = time.time()


def say(msg):
    line = "[%6.1f s] %s" % (time.time() - t0, msg)
    print(line, flush=True)
    log.write(line + "\n")
    log.flush()


def safe_move(q_to, label):
    robot.spin_until(lambda: False, 0.3)
    if fault["v"]:
        raise RuntimeError("arm in fault")
    ok, why, step, _ = path_report(wrap(np.degrees(robot.q)), q_to, c[2])
    if not ok:
        raise RuntimeError("path to %s rejected: %s" % (label, why))
    robot.move_joints(q_to, max(MIN_MOVE_S, step / SPEED))
    time.sleep(SETTLE_S)
    robot.spin_until(lambda: False, 0.3)
    miss = np.abs(wrap(np.degrees(robot.q)) - wrap(q_to)).max()
    if fault["v"] or miss > REACH_TOL:
        raise RuntimeError("stopped short at %s (fault %s, %.1f deg off)" % (label, fault["v"], miss))


def burst(count=8):
    """8 fresh colour frames while stationary: board corners per frame, robot TF at each image time."""
    stamp = lambda m: m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    cached = robot._img.get("color")
    last = stamp(cached) if cached is not None else -1
    allc, poses, mid = [], [], None
    for i in range(count):
        if not robot.spin_until(lambda: "color" in robot._img and stamp(robot._img["color"]) > last, 3):
            raise ValueError("no fresh colour frame")
        msg = robot._img["color"]
        last = stamp(msg)
        if not 0 <= (robot.get_clock().now().nanoseconds - last) / 1e9 < 1:
            raise ValueError("colour image timestamp stale")
        img = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data, strides=(msg.step, 3, 1)).copy()
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cc = find_board(img)
        if cc is None:
            raise ValueError("board not found in frame %d" % i)
        if cc[:, 0].min() < IMG_MARGIN_PX / 2 or cc[:, 1].min() < IMG_MARGIN_PX / 2 or cc[:, 0].max() > msg.width - IMG_MARGIN_PX / 2 or cc[:, 1].max() > msg.height - IMG_MARGIN_PX / 2:
            raise ValueError("board too close to the image border")
        if allc and np.linalg.norm(cc - allc[0], axis=1).mean() > np.linalg.norm(cc[::-1] - allc[0], axis=1).mean():
            cc = cc[::-1]                                              # same physical corner order as the first frame
        when = rclpy.time.Time.from_msg(msg.header.stamp)
        if not robot.spin_until(lambda: robot.tfbuf.can_transform(BASE, TOOL, when), 2):
            raise ValueError("no robot TF at the image time")
        tr = robot.tfbuf.lookup_transform(BASE, TOOL, when).transform
        poses.append(matrix([tr.translation.x, tr.translation.y, tr.translation.z], [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]))
        allc.append(cc)
        if i == count // 2:
            mid = img
    motion = stationary(poses)
    arr = np.array(allc)
    std = float(np.max(np.std(arr, axis=0)))
    if std > 0.4:
        raise ValueError("corners unstable (%.2f px)" % std)
    return poses[len(poses) // 2], np.median(arr, axis=0), mid, dict(corner_std_max_px=std, **motion)


caps = []
try:
    for i, p in enumerate(plan):
        say("pose %d/%d design %s" % (i + 1, len(plan), tuple(round(v, 2) for v in p["design"])))
        safe_move(np.array(p["q_deg"]), "pose %d" % (i + 1))
        for tries in range(2):
            try:
                BT, cc, mid, quality = burst()
                _, r_ = pnp(cc, K0, dist0)
                cv2.imwrite(str(run / "frames" / ("pose_%02d.png" % i)), mid)
                caps.append(dict(pose=i, design=p["design"], T_base_tool=BT.tolist(), corners=cc.tolist(), K=K0.tolist(), dist=dist0.tolist(), quality=quality, pnp_rms_px=r_))
                say("  capture %d: corner std %.2f px, PnP reprojection %.2f px, board at %.0f mm" % (len(caps), quality["corner_std_max_px"], r_, pnp(cc, K0, dist0)[0][2, 3] * 1e3))
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
    (run / "captures.json").write_text(json.dumps(caps, indent=1))

# ---- solve: intrinsics ----
n_c = len(caps)
say("%d captures" % n_c)
if n_c < 10:
    say("too few captures to solve")
    sys.exit(1)
val = list(range(3, n_c, 4))
train = [i for i in range(n_c) if i not in val]
imgp = [np.array(cp["corners"], np.float32) for cp in caps]
objp = [OBJ.astype(np.float32)] * n_c
size = (848, 480)


def heldout_rms(K, dist, ids):
    e = []
    for i in ids:
        _, r_ = pnp(imgp[i].astype(np.float64), K, dist)
        e.append(r_)
    return float(np.sqrt(np.mean(np.square(e))))


models = {"factory": (K0, dist0)}
base_flags = cv2.CALIB_USE_INTRINSIC_GUESS
for name, flags in (("K only (no distortion)", base_flags | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3),
                    ("K + k1, k2", base_flags | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)):
    rms_t, K, dist, _, _ = cv2.calibrateCamera([objp[i] for i in train], [imgp[i] for i in train], size, K0.copy(), np.zeros(5), flags=flags)
    models[name] = (K, dist.ravel())
    say("%-24s train RMS %.3f px | fx %.1f fy %.1f cx %.1f cy %.1f | dist %s" % (name, rms_t, K[0, 0], K[1, 1], K[0, 2], K[1, 2], np.round(dist.ravel()[:5], 4)))
held = {name: heldout_rms(K, d, val) for name, (K, d) in models.items()}
for name, v in held.items():
    say("HELD-OUT reprojection RMS with %-24s %.3f px" % (name, v))
best = min(held, key=held.get)
say("best intrinsics model on held-out views: %s" % best)

# ---- solve: hand-eye from the board ----
BTs = [np.array(cp["T_base_tool"]) for cp in caps]


def board_poses(K, dist):
    """Board pose in the camera per capture; the 180 deg symmetric alternative is chosen so the board frame is the same in the base for every view."""
    out, ref = [], None
    for i in range(n_c):
        cands = []
        for flip in (False, True):
            cc = imgp[i].astype(np.float64)[::-1] if flip else imgp[i].astype(np.float64)
            T, _ = pnp(cc, K, dist)
            cands.append(T)
        if ref is None:
            ref = (BTs[0] @ X0 @ cands[0])[:3, :3]
            out.append(cands[0])
            continue
        angles = [(Rot.from_matrix(ref.T @ (BTs[i] @ X0 @ T)[:3, :3])).magnitude() for T in cands]
        out.append(cands[int(np.argmin(angles))])
    return out


def evaluate(label, K, dist):
    CB = board_poses(K, dist)
    pos = lambda X, ids: np.array([(BTs[i] @ X @ CB[i])[:3, 3] for i in ids])
    Xt = cl.hand_eye([BTs[i] for i in train], [CB[i] for i in train])
    P = pos(Xt, train)
    held_new = np.linalg.norm(pos(Xt, val) - P.mean(0), axis=1) * 1e3
    held_cur = np.linalg.norm(pos(X0, val) - pos(X0, train).mean(0), axis=1) * 1e3
    sp_new, sp_cur = cl.hand_eye_residual([BTs[i] for i in train], [CB[i] for i in train], Xt), cl.hand_eye_residual([BTs[i] for i in train], [CB[i] for i in train], X0)
    Xa = cl.hand_eye(BTs, CB)
    say("[%s] board position spread, NEW: std %s mm max %.1f mm, %.2f deg | CURRENT hand-eye: std %s mm max %.1f mm" % (label, np.round(sp_new["std_mm"], 2), sp_new["max_dev_mm"], sp_new["max_angle_deg"], np.round(sp_cur["std_mm"], 2), sp_cur["max_dev_mm"]))
    say("[%s] HELD-OUT board position error: new %.1f mm, current %.1f mm | new vs current hand-eye differ by %.1f mm, %.2f deg" % (label, held_new.mean(), held_cur.mean(), *cl.pose_difference(Xt, X0)))
    return dict(label=label, train_spread=sp_new, current_spread=sp_cur, held_out_mm=dict(new=held_new.round(2).tolist(), current=held_cur.round(2).tolist(), new_mean=float(held_new.mean()), current_mean=float(held_cur.mean())),
                diff_new_vs_current_mm_deg=cl.pose_difference(Xt, X0), all_samples=dict(xyz=cl.xyz_quat(Xa)[0], quat=cl.xyz_quat(Xa)[1]), X_all=Xa.tolist())


res = dict(n_captures=n_c, train=train, held_out=val, intrinsics={name: dict(K=K.tolist(), dist=d.tolist(), held_out_rms_px=held[name]) for name, (K, d) in models.items()}, best_intrinsics=best,
           hand_eye=[evaluate("factory intrinsics", *models["factory"])])
if best != "factory":
    res["hand_eye"].append(evaluate("calibrated: " + best, *models[best]))
marker_ref = pathlib.Path.home() / "kinova/work/calibration"
refs = sorted(marker_ref.glob("*marker_hand_eye*/solve.json"))
if refs:
    Xm = np.array(cl.to_T(json.load(open(refs[-1]))["all_samples_hand_eye"]["xyz"], json.load(open(refs[-1]))["all_samples_hand_eye"]["quat"]))
    for h in res["hand_eye"]:
        say("[%s] board hand-eye vs the marker-based hand-eye (%s): %.1f mm, %.2f deg" % (h["label"], refs[-1].parent.name[:22], *cl.pose_difference(np.array(h["X_all"]), Xm)))
(run / "solve.json").write_text(json.dumps(res, indent=1))
for h in res["hand_eye"]:
    say("[%s] hand_eye_xyz_m: %s  hand_eye_quat_xyzw: %s" % (h["label"], [round(v, 7) for v in h["all_samples"]["xyz"]], [round(v, 7) for v in h["all_samples"]["quat"]]))
log.close()
robot.destroy_subscription(sub)
robot.destroy_node()
rclpy.shutdown()
