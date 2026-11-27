#!/usr/bin/env python3
"""Two-stage insertion servo on one printed marker, translation only (the gripper keeps the post-grasp orientation).
  python3 ros2_iface/insert_servo.py --wire aux|usb|power [--speed 0.05] [--step-m 0.04]
Each cycle: detect the wire's marker, turn its pixel centre + side length into a 3-D offset of the camera from where it must be
(centre/side from the reference images in config/insertion_refs.yaml), and run a quintic Cartesian line of the tool by that offset
(MoveIt path, same smooth profile as the grasp pipeline). Stage 1 -> pre-insertion image, brief stop, stage 2 -> insertion image."""
import argparse, datetime, pathlib, subprocess, sys, time
import cv2, numpy as np, yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / "Grasp")); sys.path.insert(0, str(ROOT.parent / "Grasp" / "ros2_iface"))
import rclpy  # noqa: E402
from robot_ros import Robot, matrix, JOINTS  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402
from run_record import Recorder, git_state  # noqa: E402

REC = None                                              # run recorder, set in main


def record(r, stage, color, c, goal, d, side, cmd=None):
    if REC is not None:
        REC.step(stage, color, c, goal, d, side, cmd, r.tool_matrix()[:3, 3] * 1e3, np.degrees(r.q))


class Blocked(RuntimeError):
    pass


class NoPath(Exception):
    def __init__(self, T_goal):
        super().__init__("no Cartesian path for this step")
        self.T_goal = T_goal

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
if hasattr(cv2.aruco, "ArucoDetector"):
    _detector = cv2.aruco.ArucoDetector(DICTIONARY, cv2.aruco.DetectorParameters())
    detect_markers = _detector.detectMarkers
else:                                                   # OpenCV < 4.7
    _params = cv2.aruco.DetectorParameters_create()
    detect_markers = lambda gray: cv2.aruco.detectMarkers(gray, DICTIONARY, parameters=_params)


def watch_camera(r):
    """Colour image and its intrinsics straight from the colour topics: the servo needs no depth (aligned depth only flows while raw depth is subscribed)."""
    r._color = r._info = None
    r.create_subscription(Image, "/camera/camera/color/image_raw", lambda m: setattr(r, "_color", m), qos_profile_sensor_data)
    r.create_subscription(CameraInfo, "/camera/camera/color/camera_info", lambda m: setattr(r, "_info", m), qos_profile_sensor_data)


def grab_color(r, timeout=2.0):
    """(BGR image, 3x3 intrinsics) of the next colour frame."""
    r._color = None
    if not r.spin_until(lambda: r._color is not None and r._info is not None, timeout):
        raise RuntimeError("no colour frames on /camera/camera/color/image_raw within %.0f s (camera running?)" % timeout)
    m = r._color
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    return np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img), np.array(r._info.k).reshape(3, 3)


def observe(r, marker_id, tries=30):
    """Marker corners (4x2 px) from a frame taken after the arm stopped, or raise."""
    r.spin_until(lambda: False, 0.35)
    why = "no frame"
    for _ in range(tries):
        r.spin_until(lambda: False, 0.05)
        try:
            color, K = grab_color(r)
        except RuntimeError as e:
            why = str(e)
            continue
        corners, ids, _ = detect_markers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        if ids is not None and marker_id in ids.ravel():
            c = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2)
            return c, np.asarray(K, float), color
        why = "frames arrive but marker %d is not in them (seen ids: %s)" % (marker_id, [] if ids is None else sorted(ids.ravel().tolist()))
    raise RuntimeError("marker %d not found: %s" % (marker_id, why))


def offset(c, K, goal, L):
    """Camera translation (camera frame, m) that puts the marker at the goal centre/side: marker now (xZ, yZ, Z) minus goal (x*Z*, y*Z*, Z*)."""
    f = (K[0, 0] + K[1, 1]) / 2
    side = np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)])
    centre = c.mean(0)
    Z = f * L / side
    Zg = f * L / goal["side"]
    p = lambda uv, z: np.array([(uv[0] - K[0, 2]) / f * z, (uv[1] - K[1, 2]) / f * z, z])
    return p(centre, Z) - p(goal["c"], Zg), side


def goal_pose(r, T_EC, hold_R, step):
    T = r.tool_matrix()
    T_goal = T.copy(); T_goal[:3, :3] = hold_R
    T_goal[:3, 3] = T[:3, 3] + (T @ T_EC)[:3, :3] @ step
    return T_goal


def line(r, T_goal):
    pts = r.line_path(T_goal)
    if pts is None:
        raise NoPath(T_goal)
    return pts


def recover(r, T_goal, speed):
    """No Cartesian path from here: go home (safest joint route), then run the line to the same tool pose from home."""
    print("no Cartesian path from here: going home and trying the same target from there", flush=True)
    r.stop()
    subprocess.run([sys.executable, str(ROOT / "ros2_iface" / "go_to_pose.py"), "--pose", "home"], check=True)
    r.spin_until(lambda: False, 0.5)
    pts = r.line_path(T_goal)
    if pts is None:
        raise RuntimeError("no Cartesian path from home either: stopping")
    r.send_line(pts, max(1.5, float(np.linalg.norm(r.tool_matrix()[:3, 3] - T_goal[:3, 3])) / (2 * speed)))


def gate(d, align_m):
    """Alignment before approach: the move toward the wall (camera z) is scaled by how well sideways (x, y) is already aligned,
    0 when the sideways error is align_m or more, 1 when it is zero. None = no gate."""
    if align_m is None:
        return d
    g = float(np.clip(1.0 - np.linalg.norm(d[:2]) / align_m, 0.0, 1.0))
    return np.array([d[0], d[1], d[2] * g])


def send_smooth(r, pts, L, v, rest):
    """Send the joint waypoints of a line of Cartesian length L (without waiting). Time law: quintic in path position, so position, speed and
    acceleration are continuous; it starts at the speed the arm has now (no jerk when it replaces the running trajectory) and ends at
    cruise speed v, or at rest. Returns the duration, and the Cartesian length still ahead."""
    P = np.array(pts)
    full = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
    q_now = r.q.copy()
    ahead = P[int(np.argmin(np.linalg.norm(P - q_now, axis=1))) + 1:]       # the arm is already past the first points
    if len(ahead) == 0:
        return 0.0, 0.0
    P = np.vstack([q_now, ahead])
    P = P[np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-6]]
    if len(P) < 2:
        return 0.0, 0.0
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    Lr = L * arc[-1] / max(full, 1e-9)
    u = arc / arc[-1]
    fit = [np.polynomial.Polynomial.fit(u, P[:, j], min(3, len(P) - 1)) for j in range(6)]
    dq0 = np.array([f.deriv()(0.0) for f in fit])
    sd0 = max(0.0, float(r.qd @ dq0 / max(dq0 @ dq0, 1e-12)))               # measured speed along the path, in path-fractions per second
    sdT = 0.0 if rest else v / max(Lr, 1e-9)
    T = max(0.6, 1.875 * Lr / v) if rest else max(0.3, 2 * Lr / (sd0 * Lr + v))
    for _ in range(8):
        M = np.array([[T**3, T**4, T**5], [3 * T**2, 4 * T**3, 5 * T**4], [6 * T, 12 * T**2, 20 * T**3]])
        c3, c4, c5 = np.linalg.solve(M, [1 - sd0 * T, sdT - sd0, 0.0])
        t = np.linspace(0.0, T, max(4, int(T * 15) + 1))
        s = sd0 * t + c3 * t**3 + c4 * t**4 + c5 * t**5
        sd = sd0 + 3 * c3 * t**2 + 4 * c4 * t**3 + 5 * c5 * t**4
        sdd = 6 * c3 * t + 12 * c4 * t**2 + 20 * c5 * t**3
        if sd.min() >= -1e-9 and s.max() <= 1.02:
            break
        T *= 1.3                                                             # time law went backwards or overshot: slower
    s = np.clip(s, 0.0, 1.0)
    traj = JointTrajectory(); traj.joint_names = JOINTS
    for ti, si, sdi, sddi in zip(t, s, sd, sdd):
        q = np.array([f(si) for f in fit]); dq = np.array([f.deriv()(si) for f in fit]); ddq = np.array([f.deriv(2)(si) for f in fit])
        pt = JointTrajectoryPoint()
        pt.positions, pt.velocities = [float(x) for x in q], [float(x) for x in dq * sdi]
        pt.accelerations = [float(x) for x in ddq * sdi**2 + dq * sddi]
        pt.time_from_start = Duration(sec=int(ti + 0.02), nanosec=int(((ti + 0.02) % 1) * 1e9)); traj.points.append(pt)
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    r.arm.send_goal_async(goal).add_done_callback(lambda f: setattr(r, "_goal", f.result()))
    return T, Lr


def go(r, T_EC, hold_R, step, v, rest):
    """Plan the straight line by `step` (camera frame) and send it. Returns what was sent, for the blocked check."""
    T_goal = goal_pose(r, T_EC, hold_R, step)
    start = r.tool_matrix()[:3, 3].copy()
    T, Lr = send_smooth(r, line(r, T_goal), float(np.linalg.norm(step)), v, rest)
    return dict(T=T, Lr=Lr, start=start, cmd=T_goal[:3, 3] - start)


def check_moved(r, sent, fraction):
    """No force sensor: contact shows as the arm not following the command. Stop if it moved less than `fraction` of what was sent."""
    moved = float(np.linalg.norm(r.tool_matrix()[:3, 3] - sent["start"]))
    if sent["Lr"] > 0.004 and moved < fraction * sent["Lr"]:
        r.stop(); r.send_joint_path([r.q.copy()], [0.4], [np.zeros(6)])
        raise Blocked("arm blocked: moved %.1f mm of %.1f mm commanded (wall or plug contact?): stopped" % (moved * 1e3, sent["Lr"] * 1e3))


def back_off(r, T_EC, hold_R, distance, speed):
    """Straight back from the wall along the camera axis, before any other move: going home from the socket would sweep the arm into the wall."""
    T_goal = goal_pose(r, T_EC, hold_R, np.array([0.0, 0.0, -distance]))
    pts = r.line_path(T_goal)
    if pts is None:
        print("WARNING: no straight path to back off, staying at the socket entrance", flush=True)
        return
    print("backing off %.0f mm" % (distance * 1e3), flush=True)
    r.send_line(pts, max(1.0, distance / speed))


def stage(r, T_EC, hold_R, marker_id, goal, L, speed, step_m, tol_m, name, align_m=None):
    """Stop-and-observe: quintic chunks of at most step_m, each ending at rest."""
    while True:
        c, K, color = observe(r, marker_id)
        d, side = offset(c, K, goal, L)
        n = float(np.linalg.norm(d))
        print("%s: marker %.0f px, still %.1f mm to go (x %+.1f y %+.1f z %+.1f)" % (name, side, n * 1e3, *(d * 1e3)), flush=True)
        if n < tol_m:
            record(r, name, color, c, goal, d, side)
            return
        d_go = gate(d, align_m)
        step = d_go * min(1.0, step_m / max(float(np.linalg.norm(d_go)), 1e-9))
        T_goal = goal_pose(r, T_EC, hold_R, step)
        start = r.tool_matrix()[:3, 3].copy()
        cmd = T_goal[:3, 3] - start
        print("  step in base frame (mm): %s, tool height %.0f mm" % (np.round(cmd * 1e3, 1), start[2] * 1e3), flush=True)
        record(r, name, color, c, goal, d, side, cmd)
        try:
            r.send_line(line(r, T_goal), max(0.6, float(np.linalg.norm(step)) / speed))
        except NoPath as e:
            recover(r, e.T_goal, speed)
            continue
        r.spin_until(lambda: False, 0.3)
        check_moved(r, dict(start=start, Lr=float(np.linalg.norm(cmd))), 0.5)


def stage_continuous(r, T_EC, hold_R, marker_id, goal, L, speed, tol_m, name, align_m=None, horizon=1.2, final_m=0.015):
    """Keep looking while the arm moves. Every camera frame is recorded, but a new trajectory is sent only every ~half horizon
    (replacing it on every frame never lets the arm get going). Far away: cruise toward the goal, each piece ends at speed and the next
    one starts at the speed the arm has. Within final_m: one smooth move that ends at rest on the goal."""
    lost, last, next_send, sent, final, why = 0, 0.0, 0.0, None, False, ""
    while True:
        try:
            r.spin_until(lambda: False, 0.02)
            color, K = grab_color(r)
            corners, ids, _ = detect_markers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
            if ids is None or marker_id not in ids.ravel():
                raise RuntimeError("frames arrive but marker %d is not in them (seen ids: %s)" % (marker_id, [] if ids is None else sorted(ids.ravel().tolist())))
            c, K = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2), np.asarray(K, float)
        except RuntimeError as e:
            lost, why = lost + 1, str(e)
            if lost > 6:
                raise RuntimeError("marker %d lost: %s" % (marker_id, why))
            continue
        lost = 0
        d, side = offset(c, K, goal, L)
        n = float(np.linalg.norm(d))
        if time.time() - last > 0.5:
            print("%s: marker %.0f px, still %.1f mm to go, camera-frame offset (mm) %s, tool height %.0f mm" % (name, side, n * 1e3, np.round(d * 1e3, 1), r.tool_matrix()[2, 3] * 1e3), flush=True); last = time.time()
        if time.time() < next_send:
            record(r, name, color, c, goal, d, side)
            continue
        if sent is not None:
            check_moved(r, sent, 0.5 if final else 0.12)                # was the last command followed?
        if n < tol_m:
            record(r, name, color, c, goal, d, side)
            r.stop(); r.send_joint_path([r.q.copy()], [0.4], [np.zeros(6)])
            return
        d_go = gate(d, align_m)
        ng = float(np.linalg.norm(d_go))
        try:
            v = min(speed, max(0.01, 2.0 * ng))
            h = min(v * horizon, ng - final_m)
            final = n <= final_m or h < 0.01                            # a cruise piece under 1 cm would not move the arm
            if final:
                sent = go(r, T_EC, hold_R, d_go, speed, rest=True)
                record(r, name, color, c, goal, d, side, sent["cmd"])
                r.spin_until(lambda: False, sent["T"] + 0.3)            # arrive and settle, then the next frame is a fresh one
                next_send = 0.0
            else:
                sent = go(r, T_EC, hold_R, d_go * (h / ng), v, rest=False)
                record(r, name, color, c, goal, d, side, sent["cmd"])
                next_send = time.time() + 0.5 * sent["T"]
        except NoPath as e:
            recover(r, e.T_goal, speed)
            sent = None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wire", required=True, choices=("aux", "usb", "power"))
    ap.add_argument("--speed", type=float, default=0.05, help="average tool speed, m/s (grasp pipeline line speed)")
    ap.add_argument("--step-m", type=float, default=0.04, help="re-observe the marker after at most this much travel")
    ap.add_argument("--tol-mm", type=float, default=2.0)
    ap.add_argument("--align-mm", type=float, default=15.0, help="insertion approach: forward motion starts only once the sideways error is below this (0 = no gate)")
    ap.add_argument("--back-off-m", type=float, default=0.15, help="after reaching the socket entrance, back straight out this far before anything else (0 = stay)")
    ap.add_argument("--stepwise", action="store_true", help="stop after every chunk to look again (default: continuous, re-plans on every frame while moving)")
    ap.add_argument("--out", type=pathlib.Path, help="run folder (default: Media/IBVS/run_<time>_insert_<wire>)")
    ap.add_argument("--pause", type=float, default=1.0, help="stop at the pre-insertion pose, s")
    a = ap.parse_args()
    a.continuous = not a.stepwise
    refs = yaml.safe_load((ROOT / "config" / "insertion_refs.yaml").read_text())
    w, L = refs["wires"][a.wire], refs["marker_side_m"]
    grasp = yaml.safe_load((ROOT.parent / "Grasp" / "config" / "grasp.yaml").read_text())
    T_EC = matrix(grasp["camera"]["hand_eye_xyz_m"], grasp["camera"]["hand_eye_quat_xyzw"])
    rclpy.init()
    r = Robot("insert_servo")
    if not r.wait_ready(15) or not r.ik_srv.wait_for_service(10) or not r.cart_srv.wait_for_service(10):
        sys.exit("arm / MoveIt not ready")
    watch_camera(r)
    r.qd = np.zeros(6)                                      # joint speeds, for a jerk-free splice between trajectories

    def joint_speeds(m):
        d = dict(zip(m.name, m.velocity))
        if all(j in d for j in JOINTS):
            r.qd = np.array([d[j] for j in JOINTS])
    r.create_subscription(JointState, "/joint_states", joint_speeds, 10)
    hold_R = r.tool_matrix()[:3, :3].copy()
    ins = dict(c=np.add(w["insert"]["c"], w["nudge_px"]), side=w["insert"]["side"])
    global REC
    folder = a.out or ROOT.parents[1] / "Media" / "IBVS" / ("run_%s_insert_%s" % (datetime.datetime.now().strftime("%Y%m%d_%H%M%S"), a.wire))
    REC = Recorder(folder, dict(
        started=datetime.datetime.now().isoformat(timespec="seconds"), command=sys.argv, wire=a.wire,
        args={k: str(v) if isinstance(v, pathlib.Path) else v for k, v in vars(a).items()},
        marker_id=w["marker_id"], marker_side_m=L, pre_goal=w["pre"], insertion_goal=dict(c=list(map(float, ins["c"])), side=ins["side"]),
        nudge_px=w["nudge_px"], references=w, hand_eye=grasp["camera"], post_grasp_joints_deg=grasp["post_grasp"]["joint_waypoints_deg"][-1],
        start_joints_deg=np.degrees(r.q).round(2).tolist(), held_tool_rotation=hold_R.round(4).tolist(), git=git_state(ROOT),
        insertion_refs_file=(ROOT / "config" / "insertion_refs.yaml").read_text()))
    print("recording to", folder, flush=True)
    try:
        for name, goal in (("pre-insertion", w["pre"]), ("insertion", ins)):
            align = a.align_mm / 1e3 if name == "insertion" and a.align_mm > 0 else None      # align sideways first, then go toward the wall
            if a.continuous:
                stage_continuous(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.tol_mm / 1e3, name, align)
            else:
                stage(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.step_m, a.tol_mm / 1e3, name, align)
            if name == "pre-insertion":
                time.sleep(a.pause)
        print("at the socket entrance (%s)" % a.wire, flush=True)
        if a.back_off_m > 0:
            back_off(r, T_EC, hold_R, a.back_off_m, a.speed)
        REC.close("COMPLETED_AT_ENTRANCE")
    except BaseException as e:
        try:
            r.stop()
        except Exception:
            pass
        print("STOPPED: %s" % e, flush=True)
        if isinstance(e, Blocked) and a.back_off_m > 0:             # pressing on the plate: get out of contact before anything else moves
            try:
                back_off(r, T_EC, hold_R, a.back_off_m, a.speed)
            except Exception as err:
                print("back-off failed: %s" % err, flush=True)
        REC.close("STOPPED", str(e) or type(e).__name__); raise SystemExit(1)


if __name__ == "__main__":
    main()
