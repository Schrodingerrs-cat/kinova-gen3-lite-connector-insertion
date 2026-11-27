#!/usr/bin/env python3
"""Two-stage insertion servo on one printed marker, translation only (the gripper keeps the post-grasp orientation).
  python3 ros2_iface/insert_servo.py --wire aux|usb|power [--speed 0.05] [--step-m 0.04]
Each cycle: detect the wire's marker, turn its pixel centre + side length into a 3-D offset of the camera from where it must be
(centre/side from the reference images in config/insertion_refs.yaml), and run a quintic Cartesian line of the tool by that offset
(MoveIt path, same smooth profile as the grasp pipeline). Stage 1 -> pre-insertion image, brief stop, stage 2 -> insertion image."""
import argparse, datetime, pathlib, subprocess, sys, time
import cv2, numpy as np, yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT.parent / "Grasp")); sys.path.insert(0, str(ROOT.parent / "Grasp" / "ros2_iface"))
import rclpy  # noqa: E402
from robot_ros import Robot, matrix, JOINTS  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402
from run_record import Recorder, git_state  # noqa: E402
from ibvs.plug import finger_pair_x  # noqa: E402
from refs_file import save_wire  # noqa: E402

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


def _finger_x(r):
    try:
        return finger_pair_x(grab_color(r)[0])
    except RuntimeError:
        return None


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


def arrived(c, side, goal, tol):
    """Done when the marker is close enough to the goal in the image: centre within tol[0] px in x and in y, size within tol[1] px (no need to hit zero)."""
    return bool(np.abs(c.mean(0) - np.asarray(goal["c"])).max() <= tol[0] and abs(side - goal["side"]) <= tol[1])


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
    cruise speed v, or at rest. Returns the duration, the Cartesian length still ahead, and progress(t): how far along it should be t seconds after sending."""
    P = np.array(pts)
    full = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
    q_now = r.q.copy()
    ahead = P[int(np.argmin(np.linalg.norm(P - q_now, axis=1))) + 1:]       # the arm is already past the first points
    none = (0.0, 0.0, lambda t: 0.0)
    if len(ahead) == 0:
        return none
    P = np.vstack([q_now, ahead])
    P = P[np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-6]]
    if len(P) < 2:
        return none
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
    return T, Lr, lambda tt: Lr * float(np.interp(tt, t, s))


def go(r, T_EC, hold_R, step, v, rest):
    """Plan the straight line by `step` (camera frame) and send it. Returns what was sent, for the blocked check."""
    T_goal = goal_pose(r, T_EC, hold_R, step)
    start = r.tool_matrix()[:3, 3].copy()
    T, Lr, prog = send_smooth(r, line(r, T_goal), float(np.linalg.norm(step)), v, rest)
    return dict(T=T, Lr=Lr, prog=prog, t=time.time(), start=start, cmd=T_goal[:3, 3] - start)


def check_moved(r, sent, fraction):
    """No force sensor: contact shows as the arm not following the command. Stop if it moved less than `fraction` of what the plan says it should
    have moved by now (0.2 s of tracking lag allowed; a plan without a time law counts as finished)."""
    moved = float(np.linalg.norm(r.tool_matrix()[:3, 3] - sent["start"]))
    expected = sent["Lr"] if "prog" not in sent else sent["prog"](max(0.0, time.time() - sent["t"] - 0.2))
    if expected > 0.004 and moved < fraction * expected:
        r.stop(); r.send_joint_path([r.q.copy()], [0.4], [np.zeros(6)])
        raise Blocked("arm blocked: moved %.1f mm where %.1f mm was expected (wall or plug contact?): stopped" % (moved * 1e3, expected * 1e3))


def back_off(r, T_EC, hold_R, distance, speed):
    """Straight back from the wall along the camera axis, before any other move: going home from the socket would sweep the arm into the wall."""
    T_goal = goal_pose(r, T_EC, hold_R, np.array([0.0, 0.0, -distance]))
    pts = r.line_path(T_goal)
    if pts is None:
        print("WARNING: no straight path to back off, staying at the socket entrance", flush=True)
        return
    print("backing off %.0f mm" % (distance * 1e3), flush=True)
    r.send_line(pts, max(1.0, distance / speed))


def marker_side(r, marker_id):
    """Side length (px) of the marker in the next frames, or None."""
    for _ in range(3):
        try:
            color, _ = grab_color(r)
        except RuntimeError:
            continue
        corners, ids, _ = detect_markers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        if ids is not None and marker_id in ids.ravel():
            c = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2)
            return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
    return None


def push_depth(r, T_EC, hold_R, origin, z_hat, limit, marker_id, target_side, step=0.002, speed=0.005):
    """Guarded push along the camera axis in short pieces. Stops when the marker is as big as at the taught seated pose (the camera moves with the plug,
    so marker size is the depth), when the arm stops following a piece (contact, jam or full seating: no force sensor, these cannot be told apart),
    or at `limit` past `origin`. Returns (how far past the origin the tool is in m, marker side px or None)."""
    here = lambda: float((r.tool_matrix()[:3, 3] - origin) @ z_hat)
    side = None
    while here() < limit - 0.0005:
        piece = min(step, limit - here())
        before = r.tool_matrix()[:3, 3].copy()
        try:
            sent = go(r, T_EC, hold_R, np.array([0.0, 0.0, piece]), speed, rest=True)
        except NoPath:
            break
        r.spin_until(lambda: False, sent["T"] + 0.25)
        stalled = np.linalg.norm(r.tool_matrix()[:3, 3] - before) < 0.5 * piece
        side = marker_side(r, marker_id) if target_side else None
        if (target_side and side is not None and side >= target_side - 0.2) or stalled:
            break
    return here(), side


def insert_with_search(r, T_EC, hold_R, depth, radius, marker_id, target_side, retreat=0.003, ring=0.002):
    """Push in from the entrance pose. `depth` is where the taught seated pose is (m, from the taught marker sizes); the push may go 10 mm past it
    but stops as soon as the marker has the taught seated size. If it stops early (plug on the rim: the grasp offset is not known to the camera) it backs
    out a little, shifts sideways to points on rings of growing radius (up to `radius`, 8 per ring) and pushes again. Returns (best advance m, its marker side)."""
    T0 = r.tool_matrix()
    origin, R_cam = T0[:3, 3].copy(), (T0 @ T_EC)[:3, :3]
    z_hat = R_cam @ np.array([0.0, 0.0, 1.0])
    offsets = [(0.0, 0.0)] + [(k * ring * np.cos(t), k * ring * np.sin(t)) for k in range(1, int(round(radius / ring)) + 1) for t in np.radians(np.arange(0, 360, 45))]
    limit = depth + (0.010 if target_side else 0.0)
    best = (0.0, None)
    for i, (dx, dy) in enumerate(offsets):
        if i:
            step = R_cam.T @ (origin + R_cam @ np.array([dx, dy, -retreat]) - r.tool_matrix()[:3, 3])
            try:
                sent = go(r, T_EC, hold_R, step, 0.01, rest=True)
            except NoPath:
                continue
            r.spin_until(lambda: False, sent["T"] + 0.25)
        advance, side = push_depth(r, T_EC, hold_R, origin, z_hat, limit, marker_id, target_side)
        print("push %d: offset (%+.1f, %+.1f) mm, advanced %.1f mm (taught depth %.1f mm), marker %s px (seated %s px)" % (
            i, dx * 1e3, dy * 1e3, advance * 1e3, depth * 1e3, "-" if side is None else "%.1f" % side, target_side or "-"), flush=True)
        done = (side is not None and side >= target_side - 0.2) if target_side else advance >= 0.8 * depth
        if advance > best[0]:
            best = (advance, side)
        if done:
            return advance, side
    return best


def measure_marker(r, marker_id, frames=15):
    """Median centre (px, 2) and side (px) of the marker over `frames` frames, or None."""
    seen = []
    for _ in range(frames * 2):
        try:
            color, _ = grab_color(r)
        except RuntimeError:
            continue
        corners, ids, _ = detect_markers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        if ids is not None and marker_id in ids.ravel():
            seen.append(corners[list(ids.ravel()).index(marker_id)].reshape(4, 2))
        if len(seen) >= frames:
            break
    if len(seen) < frames // 2:
        return None
    c = np.median(np.array(seen), axis=0)
    return c.mean(0), float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


def interactive_seated(r, T_EC, hold_R, wire_name, w, command_file, folder):
    """Teach the seated depth with a person in the loop: from the entrance, move as told (the person watches the plug), and when told `seated` record the marker there.
    Commands, one per line in the command file: more [mm] | back [mm] | left|right|up|down [mm] | seated | stop. Moves are along the camera axes, 2 mm if no number."""
    origin = r.tool_matrix()[:3, 3].copy()
    z_hat = (r.tool_matrix() @ T_EC)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    axes = {"more": (0, 0, 1), "back": (0, 0, -1), "left": (-1, 0, 0), "right": (1, 0, 0), "up": (0, -1, 0), "down": (0, 1, 0)}
    command_file = pathlib.Path(command_file)
    n = 0
    while True:
        m = measure_marker(r, w["marker_id"], 8)
        color = None
        try:
            color, _ = grab_color(r)
        except RuntimeError:
            pass
        if color is not None:
            cv2.imwrite(str(folder / "latest.png"), color)
        advance = float((r.tool_matrix()[:3, 3] - origin) @ z_hat) * 1e3
        print("STATUS %d: %.1f mm past the entrance, marker %s px, centre %s. Waiting for: more|back|left|right|up|down [mm] | seated | stop" % (
            n, advance, "-" if m is None else "%.1f" % m[1], "-" if m is None else np.round(m[0], 1).tolist()), flush=True)
        n += 1
        while not command_file.exists():
            r.spin_until(lambda: False, 0.2)
        words = command_file.read_text().split(); command_file.unlink()
        if not words:
            continue
        cmd, mm = words[0], (float(words[1]) if len(words) > 1 else 2.0)
        if cmd == "stop":
            return "TEACH_STOPPED"
        if cmd == "seated":
            m = measure_marker(r, w["marker_id"], 15)
            if m is None:
                print("marker not seen: cannot record, say seated again or move", flush=True)
                continue
            w["seated"] = dict(c=[round(float(m[0][0]), 1), round(float(m[0][1]), 1)], side=round(m[1], 1))
            save_wire(wire_name, w)
            f = (r._info.k[0] + r._info.k[4]) / 2
            depth = 1e3 * f * 0.030 * (1.0 / w["insert"]["side"] - 1.0 / m[1])
            print("TAUGHT %s seated: marker %.1f px at %s (entrance %.1f px): depth %.1f mm, %.1f mm of travel from the entrance pose. Written to insertion_refs.yaml" % (
                wire_name, m[1], w["seated"]["c"], w["insert"]["side"], depth, advance), flush=True)
            return "TAUGHT_SEATED"
        if cmd in axes:
            before = r.tool_matrix()[:3, 3].copy()
            step = np.array(axes[cmd], float) * mm / 1e3
            try:
                sent = go(r, T_EC, hold_R, step, 0.005, rest=True)
            except NoPath:
                print("no straight path for that move", flush=True)
                continue
            r.spin_until(lambda: False, sent["T"] + 0.4)
            moved = float(np.linalg.norm(r.tool_matrix()[:3, 3] - before)) * 1e3
            if moved < 0.5 * mm:
                print("BLOCKED: moved %.1f of %.1f mm (contact, jam, or fully seated?)" % (moved, mm), flush=True)
        else:
            print("unknown command %r" % cmd, flush=True)


def stage(r, T_EC, hold_R, marker_id, goal, L, speed, step_m, tol, name, align_m=None):
    """Stop-and-observe: quintic chunks of at most step_m, each ending at rest."""
    while True:
        c, K, color = observe(r, marker_id)
        d, side = offset(c, K, goal, L)
        n = float(np.linalg.norm(d))
        print("%s: marker %.0f px, still %.1f mm to go (x %+.1f y %+.1f z %+.1f)" % (name, side, n * 1e3, *(d * 1e3)), flush=True)
        if arrived(c, side, goal, tol):
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


def check_wall(r, T_EC, history, K, side, L, window=1.5):
    """The camera moves with the plug, so the marker must get closer as fast as the tool advances. If the tool advances 8 mm or more over the last
    `window` s but the marker-measured distance shrank by less than 40% of it, the plug is pushing the board or sliding along it: stop."""
    now = time.time()
    z_hat = (r.tool_matrix() @ T_EC)[:3, 2]
    history.append((now, r.tool_matrix()[:3, 3].copy(), K[0, 0] * L / side))
    while history and now - history[0][0] > window:
        history.pop(0)
    t0, p0, z0 = history[0]
    advance = float((history[-1][1] - p0) @ z_hat)
    if advance > 0.008 and (z0 - history[-1][2]) < 0.4 * advance:
        r.stop(); r.send_joint_path([r.q.copy()], [0.4], [np.zeros(6)])
        raise Blocked("wall giving way: the tool advanced %.1f mm but the marker only got %.1f mm closer (plug pushing the board or sliding on it)" % (advance * 1e3, (z0 - history[-1][2]) * 1e3))


def stage_continuous(r, T_EC, hold_R, marker_id, goal, L, speed, tol, name, align_m=None, horizon=1.2, final_m=0.015):
    """Keep looking while the arm moves. Every camera frame is recorded, but a new trajectory is sent only every ~half horizon
    (replacing it on every frame never lets the arm get going). Far away: cruise toward the goal, each piece ends at speed and the next
    one starts at the speed the arm has. Within final_m: one smooth move that ends at rest on the goal."""
    lost, last, next_send, sent, final, why, history = 0, 0.0, 0.0, None, False, "", []
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
        if name == "insertion":
            check_wall(r, T_EC, history, K, side, L)
        if time.time() - last > 0.5:
            print("%s: marker %.0f px, still %.1f mm to go, camera-frame offset (mm) %s, tool height %.0f mm" % (name, side, n * 1e3, np.round(d * 1e3, 1), r.tool_matrix()[2, 3] * 1e3), flush=True); last = time.time()
        if time.time() < next_send:
            record(r, name, color, c, goal, d, side)
            continue
        if sent is not None:
            check_moved(r, sent, 0.5 if final else 0.3)                # was the last command followed?
        if arrived(c, side, goal, tol):
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
    ap.add_argument("--tol-px", type=float, default=9.0, help="done when the marker centre is within this many pixels of the goal, in x and in y")
    ap.add_argument("--tol-side-px", type=float, default=4.0, help="... and its side length within this many pixels of the goal's")
    ap.add_argument("--plug-gain", type=float, default=1.0, help="move the insertion goal by this times how far the plug sits from where it sat when the goal was taught (0 = off)")
    ap.add_argument("--align-mm", type=float, default=15.0, help="insertion approach: forward motion starts only once the sideways error is below this (0 = no gate)")
    ap.add_argument("--push-mm", type=float, help="push this far past the entrance (overrides the depth taught with teach_insert.sh <wire> seated)")
    ap.add_argument("--search-mm", type=float, default=4.0, help="if the push stops early, try sideways offsets up to this radius (0 = no search)")
    ap.add_argument("--teach-seated", action="store_true", help="at the entrance, wait for commands from a person (more/back/left/right/up/down/seated/stop) and record the seated pose")
    ap.add_argument("--command-file", type=pathlib.Path, default=pathlib.Path("/tmp/insert_command"), help="where --teach-seated reads its commands")
    ap.add_argument("--back-out-only", action="store_true", help="only back straight out by --back-off-m from where the arm is (after a run that ended at the wall)")
    ap.add_argument("--no-release", action="store_true", help="after a successful push keep holding the plug (the back-off would pull it out again)")
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
    if a.back_out_only:
        back_off(r, T_EC, hold_R, a.back_off_m, 0.01)
        return
    reached_insertion = [False]
    plug_now, plug_shift = None, 0.0                        # the plug sits differently in the grasp every time: shift the goal by how much
    if a.plug_gain > 0 and "plug_x" in w:
        xs = [x for x in (_finger_x(r) for _ in range(15)) if x is not None]
        if len(xs) >= 8:
            plug_now = float(np.median(xs)); plug_shift = (plug_now - w["plug_x"]) * a.plug_gain
            if abs(plug_shift) > 25:
                print("WARNING: plug shift %+.1f px is implausible (fingertips mis-detected or a very different grasp): not used" % plug_shift, flush=True)
                plug_shift = 0.0
            print("plug between the fingers at x = %.1f px (goal taught at %.1f): insertion goal moved %+.1f px sideways" % (plug_now, w["plug_x"], plug_shift), flush=True)
        else:
            print("WARNING: fingertips not found (%d of 15 frames): insertion goal not adjusted for the plug position" % len(xs), flush=True)
    ins = dict(c=np.add(w["insert"]["c"], w["nudge_px"]) + np.array([plug_shift, 0.0]), side=w["insert"]["side"])
    global REC
    folder = a.out or ROOT.parents[1] / "Media" / "IBVS" / ("run_%s_insert_%s" % (datetime.datetime.now().strftime("%Y%m%d_%H%M%S"), a.wire))
    REC = Recorder(folder, dict(
        started=datetime.datetime.now().isoformat(timespec="seconds"), command=sys.argv, wire=a.wire,
        args={k: str(v) if isinstance(v, pathlib.Path) else v for k, v in vars(a).items()},
        marker_id=w["marker_id"], marker_side_m=L, pre_goal=w["pre"], insertion_goal=dict(c=list(map(float, ins["c"])), side=ins["side"]),
        nudge_px=w["nudge_px"], plug_x_taught=w.get("plug_x"), plug_x_now=plug_now, plug_shift_px=plug_shift, references=w, hand_eye=grasp["camera"], post_grasp_joints_deg=grasp["post_grasp"]["joint_waypoints_deg"][-1],
        start_joints_deg=np.degrees(r.q).round(2).tolist(), held_tool_rotation=hold_R.round(4).tolist(), git=git_state(ROOT),
        insertion_refs_file=(ROOT / "config" / "insertion_refs.yaml").read_text()))
    print("recording to", folder, flush=True)
    try:
        for name, goal in (("pre-insertion", w["pre"]), ("insertion", ins)):
            align = a.align_mm / 1e3 if name == "insertion" and a.align_mm > 0 else None      # align sideways first, then go toward the wall
            reached_insertion[0] = reached_insertion[0] or name == "insertion"
            try:
                if a.continuous:
                    stage_continuous(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, (a.tol_px, a.tol_side_px), name, align)
                else:
                    stage(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.step_m, (a.tol_px, a.tol_side_px), name, align)
            except Blocked as e:
                if name == "insertion" and REC.last_to_go is not None and REC.last_to_go <= 15:      # stopped by contact within 15 mm of the goal: the plug is at the socket
                    print("contact at the socket entrance (%.1f mm from the goal): %s" % (REC.last_to_go, e), flush=True)
                    break
                raise
            if name == "pre-insertion":
                time.sleep(a.pause)
        print("at the socket entrance (%s)" % a.wire, flush=True)
        outcome, note = "COMPLETED_AT_ENTRANCE", ""
        if a.teach_seated:
            outcome = interactive_seated(r, T_EC, hold_R, a.wire, w, a.command_file, REC.dir)
            if outcome == "TAUGHT_SEATED":
                print("opening the gripper", flush=True)
                r.gripper(0.0); time.sleep(0.8)
            if a.back_off_m > 0:
                back_off(r, T_EC, hold_R, a.back_off_m, a.speed)
            REC.close(outcome, "taught with a person in the loop")
            return
        f = (r._info.k[0] + r._info.k[4]) / 2
        target_side = None
        if a.push_mm is not None:
            push = a.push_mm
        elif "seated" in w:                                     # depth taught, not predefined: marker sizes at the entrance and fully inserted
            target_side = w["seated"]["side"]
            here = measure_marker(r, w["marker_id"], 10)         # the entrance pose differs by a few mm between runs: depth is measured from where the arm is now
            push = 1e3 * f * L * (1.0 / (here[1] if here else w["insert"]["side"]) - 1.0 / target_side)
            print("taught seated size for %s: marker %.1f px; now %s px, so %.1f mm to go" % (a.wire, target_side, "-" if not here else "%.1f" % here[1], push), flush=True)
        else:
            push = None
            print("no insertion depth: teach it with  bash Code/IBVS/teach_insert.sh %s seated  (plug fully inserted by hand), or pass --push-mm" % a.wire, flush=True)
        if push and push > 0:
            advance, side = insert_with_search(r, T_EC, hold_R, push / 1e3, a.search_mm / 1e3, w["marker_id"], target_side)
            engaged = (side is not None and side >= target_side - 0.2) if target_side else advance >= 0.8 * push / 1e3
            outcome = "ADVANCED_TO_DEPTH" if engaged else "NOT_ENGAGED"
            note = "advanced %.1f mm, taught depth %.1f mm (no seating sensor: this is not a verified insertion)" % (advance * 1e3, push)
            print("%s: %s" % (outcome, note), flush=True)
            if engaged and not a.no_release:
                print("opening the gripper", flush=True)
                r.gripper(0.0); time.sleep(0.8)
        if a.back_off_m > 0:
            back_off(r, T_EC, hold_R, a.back_off_m, a.speed)
        REC.close(outcome, note)
    except BaseException as e:
        try:
            r.stop()
        except Exception:
            pass
        print("STOPPED: %s" % e, flush=True)
        if (isinstance(e, Blocked) or reached_insertion[0]) and a.back_off_m > 0:      # at the wall: get out of contact before anything else moves
            try:
                back_off(r, T_EC, hold_R, a.back_off_m, a.speed)
            except Exception as err:
                print("back-off failed: %s" % err, flush=True)
        REC.close("STOPPED", str(e) or type(e).__name__); raise SystemExit(1)


if __name__ == "__main__":
    main()
