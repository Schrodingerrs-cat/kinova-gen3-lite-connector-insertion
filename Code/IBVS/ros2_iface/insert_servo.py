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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402
from run_record import Recorder, git_state  # noqa: E402

REC = None                                              # run recorder, set in main


def record(r, stage, color, c, goal, d, side, cmd=None):
    if REC is not None:
        REC.step(stage, color, c, goal, d, side, cmd, r.tool_matrix()[:3, 3] * 1e3, np.degrees(r.q))


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


def observe(r, marker_id, tries=30):
    """Marker corners (4x2 px) from a frame taken after the arm stopped, or raise."""
    r.spin_until(lambda: False, 0.35)
    for _ in range(tries):
        r.spin_until(lambda: False, 0.05)
        try:
            color, _, K = r.grab()
        except Exception:
            continue
        corners, ids, _ = detect_markers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        if ids is not None and marker_id in ids.ravel():
            c = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2)
            return c, np.asarray(K, float), color
    raise RuntimeError("marker %d not seen: stopping" % marker_id)


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


def stage(r, T_EC, hold_R, marker_id, goal, L, speed, step_m, tol_m, name):
    """Stop-and-observe: quintic chunks of at most step_m, each ending at rest."""
    while True:
        c, K, color = observe(r, marker_id)
        d, side = offset(c, K, goal, L)
        n = float(np.linalg.norm(d))
        print("%s: marker %.0f px, still %.1f mm to go (x %+.1f y %+.1f z %+.1f)" % (name, side, n * 1e3, *(d * 1e3)), flush=True)
        if n < tol_m:
            record(r, name, color, c, goal, d, side)
            return
        step = d * min(1.0, step_m / n)
        T_goal = goal_pose(r, T_EC, hold_R, step)
        cmd = T_goal[:3, 3] - r.tool_matrix()[:3, 3]
        print("  step in base frame (mm): %s, tool height %.0f mm" % (np.round(cmd * 1e3, 1), r.tool_matrix()[2, 3] * 1e3), flush=True)
        record(r, name, color, c, goal, d, side, cmd)
        try:
            r.send_line(line(r, T_goal), max(0.6, float(np.linalg.norm(step)) / speed))
        except NoPath as e:
            recover(r, e.T_goal, speed)


def stream(r, pts, h, v):
    """Send the next h metres of a line at speed v, ending at cruise speed (not at rest), without waiting: the next frame replaces it.
    The path was planned a moment ago, so the arm is already past its first points: start from the joints now and drop the points behind."""
    P = np.array(pts)
    joint_len = np.linalg.norm(np.diff(P, axis=0), axis=1)
    q_now = r.q.copy()
    ahead = P[int(np.argmin(np.linalg.norm(P - q_now, axis=1))) + 1:]
    if len(ahead) == 0:
        return
    P = np.vstack([q_now, ahead])
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    T = max(0.15, h * seg.sum() / max(joint_len.sum(), 1e-9) / v)          # time for the part still ahead
    t = np.r_[0.0, np.cumsum(seg)] / max(seg.sum(), 1e-9) * T
    V = np.gradient(P, t, axis=0)
    traj = JointTrajectory(); traj.joint_names = JOINTS
    for q, vel, ti in zip(P, V, t + 0.02):
        pt = JointTrajectoryPoint(); pt.positions = [float(x) for x in q]; pt.velocities = [float(x) for x in vel]
        pt.time_from_start = Duration(sec=int(ti), nanosec=int((ti % 1) * 1e9)); traj.points.append(pt)
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    r.arm.send_goal_async(goal).add_done_callback(lambda f: setattr(r, "_goal", f.result()))


def stage_continuous(r, T_EC, hold_R, marker_id, goal, L, speed, tol_m, name, horizon=1.0):
    """Re-plan on every camera frame while moving: speed follows the remaining distance (cruise at `speed`, slowing into the goal)."""
    lost, last = 0, 0.0
    while True:
        try:
            r.spin_until(lambda: False, 0.02)
            color, _, K = r.grab()
            corners, ids, _ = detect_markers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
            if ids is None or marker_id not in ids.ravel():
                raise RuntimeError("marker not seen")
            c, K = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2), np.asarray(K, float)
        except RuntimeError:
            lost += 1
            if lost > 6:
                raise RuntimeError("marker %d lost: stopping" % marker_id)
            continue
        lost = 0
        d, side = offset(c, K, goal, L)
        n = float(np.linalg.norm(d))
        if time.time() - last > 0.5:
            print("%s: marker %.0f px, still %.1f mm to go, camera-frame offset (mm) %s, tool height %.0f mm" % (name, side, n * 1e3, np.round(d * 1e3, 1), r.tool_matrix()[2, 3] * 1e3), flush=True); last = time.time()
        if n < tol_m:
            record(r, name, color, c, goal, d, side)
            r.stop(); r.send_joint_path([r.q.copy()], [0.4], [np.zeros(6)])
            return
        v = min(speed, max(0.01, 2.0 * n))
        h = min(n, v * horizon)
        try:
            T_goal = goal_pose(r, T_EC, hold_R, d * (h / n))
            record(r, name, color, c, goal, d, side, T_goal[:3, 3] - r.tool_matrix()[:3, 3])
            stream(r, line(r, T_goal), h, v)
        except NoPath as e:
            recover(r, e.T_goal, speed)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wire", required=True, choices=("aux", "usb", "power"))
    ap.add_argument("--speed", type=float, default=0.05, help="average tool speed, m/s (grasp pipeline line speed)")
    ap.add_argument("--step-m", type=float, default=0.04, help="re-observe the marker after at most this much travel")
    ap.add_argument("--tol-mm", type=float, default=2.0)
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
            if a.continuous:
                stage_continuous(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.tol_mm / 1e3, name)
            else:
                stage(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.step_m, a.tol_mm / 1e3, name)
            if name == "pre-insertion":
                time.sleep(a.pause)
        print("at the socket entrance (%s)" % a.wire, flush=True)
        REC.close("COMPLETED_AT_ENTRANCE")
    except BaseException as e:
        try:
            r.stop()
        except Exception:
            pass
        print("STOPPED: %s" % e, flush=True)
        REC.close("STOPPED", str(e) or type(e).__name__); raise SystemExit(1)


if __name__ == "__main__":
    main()
