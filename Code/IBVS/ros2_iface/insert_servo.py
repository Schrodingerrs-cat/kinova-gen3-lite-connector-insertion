#!/usr/bin/env python3
"""Two-stage insertion servo on one printed marker, translation only (the gripper keeps the post-grasp orientation).
  python3 ros2_iface/insert_servo.py --wire aux|usb|power [--speed 0.05] [--step-m 0.04]
Each cycle: detect the wire's marker, turn its pixel centre + side length into a 3-D offset of the camera from where it must be
(centre/side from the reference images in config/insertion_refs.yaml), and run a quintic Cartesian line of the tool by that offset
(MoveIt path, same smooth profile as the grasp pipeline). Stage 1 -> pre-insertion image, brief stop, stage 2 -> insertion image."""
import argparse, pathlib, subprocess, sys, time
import cv2, numpy as np, yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / "Grasp")); sys.path.insert(0, str(ROOT.parent / "Grasp" / "ros2_iface"))
import rclpy  # noqa: E402
from robot_ros import Robot, matrix, JOINTS  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402


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
            return c, np.asarray(K, float)
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
        c, K = observe(r, marker_id)
        d, side = offset(c, K, goal, L)
        n = float(np.linalg.norm(d))
        print("%s: marker %.0f px, still %.1f mm to go (x %+.1f y %+.1f z %+.1f)" % (name, side, n * 1e3, *(d * 1e3)), flush=True)
        if n < tol_m:
            return
        step = d * min(1.0, step_m / n)
        T_goal = goal_pose(r, T_EC, hold_R, step)
        try:
            r.send_line(line(r, T_goal), max(0.6, float(np.linalg.norm(step)) / speed))
        except NoPath as e:
            recover(r, e.T_goal, speed)


def stream(r, pts, T):
    """Send a short trajectory along the joint waypoints of a line, ending at cruise speed (not at rest), without waiting: the next frame replaces it."""
    P = np.array(pts)
    if len(P) < 2:
        return
    t = np.linspace(0.0, T, len(P))
    V = np.gradient(P, t, axis=0)
    traj = JointTrajectory(); traj.joint_names = JOINTS
    for q, v, ti in zip(P, V, t + 0.05):
        pt = JointTrajectoryPoint(); pt.positions = [float(x) for x in q]; pt.velocities = [float(x) for x in v]
        pt.time_from_start = Duration(sec=int(ti), nanosec=int((ti % 1) * 1e9)); traj.points.append(pt)
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    r.arm.send_goal_async(goal).add_done_callback(lambda f: setattr(r, "_goal", f.result()))


def stage_continuous(r, T_EC, hold_R, marker_id, goal, L, speed, tol_m, name, horizon=0.6):
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
            print("%s: marker %.0f px, still %.1f mm to go" % (name, side, n * 1e3), flush=True); last = time.time()
        if n < tol_m:
            r.stop(); r.send_joint_path([r.q.copy()], [0.4], [np.zeros(6)])
            return
        v = min(speed, max(0.01, 2.0 * n))
        h = min(n, v * horizon)
        try:
            stream(r, line(r, goal_pose(r, T_EC, hold_R, d * (h / n))), h / v)
        except NoPath as e:
            recover(r, e.T_goal, speed)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wire", required=True, choices=("aux", "usb", "power"))
    ap.add_argument("--speed", type=float, default=0.05, help="average tool speed, m/s (grasp pipeline line speed)")
    ap.add_argument("--step-m", type=float, default=0.04, help="re-observe the marker after at most this much travel")
    ap.add_argument("--tol-mm", type=float, default=2.0)
    ap.add_argument("--continuous", action="store_true", help="re-plan on every frame while moving instead of stopping to look")
    ap.add_argument("--pause", type=float, default=1.0, help="stop at the pre-insertion pose, s")
    a = ap.parse_args()
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
    try:
        for name, goal in (("pre-insertion", w["pre"]), ("insertion", ins)):
            if a.continuous:
                stage_continuous(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.tol_mm / 1e3, name)
            else:
                stage(r, T_EC, hold_R, w["marker_id"], goal, L, a.speed, a.step_m, a.tol_mm / 1e3, name)
            if name == "pre-insertion":
                time.sleep(a.pause)
        print("at the socket entrance (%s)" % a.wire, flush=True)
    except BaseException as e:
        r.stop(); print("STOPPED: %s" % e, flush=True); raise SystemExit(1)


if __name__ == "__main__":
    main()
