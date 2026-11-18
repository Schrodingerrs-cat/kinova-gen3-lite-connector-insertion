#!/usr/bin/env python3
"""Hand-eye calibration (tool_frame -> camera) from an ArUco marker fixed on the table.

python3 hand_eye_calibration.py            interactive: you move the arm (Code/Teleop in another terminal), this tool captures and solves
python3 hand_eye_calibration.py --from F   solve again from saved samples (no robot, no camera)

Setup: print an ArUco marker (calibration.yaml `aruco`), measure the BLACK square with a ruler and put it in `aruco.marker_size_m`, stick it FLAT on the
table and never move it. Needs the arm driver (TF) and the camera. Keys: c capture, u undo last, s solve + save, q quit.
Capture 12+ poses with the marker visible and the camera ROTATED between poses (tilt and yaw changes, not only translations).
It never edits the grasp config: it prints the two lines to paste.
"""
import argparse
import json
import pathlib
import sys
import time

import numpy as np
import yaml

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib_lib as cl  # noqa: E402

cfg = yaml.safe_load(open(HERE / "calibration.yaml"))
current = cl.to_T(cfg["camera"]["hand_eye_xyz_m"], cfg["camera"]["hand_eye_quat_xyzw"])


def solve(T_bt, T_cm, out_prefix):
    if len(T_bt) < 6:
        print("need at least 6 samples (have %d)" % len(T_bt))
        return
    from scipy.spatial.transform import Rotation as Rot
    spread = max((Rot.from_matrix(a[:3, :3]) * Rot.from_matrix(b[:3, :3]).inv()).magnitude() for a in T_bt for b in T_bt)
    if np.degrees(spread) < 30:
        print("WARNING: the tool rotations differ by only %.0f deg: rotate the camera more between captures, the result will be poor" % np.degrees(spread))
    X = cl.hand_eye(T_bt, T_cm)
    new, old = cl.hand_eye_residual(T_bt, T_cm, X), cl.hand_eye_residual(T_bt, T_cm, current)
    mm, deg = cl.pose_difference(X, current)
    xyz, quat = cl.xyz_quat(X)
    print("\nNEW hand-eye   : marker position spread in the base frame: std %s mm, max %.1f mm, max angle %.2f deg" % (np.round(new["std_mm"], 2), new["max_dev_mm"], new["max_angle_deg"]))
    print("CURRENT hand-eye: marker position spread in the base frame: std %s mm, max %.1f mm, max angle %.2f deg" % (np.round(old["std_mm"], 2), old["max_dev_mm"], old["max_angle_deg"]))
    print("difference new - current: %.1f mm, %.2f deg" % (mm, deg))
    print("lower spread = better. Paste into Code/Grasp/config/grasp.yaml (camera:) only if NEW is clearly better:")
    print("  hand_eye_xyz_m: %s\n  hand_eye_quat_xyzw: %s" % ([round(v, 7) for v in xyz], [round(v, 7) for v in quat]))
    res = HERE / "results" / (out_prefix + "_result.json")
    res.write_text(json.dumps(dict(n_samples=len(T_bt), hand_eye_xyz_m=xyz, hand_eye_quat_xyzw=quat, new_spread=new, current_spread=old, difference_mm_deg=[mm, deg]), indent=1))
    print("saved", res)


def load(path):
    d = json.load(open(path))
    return [np.array(a) for a in d["T_base_tool"]], [np.array(a) for a in d["T_cam_marker"]]


def capture_session(resume=None):
    import rclpy
    from console import Keys
    from robot_ros import Robot
    a = cfg["aruco"]
    if a["marker_size_m"] is None:
        sys.exit("set aruco.marker_size_m in calibration.yaml (side of the black square, measured with a ruler)")
    from burst_capture import setup, capture
    rclpy.init()
    robot = Robot("hand_eye_calibration", background=False)
    subscription=setup(robot)
    if resume is not None:
        stored=json.loads(resume.read_text())
        if stored.get("aruco")!=a:
            raise ValueError("Tag settings changed; start a new session")
        stamp=resume.name.removesuffix("_samples.json")
        folder=resume.parent/stamp
        if not folder.is_dir():raise ValueError("Resume evidence folder missing")
        T_bt=[np.array(t) for t in stored["T_base_tool"]]
        T_cm=[np.array(t) for t in stored["T_cam_marker"]]
        accepted=stored["captures"]
        attempts=max([int(p.name.split("_")[-1]) for p in folder.glob("attempt_*")],default=0)
    else:
        stamp=time.strftime("hand_eye_burst_%Y%m%d_%H%M%S")
        folder=HERE/"results"/stamp
        folder.mkdir(parents=True,exist_ok=False)
        T_bt,T_cm,accepted=[],[],[]
        attempts=0
    def save():
        (HERE/"results"/(stamp+"_samples.json")).write_text(json.dumps(dict(
            method="8 fresh colour frames, subpixel corners, image-timestamp robot TF",
            aruco=a,captures=accepted,T_base_tool=[t.tolist() for t in T_bt],
            T_cam_marker=[t.tolist() for t in T_cm]),indent=2))
    save()
    print(("RESUMED BURST SESSION: " if resume else "NEW BURST SESSION: ")+stamp+" (%d saved samples)"%len(T_bt),flush=True)
    print("c: capture 8 frames while stationary | u: undo | s: solve | q: save and quit. No robot motion.",flush=True)
    print("Keep entire tag visible with >=8px margin. Tag and robot base must remain fixed.",flush=True)
    try:
        with Keys() as keys:
            while True:
                k=keys.get()
                if k=="c":
                    attempts+=1
                    print("Capturing stationary burst...",flush=True)
                    try:
                        BT,CM,quality=capture(robot,a,folder/("attempt_%03d"%attempts))
                    except Exception as exc:
                        print("CAPTURE REJECTED: "+str(exc),flush=True)
                        continue
                    T_bt.append(BT);T_cm.append(CM);accepted.append(dict(attempt=attempts,quality=quality))
                    save()
                    print("sample %d captured: 8 frames, reprojection %.2f px, corner std %.2f px, marker %.0f mm" % (
                        len(T_bt),quality['reprojection_rms_px'],quality['corner_std_max_px'],CM[2,3]*1000),flush=True)
                elif k=="u" and T_bt:
                    T_bt.pop();T_cm.pop();accepted.pop();save()
                    print("removed; %d samples"%len(T_bt),flush=True)
                elif k=="s":solve(T_bt,T_cm,stamp)
                elif k=="q":break
    finally:
        save()
        robot.destroy_subscription(subscription)
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", metavar="FILE")
    ap.add_argument("--resume-latest",action="store_true",help="resume latest burst session only if tag/base/mount stayed fixed")
    args = ap.parse_args()
    if args.src and args.resume_latest:ap.error("choose --from or --resume-latest")
    if args.src:
        solve(*load(args.src), pathlib.Path(args.src).stem)
    else:
        resume=None
        if args.resume_latest:
            sessions=list((HERE/"results").glob("hand_eye_burst_*_samples.json"))
            if not sessions:ap.error("no burst session to resume")
            resume=max(sessions,key=lambda p:p.stat().st_mtime)
        capture_session(resume)
