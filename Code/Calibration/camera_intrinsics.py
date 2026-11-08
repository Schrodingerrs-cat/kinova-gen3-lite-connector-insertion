#!/usr/bin/env python3
"""Camera intrinsics check with a chessboard: calibrate K and distortion from your own images and compare with the factory values in camera_info.

python3 camera_intrinsics.py            interactive: hold / place the chessboard in front of the camera (the camera can be on the arm, keep it still per capture)
python3 camera_intrinsics.py --from F   solve again from saved corner sets (no camera)

Fill calibration.yaml `chessboard.square_size_m` (measured) and `inner_corners` (columns x rows of INNER corners). Keys: c capture, u undo, s solve + save, q quit.
Capture 15+ views: board near / far, tilted both ways, covering the whole image incl. the corners. A good result has an RMS reprojection error below ~0.5 px.
"""
import argparse
import json
import pathlib
import sys
import time

import cv2
import numpy as np
import yaml

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib_lib as cl  # noqa: E402

cfg = yaml.safe_load(open(HERE / "calibration.yaml"))


def solve(img_pts, size, factory_K, prefix):
    if len(img_pts) < 10:
        print("need at least 10 captures (have %d)" % len(img_pts))
        return
    cb = cfg["chessboard"]
    obj = cl.chessboard_points(cb["inner_corners"][0], cb["inner_corners"][1], cb["square_size_m"])
    rms, K, dist = cl.intrinsics([obj] * len(img_pts), [np.array(p, np.float32) for p in img_pts], tuple(size))
    print("\nRMS reprojection error %.3f px over %d images (below ~0.5 px is good)" % (rms, len(img_pts)))
    print("calibrated : fx %.1f fy %.1f cx %.1f cy %.1f, distortion %s" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], np.round(dist, 4)))
    if factory_K is not None:
        print("factory    : fx %.1f fy %.1f cx %.1f cy %.1f" % (factory_K[0, 0], factory_K[1, 1], factory_K[0, 2], factory_K[1, 2]))
        print("difference : fx %+.1f fy %+.1f cx %+.1f cy %+.1f px" % (K[0, 0] - factory_K[0, 0], K[1, 1] - factory_K[1, 1], K[0, 2] - factory_K[0, 2], K[1, 2] - factory_K[1, 2]))
    res = HERE / "results" / (prefix + "_result.json")
    res.write_text(json.dumps(dict(n_images=len(img_pts), rms_px=rms, K=K.tolist(), distortion=dist.tolist(), factory_K=None if factory_K is None else factory_K.tolist(), image_size=list(size)), indent=1))
    print("saved", res)


def capture_session():
    import rclpy
    from console import Keys
    from robot_ros import Robot
    cb = cfg["chessboard"]
    if cb["square_size_m"] is None:
        sys.exit("set chessboard.square_size_m in calibration.yaml (measured side of one square)")
    rclpy.init()
    robot = Robot("intrinsics", background=True)
    stamp = time.strftime("intrinsics_%Y%m%d_%H%M%S")
    pts, size, factory = [], None, None
    pattern = tuple(cb["inner_corners"])
    print(__doc__.split("Fill calibration")[1])
    with Keys() as keys:
        while True:
            k = keys.get()
            if k == "c":
                color, _, K = robot.grab()
                gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
                ok, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
                if not ok:
                    print("chessboard (%dx%d inner corners) not found" % pattern, flush=True)
                    continue
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                pts.append(corners.reshape(-1, 2).tolist()); size, factory = (gray.shape[1], gray.shape[0]), K
                (HERE / "results" / (stamp + "_corners.json")).write_text(json.dumps(dict(image_size=list(size), factory_K=K.tolist(), corners=pts)))
                print("capture %d ok" % len(pts), flush=True)
            elif k == "u" and pts:
                pts.pop(); print("removed, %d captures" % len(pts), flush=True)
            elif k == "s":
                solve(pts, size, factory, stamp)
            elif k == "q":
                break
    import os
    os._exit(0)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", metavar="FILE")
    a = ap.parse_args()
    if a.src:
        d = json.load(open(a.src))
        if cfg["chessboard"]["square_size_m"] is None:
            sys.exit("set chessboard.square_size_m in calibration.yaml")
        solve(d["corners"], d["image_size"], None if d.get("factory_K") is None else np.array(d["factory_K"]), pathlib.Path(a.src).stem)
    else:
        capture_session()
