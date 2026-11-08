#!/usr/bin/env python3
"""Colour-marker pipeline end to end, with data collection into a Media/Grasp run folder.

python3 ros2_iface/run_pipeline.py RUN_DIR [--grasp] [--skip-scan] [--mode sticker|head|cloud]

home -> scan 15 views (colour) -> merge + ICP -> table RANSAC, clusters, marker clusters (coarse target)
     -> camera over the target -> marker-sticker grasp (close view) -> [--grasp] execute (+ wrist-camera hold check).
Records camera video + joint trajectory the whole time. Needs the stack and the grasp node (dry_run:=false) running.
"""
import argparse
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from std_srvs.srv import Trigger

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
WORK = ROOT.parents[1] / "Media" / "Grasp" / "work"  # where the node writes its scans / plan frames (same as grasp_node.py)
sys.path.insert(0, str(HERE))
from robot_ros import Robot  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run_dir")
ap.add_argument("--grasp", action="store_true")
ap.add_argument("--skip-scan", action="store_true")
ap.add_argument("--mode", choices=("sticker", "head", "cloud"), default="sticker", help="sticker = sticker + colour footprint (default); head = head cloud + candidates; cloud = marker region only + candidate search")
args = ap.parse_args()
RUN = pathlib.Path(args.run_dir)
for sub in ("screenshots", "videos", "plots"):
    (RUN / sub).mkdir(parents=True, exist_ok=True)
T0 = time.time()
metrics = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"), grasp_requested=args.grasp, stages={}, outcome="INCOMPLETE")


def log(msg):
    line = "[%6.1f s] %s" % (time.time() - T0, msg)
    print(line, flush=True)
    with open(RUN / "controller.log", "a") as f:
        f.write(line + "\n")


rclpy.init()
robot = Robot("pipeline", background=True)
assert robot.wait_ready(10), "no driver / controller"
rec = subprocess.Popen([sys.executable, str(HERE / "record.py"), str(RUN)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
CLI = {}


def call(name, timeout):
    cli = CLI.setdefault(name, robot.create_client(Trigger, "/grasp/" + name))
    assert cli.wait_for_service(10.0), "service /grasp/%s not available" % name
    t = time.time()
    fut = cli.call_async(Trigger.Request())
    while not fut.done() and time.time() - t < timeout:
        time.sleep(0.2)
    if not fut.done():
        raise TimeoutError("/grasp/%s took longer than %d s" % (name, timeout))
    r = fut.result()
    metrics["stages"][name] = dict(ok=r.success, seconds=round(time.time() - t, 1), message=r.message)
    log("%s -> %s: %s" % (name, "OK" if r.success else "FAILED", r.message))
    return r.success, r.message


def snap(name):
    m = robot._img.get("color")
    if m is None:
        return
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    cv2.imwrite(str(RUN / "screenshots" / (name + ".png")), np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img))


def newest(pattern):
    found = sorted(WORK.glob(pattern), key=lambda p: p.stat().st_mtime)
    return found[-1] if found else None


try:
    snap("00_start")
    q = np.degrees(robot.q)
    home = (np.array(yaml.safe_load(open(ROOT / "config" / "grasp.yaml"))["scan"]["home_joints_deg"]) + 180) % 360 - 180
    if np.abs(q - home).max() > 1.0:
        log("arm is %.0f deg from home: going home first" % np.abs(q - home).max())
        call("home", 120)
    if not args.skip_scan:
        ok, _ = call("scan", 1500)
        assert ok, "scan failed"
    scan_dir = newest("scans/*/marker_cloud.npy").parent
    ok, _ = call("color_target", 300)
    assert ok, "coarse colour-target stage failed"
    ok, _ = call("view", 300)
    assert ok, "camera could not go over the target"
    time.sleep(1.0)
    snap("03_close_view")
    ok, _ = call(dict(sticker="sticker_grasp", head="head_grasp", cloud="cloud_grasp")[args.mode], 300)
    snap("04_after_grasp_plan")
    if not ok:
        metrics["outcome"] = "STOPPED_BEFORE_GRASP: sticker grasp could not be computed"
    elif not args.grasp:
        metrics["outcome"] = "PERCEPTION_ONLY: grasp computed, not requested"
    else:
        call("allow_grasp", 20)
        ok, msg = call("execute", 600)
        snap("05_after_grasp")
        metrics["outcome"] = ("GRASP_REPORTED_HELD" if ok else "GRASP_FAILED") + ": " + msg
except Exception as e:  # keep the evidence of whatever happened
    metrics["outcome"] = "ERROR: %s: %s" % (type(e).__name__, e)
    log(metrics["outcome"])
finally:
    snap("99_end")
    metrics["seconds_total"] = round(time.time() - T0, 1)
    rec.send_signal(signal.SIGTERM)
    rec.wait(timeout=20)
    sc, rn = newest("scans/*/marker_cloud.npy"), newest("runs/2*/plan.json")
    if sc is not None and not args.skip_scan:
        shutil.copytree(sc.parent, RUN / "scan" / sc.parent.name, dirs_exist_ok=True)
    elif sc is not None:
        (RUN / "scan_used.txt").write_text(str(sc.parent) + "\n")
    if rn is not None:
        shutil.copytree(rn.parent, RUN / "plan" / rn.parent.name, dirs_exist_ok=True)
        for p in (rn.parent / "plots").glob("*.png"):
            shutil.copy(p, RUN / "plots" / ("plan_" + p.name))
    if sc is not None:
        for p in (sc.parent / "plots").glob("*.png"):
            shutil.copy(p, RUN / "plots" / ("scan_" + p.name))
    shutil.copy(ROOT / "config" / "grasp.yaml", RUN / "config_grasp.yaml")
    shutil.copy(ROOT / "logs" / "grasp_node.log", RUN / "grasp_node.log")
    json.dump(metrics, open(RUN / "metrics.json", "w"), indent=1)
    log("outcome: " + metrics["outcome"])
    os._exit(0)
