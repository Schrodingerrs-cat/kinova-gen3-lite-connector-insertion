#!/usr/bin/env python3
"""Green-marker pipeline end to end, with data collection into a Media/Grasp run folder.

python3 ros2_iface/run_green_pipeline.py RUN_DIR [--grasp] [--skip-scan] [--depth-plan]

home -> scan 15 views (colour) -> merge + ICP -> table RANSAC, clusters, green clusters (coarse target)
     -> camera over the target -> green ROI in the close view -> housing grasp plan -> gates -> [--grasp] execute.
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
sys.path.insert(0, str(HERE))
from robot_ros import Robot  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run_dir")
ap.add_argument("--grasp", action="store_true")
ap.add_argument("--skip-scan", action="store_true")
ap.add_argument("--depth-plan", action="store_true", help="old depth-based housing planner + gates instead of the green-sticker grasp")
ap.add_argument("--no-gates", action="store_true", help="log the gates but do not stop on them")
args = ap.parse_args()
args.green_grasp = not args.depth_plan  # MAIN pipeline = green-sticker grasp
RUN = pathlib.Path(args.run_dir)
for sub in ("screenshots", "videos", "plots"):
    (RUN / sub).mkdir(parents=True, exist_ok=True)
T0 = time.time()
metrics = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"), grasp_requested=args.grasp, stages={}, gates=[], outcome="INCOMPLETE")


def log(msg):
    line = "[%6.1f s] %s" % (time.time() - T0, msg)
    print(line, flush=True)
    with open(RUN / "controller.log", "a") as f:
        f.write(line + "\n")


rclpy.init()
robot = Robot("green_pipeline", background=True)
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
    found = sorted(ROOT.glob(pattern), key=lambda p: p.stat().st_mtime)
    return found[-1] if found else None


def gates(plan_dir):
    """Sanity gates before any grasp motion. Returns [(name, ok, detail)]."""
    for _ in range(40):  # plots / metrics are written by a background thread
        if (plan_dir / "metrics_pipeline.json").exists() and (plan_dir / "green_close.json").exists():
            break
        time.sleep(0.5)
    met = json.load(open(plan_dir / "metrics_pipeline.json"))
    gc = json.load(open(plan_dir / "green_close.json"))
    out = [("plan found a grasp", bool(met["ok"]), "n_grasps=%s" % met["n_grasps"])]
    if not met["ok"]:
        return out
    L, W, H = met["housing_LWH_mm"]
    out.append(("housing size plausible (L 10-60, W 6-35, H 3-35 mm)", 10 <= L <= 60 and 6 <= W <= 35 and 3 <= H <= 35, "%.1f x %.1f x %.1f mm" % (L, W, H)))
    tcp, c = np.array(met["best_grasp_tcp_base_mm"]), np.array(gc["centroid_m"]) * 1e3
    out.append(("grasp point within 30 mm (xy) of the green patch", float(np.linalg.norm(tcp[:2] - c[:2])) <= 30, "grasp %s mm, green %s mm" % (tcp.round(1).tolist(), c.round(1).tolist())))
    drop = c[2] - tcp[2]
    out.append(("grasp height 0-25 mm below the green top face (support surface sanity)", 0 <= drop <= 25, "green top at z=%.1f mm, grasp z=%.1f mm, drop %.1f mm" % (c[2], tcp[2], drop)))
    out.append(("jaw opening <= 60 mm", met["best_grasp_open_width_mm"] <= 60, "%.1f mm" % met["best_grasp_open_width_mm"]))
    out.append(("enough green points in the close view (>= 100)", gc["n_points"] >= 100, "%d points" % gc["n_points"]))
    return out


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
    scan_dir = newest("scans/*/green_cloud.npy").parent
    ok, _ = call("green", 300)
    assert ok, "coarse green stage failed"
    ok, _ = call("view", 300)
    assert ok, "camera could not go over the target"
    time.sleep(1.0)
    snap("03_close_view")
    ok, _ = call("green_grasp" if args.green_grasp else "green_plan", 300)
    plan_dir = newest("runs/2*/plan.json").parent
    snap("04_after_plan")
    g = [] if args.green_grasp else gates(plan_dir)
    metrics["gates"] = [dict(name=n, ok=bool(o), detail=d) for n, o, d in g]
    for n, o, d in g:
        log("gate %-55s %s  %s" % (n, "PASS" if o else "FAIL", d))
    if not ok or (not all(o for _, o, _ in g) and not args.no_gates):
        metrics["outcome"] = "STOPPED_BEFORE_GRASP: " + ("plan failed" if not ok else "a gate failed")
    elif not args.grasp:
        metrics["outcome"] = "PERCEPTION_ONLY: gates passed, grasp not requested"
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
    sc, rn = newest("scans/*/green_cloud.npy"), newest("runs/2*/plan.json")
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
