#!/usr/bin/env python3
"""method 2's nearby-view CAD pickup (its scripts, plus our --allow-pickup flag) inside a run folder under ~/kinova/work/grasp_runs.

   python3 ~/kinova/work/tools/run_method2_full.py            prints what it would do, moves nothing
   python3 ~/kinova/work/tools/run_method2_full.py --plan-only   checks the 4 camera legs only (find stage), moves nothing
   python3 ~/kinova/work/tools/run_method2_full.py --find-only --execute   REAL MOTION to the 3 camera views and back home, CAD fit, NO grasp (stage 1: find the plug)
   python3 ~/kinova/work/tools/run_method2_full.py --execute  REAL MOTION and a real grasp: 3 capture views, fit, pregrasp, 80 mm, 20 mm + close, 5 mm lift, 50 mm lift. Stops holding.

Needs ~/kinova/work/tools/run_stack_method2.sh running, the arm at project home, the gripper open, e-stop in reach. Source method 1's ros_env.sh first.
"""
import argparse
import json
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy

MINE = pathlib.Path.home() / "kinova/ins/repo/Code/Grasp"
M2 = pathlib.Path.home() / "kinova/ins/method2_src/Code/Grasp"
sys.path.insert(0, str(MINE / "ros2_iface"))
from robot_ros import Robot  # noqa: E402

HOME = (np.array([3.0266, 356.8712, 93.4037, 271.4634, 272.3364, 277.3673]) + 180) % 360 - 180
FLAGS = ["--execute", "--sideways", "--fit-profile", "supervised_pickup", "--allow-pickup"]
WORK = M2.parents[1] / "Media/Grasp/work"

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--execute", action="store_true")
ap.add_argument("--find-only", action="store_true", help="stop after the CAD fit: camera views and fit, no grasp motion")
ap.add_argument("--plan-only", action="store_true", help="check the camera legs, no motion at all (implies --find-only)")
ap.add_argument("--name", default="usb_method2_pickup")
ap.add_argument("--replay-fit", type=pathlib.Path, help="grasp from this saved, accepted fit directory (method 2 run_pickup_video.py --fit-dir), no new capture")
args = ap.parse_args()
args.find_only = args.find_only or args.plan_only
if not (args.execute or args.plan_only):
    print(__doc__)
    sys.exit(0)

ts = time.strftime("%Y%m%d_%H%M%S")
run = pathlib.Path.home() / "kinova/work/grasp_runs" / ("run_%s_%s" % (ts, args.name))
for sub in ("screenshots", "videos", "plots"):
    (run / sub).mkdir(parents=True)
t_start = time.time()
if args.replay_fit:
    fitdir = args.replay_fit.resolve()
    report = json.loads((fitdir / "fit.json").read_text())
    if report.get("accepted") is not True or not (fitdir / "joint_fit.npz").exists():
        sys.exit("replay fit is not an accepted fit with joint_fit.npz")
    SCRIPT, FLAGS = "ros2_iface/run_pickup_video.py", ["--execute", "--fit-dir", str(fitdir)]
else:
    SCRIPT = "ros2_iface/run_fresh_nearby_pickup.py"
    if args.find_only:
        FLAGS = [f for f in FLAGS if f not in ("--allow-pickup", "--execute")] + (["--plan-scan"] if args.plan_only else ["--execute"])
metrics = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"), mode="method2_saved_fit_replay" if args.replay_fit else ("method2_find_only_plan" if args.plan_only else "method2_find_only" if args.find_only else "method2_full_pickup"), command="python3 %s " % SCRIPT + " ".join(FLAGS),
               grasp_requested=not args.find_only, stages={}, events=[], outcome="INCOMPLETE")


def log(msg):
    line = "[%6.1f s] %s" % (time.time() - t_start, msg)
    print(line, flush=True)
    with open(run / "controller.log", "a") as f:
        f.write(line + "\n")


rclpy.init()
robot = Robot("method2_run_wrapper", background=True)
if not robot.wait_ready(10):
    sys.exit("no driver / controller")


def snap(name):
    m = robot._img.get("color")
    if m is not None:
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        cv2.imwrite(str(run / "screenshots" / (name + ".png")), np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img))


def gates():
    out = {}
    for p in ("dry_run", "pregrasp_only", "allow_grasp"):
        r = subprocess.run(["ros2", "param", "get", "/grasp_node", p], capture_output=True, text=True, timeout=15)
        out[p] = r.stdout.strip().split()[-1] if r.stdout.strip() else "unknown"
    return out


away = float(np.max(np.abs(np.degrees(robot.q) - HOME)))
if away > 2:
    sys.exit("arm is %.1f deg from project home: run move_home_once.py --execute first" % away)
if robot.grip_pos_msr is None or robot.grip_pos_msr > 0.05:
    sys.exit("gripper is not fully open")
if robot.fault:
    sys.exit("arm is in fault")
metrics["gates_before"] = gates()
if metrics["gates_before"] != dict(dry_run="True", pregrasp_only="True", allow_grasp="False"):
    sys.exit("node gates must be dry_run=True, pregrasp_only=True, allow_grasp=False: %s" % metrics["gates_before"])

recorder = subprocess.Popen([sys.executable, str(MINE / "ros2_iface/record.py"), str(run)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2)
log("run folder %s" % run)
log(("PLAN ONLY, no motion: " if args.plan_only else "REAL MOTION to the camera views, no grasp: " if args.find_only else "REAL MOTION and a real grasp: ") + metrics["command"])
snap("00_start")
view, stage_n, current = 0, 0, None
exit_code = None
try:
    with open(run / "console.log", "w") as con:
        proc = subprocess.Popen([sys.executable, str(M2 / SCRIPT), *FLAGS], cwd=M2, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in proc.stdout:
                print(line, end="", flush=True)
                con.write(line)
                con.flush()
                m = re.match(r"Fresh view(\d) /3 saved", line)
                if m:
                    snap("%02d_view%s_saved" % (int(m.group(1)), m.group(1)))
                    metrics["events"].append(dict(t_s=round(time.time() - t_start, 1), event="view %s saved" % m.group(1)))
                m = re.match(r"STAGE: (\S+)", line)
                if m:
                    stage_n += 1
                    if current:
                        metrics["stages"][current]["seconds"] = round(time.time() - metrics["stages"][current].pop("t0"), 1)
                    current = m.group(1)
                    snap("%02d_before_%s" % (10 + stage_n, current.replace(".py", "")))
                    metrics["stages"][current] = dict(t0=time.time())
                    log("stage %s" % current)
            exit_code = proc.wait()
        except BaseException:
            proc.send_signal(signal.SIGINT)
            exit_code = proc.wait(timeout=60)
            raise
finally:
    snap("99_end")
    if current and "t0" in metrics["stages"][current]:
        metrics["stages"][current]["seconds"] = round(time.time() - metrics["stages"][current].pop("t0"), 1)
    recorder.send_signal(signal.SIGTERM)
    try:
        recorder.wait(timeout=20)
    except subprocess.TimeoutExpired:
        recorder.kill()
    t_end = time.time()
    metrics["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    metrics["seconds_total"] = round(t_end - t_start, 1)
    metrics["exit_code"] = exit_code
    metrics["gates_after"] = gates()
    metrics["fault_after"] = bool(robot.fault)
    metrics["gripper_rad_after"] = None if robot.grip_pos_msr is None else round(float(robot.grip_pos_msr), 4)

    def newest(sub):
        ds = [d for d in (WORK / sub).glob("*") if d.is_dir() and d.stat().st_mtime >= t_start]
        return max(ds, key=lambda d: d.stat().st_mtime) if ds else None

    perception, pickup = (args.replay_fit.resolve().parent if args.replay_fit else newest("fresh_nearby_runs")), newest("video_runs")
    if perception:
        shutil.copytree(perception, run / "method2_perception", dirs_exist_ok=True)
        for sub, prefix in (("fit", "fit_"), ("detection", "detect_")):
            for p in (perception / sub).glob("*.png"):
                shutil.copy(p, run / "plots" / (prefix + p.name))
        try:
            fit = json.loads((perception / "fit/fit.json").read_text())
            metrics["fit"] = dict(accepted=fit["accepted"], profile=fit["fit_quality"]["profile"], per_view=fit["per_view"], yaw_spread_deg=fit["near_optimal_yaw_spread_deg"], translation_mm=fit["translation_mm"])
        except Exception as e:
            metrics["fit"] = "unavailable: %r" % e
        try:
            metrics["perception_outcome"] = json.loads((perception / "run.json").read_text()).get("outcome")
        except Exception:
            pass
    if pickup:
        shutil.copytree(pickup, run / "method2_pickup", dirs_exist_ok=True)
        try:
            metrics["pickup_outcome"] = json.loads((pickup / "run.json").read_text()).get("outcome")
            metrics["pickup_stage_exit_codes"] = {s["script"]: s["exit_code"] for s in json.loads((pickup / "run.json").read_text()).get("stages", [])}
        except Exception:
            pass
    metrics["outcome"] = (metrics.get("pickup_outcome") or metrics.get("perception_outcome") or "NO_METHOD2_RUN_RECORD") + ("" if exit_code == 0 else " | runner exit code %s" % exit_code)
    for src, dst in ((M2 / "config/grasp.yaml", "config_grasp.yaml"), (M2 / "config/nearby_camera_poses_sideways.json", "config_nearby_camera_poses.json")):
        if src.exists():
            shutil.copy(src, run / dst)
    diff = subprocess.run(["git", "-C", str(M2), "diff"], capture_output=True, text=True).stdout
    (run / "method2_code_changes.diff").write_text(diff)
    driver = M2 / "logs/humble/driver.log"
    if driver.exists():
        keep = []
        for ln in driver.read_text(errors="replace").splitlines():
            m = re.search(r"\[(17\d{8})\.\d+\]", ln)
            if m and t_start - 5 <= float(m.group(1)) <= t_end + 5 and "No controller active" not in ln:
                keep.append(ln)
        (run / "stack_driver.log").write_text("\n".join(keep) + "\n")
    metrics["physical_grip_verified"] = "NOT verified by software: judge from videos/camera.mp4 and the operator's observation"
    (run / "metrics.json").write_text(json.dumps(metrics, indent=1))
    fit = metrics.get("fit") if isinstance(metrics.get("fit"), dict) else None
    lines = ["# Run %s: method 2's nearby-view CAD pickup on our arm (Humble)\n" % run.name,
             "**Result (software):** %s\n" % metrics["outcome"],
             "Physical holding is NOT verified by software. Operator observation: _to be filled in_.\n",
             "## What ran\n", "`%s`, with method 2's scripts unchanged except the `--allow-pickup` flag (see `method2_code_changes.diff`). Stack: `run_stack_method2.sh` (Humble driver, MoveIt, D405 with enable_sync, method 2's grasp node). Hand-eye and camera poses as in `config_*`.\n",
             ("Sequence (find only): 3 nearby Cartesian camera views -> home -> CAD fit. No grasp motion, the gripper does not move.\n" if args.find_only else "Sequence: 3 nearby Cartesian camera views -> home -> CAD fit -> pregrasp (100 mm) -> jaw opening -> 80 mm approach -> final 20 mm + close -> 5 mm lift -> 50 mm lift. No release.\n")]
    if fit:
        lines += ["## Fit\n", "Profile `%s`, accepted: %s, yaw spread %.2f deg.\n" % (fit["profile"], fit["accepted"], fit["yaw_spread_deg"]),
                  "| view | points | within 2 mm | median / p95 (mm) |\n|---|---|---|---|\n"]
        lines += ["| %s | %d | %.1f %% | %.2f / %.2f |\n" % (v["view"], v["points"], v["within_2mm_percent"], *v["median_p95_mm"]) for v in fit["per_view"]]
    lines += ["\n## Files\n", "`metrics.json` (stages, gates, fit, outcome), `controller.log` (timeline), `console.log` (method 2's scripts, verbatim), `trajectory.csv` and `videos/camera.mp4` (10 Hz recorder), `screenshots/`, `plots/` (fit and detection figures), `method2_perception/` (raw views, fit, ROIs), `method2_pickup/` (stage logs, plans, attempt records), `stack_driver.log` (controller goals during the run).\n"]
    (run / "README.md").write_text("\n".join(lines))
    log("outcome: %s" % metrics["outcome"])
    log("saved to %s" % run)
    robot.destroy_node()
    rclpy.shutdown()
import os
sys.stdout.flush()
os._exit(0 if exit_code == 0 else 1)   # a plain exit aborts on the helper thread of the Robot node
