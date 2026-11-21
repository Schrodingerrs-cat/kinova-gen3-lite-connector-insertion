"""Real backend: Grasp pipeline (Pipeline.run_one), IBVS runner (subprocess), straight-line insertion. Needs the stack from Grasp/run_ros.sh dry_run:=false."""
import json
import pathlib
import re
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from core import Backend, StageResult

CODE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CODE / "Grasp" / "ros2_iface"))
sys.path.insert(0, str(CODE / "Grasp"))
import pipeline  # noqa: E402
import run_pipeline  # noqa: E402
from housing_grasp import connectors, strip_config  # noqa: E402
from robot_ros import Robot  # noqa: E402

IBVS_OK = ("CONVERGED", "ALREADY_ALIGNED", "PREVIEW")
LOST = re.compile(r"marker|tracking|visible|lost", re.I)
CAMERAS = dict(   # Grasp reads /camera/camera, IBVS reads /eih_cam: one device, so swap the driver between stages
    grasp=("/camera/camera/color/image_raw", ["align_depth.enable:=true", "pointcloud.enable:=true"]),
    ibvs=("/eih_cam/color/image_raw", ["align_depth.enable:=true", "camera_name:=eih_cam", "camera_namespace:=/", "tf_prefix:=eih_cam",
                                       "depth_module.color_profile:=848x480x30", "depth_module.depth_profile:=848x480x30"]))


class Camera:
    def __init__(self, robot, log_dir):
        self.log_dir, self.proc, self.frames = log_dir, None, {}
        for name, (topic, _) in CAMERAS.items():
            self.frames[name] = 0
            robot.create_subscription(Image, topic, lambda m, n=name: self.frames.__setitem__(n, self.frames[n] + 1), qos_profile_sensor_data)

    def streaming(self, name):
        n = self.frames[name]
        time.sleep(1.5)
        return self.frames[name] > n

    def use(self, name):
        if self.streaming(name):
            return
        for pat in ("rs_launch.py", "realsense2_camera_node"):
            subprocess.run(["pkill", "-f", pat])
        time.sleep(3)
        log = open(self.log_dir / ("camera_%s.log" % name), "a")
        self.proc = subprocess.Popen(["ros2", "launch", "realsense2_camera", "rs_launch.py", *CAMERAS[name][1]], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        end = time.time() + 40
        while time.time() < end:
            if self.streaming(name):
                return
        raise RuntimeError("camera %s did not start, see camera_%s.log" % (name, name))

    def close(self):
        if self.proc:
            self.proc.send_signal(signal.SIGINT)


class RosBackend(Backend):
    def __init__(self, cfg, live, run_dir):
        self.cfg, self.live, self.dir = cfg, live, pathlib.Path(run_dir)
        self.home = (np.array(yaml.safe_load(open(CODE / "Grasp/config/grasp.yaml"))["scan"]["home_joints_deg"]) + 180) % 360 - 180
        rclpy.init()
        self.robot = Robot("harness", background=True)
        if not self.robot.wait_ready(10):
            sys.exit("no driver / controller")
        self.io = run_pipeline.RosIo(self.robot)
        self.cam = Camera(self.robot, self.dir)
        self.ibvs_proc = None

    def preflight(self, wires):
        bad = []
        goals = yaml.safe_load(open(CODE / "IBVS/config/taught_goals.yaml")) or {}
        goals = goals.get("goals", goals)
        ins = self.cfg["insert"]
        if self.live and (ins.get("depth_m") is None or ins.get("axis_tool") is None):
            bad.append("insert.depth_m / axis_tool not measured (null): live insertion blocked")
        for w in wires:
            if w.socket not in goals:
                bad.append("no taught IBVS goal for socket '%s' (run IBVS/run_teach_goal.sh --socket %s)" % (w.socket, w.socket))
            try:
                (strip_config if self.cfg["pick"]["modes"][0] == "strip" else connectors).load(w.name)
            except (KeyError, ValueError) as e:
                bad.append("connector %s: %s" % (w.name, e.args[0] if e.args else e))
        return bad

    # ---- stages ----
    def run_stage(self, phase, wire, attempt, out):
        return getattr(self, "_" + phase)(wire, attempt, out)

    def _pick(self, wire, attempt, out):
        modes = self.cfg["pick"]["modes"]
        mode = modes[(attempt - 1) % len(modes)]       # next attempt tries the next perception mode
        self.cam.use("grasp")
        opts = pipeline.Options(out, self.live, False, mode, (wire.name,), (wire.hint,) if wire.hint else ())
        r = pipeline.Pipeline(opts, self.io, self.home, run_pipeline.WORK, run_pipeline.ROOT).run_one(out, wire.name, wire.hint or None)
        detail = "%s [%s]" % (r.outcome, mode)
        if r.outcome.startswith(run_pipeline.OK_OUTCOMES):
            return StageResult.good(detail)
        kind = "perception" if r.outcome.startswith("STOPPED_BEFORE_GRASP") else "unsafe" if r.outcome.startswith("GRASP_FAILED") else "transient"
        return StageResult.bad(kind, detail)

    def _align(self, wire, attempt, out):
        self.cam.use("ibvs")
        a = self.cfg["align"]
        cmd = ["bash", str(CODE / "IBVS/run_joint2_priority.sh"), "--socket", wire.socket, *a.get("extra_args", [])]
        if self.live:
            cmd += ["--live", "--home-first"]
        log = open(out / "ibvs.log", "w")
        self.ibvs_proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        end = time.time() + a["timeout_s"]
        while self.ibvs_proc.poll() is None and time.time() < end:
            time.sleep(1)
        if self.ibvs_proc.poll() is None:
            self.ibvs_proc.send_signal(signal.SIGINT)
            time.sleep(15)
            self.ibvs_proc.poll() is None and self.ibvs_proc.kill()
            return StageResult.bad("transient", "IBVS timeout %d s" % a["timeout_s"])
        m = re.findall(r"Run evidence: (\S+)", open(out / "ibvs.log").read())
        summary = json.loads((pathlib.Path(m[-1]) / "summary.json").read_text()) if m else {}
        (out / "ibvs_run.txt").write_text(m[-1] + "\n" if m else "")
        status = summary.get("status", "NO_SUMMARY")
        if status in IBVS_OK:
            return StageResult.good(status)
        text = "%s %s" % (status, summary.get("error", ""))
        return StageResult.bad("fatal" if status == "INTERRUPTED" else "perception" if LOST.search(text) else "transient", text.strip())

    def _insert(self, wire, attempt, out):
        ins, r = self.cfg["insert"], self.robot
        T0 = r.tool_matrix()
        push = np.eye(4)
        push[:3, 3] = np.array(ins["axis_tool"], float) * (ins["depth_m"] or 0.0)
        pts = r.line_path(T0 @ push)
        if pts is None:
            return StageResult.bad("transient", "insertion line not feasible")
        if not self.live:
            return StageResult.good("PLAN_CHECKED_ONLY: line feasible, not moved")
        r.send_line(pts, max(1.5, ins["depth_m"] / ins["speed_mps"]))
        r.gripper(0.0)
        back = np.eye(4)
        back[:3, 3] = -np.array(ins["axis_tool"], float) * ins["retreat_m"]
        out_pts = r.line_path(r.tool_matrix() @ back)
        if out_pts is None:
            return StageResult.bad("unsafe", "retreat line not feasible after release")
        r.send_line(out_pts, 3.0)
        return StageResult.good("INSERTED_UNVERIFIED: no seating sensor yet")

    # ---- recovery ----
    def recover(self, wire, holding):
        if holding and self.live:
            self.io.ask("Holding %s. Take it out of the gripper, press Enter: " % wire.name)
        self.cam.use("grasp")
        ok = self.io.call("home", 120)[0]
        return ok and self.io.call("open_gripper", 60)[0]

    def abort(self):
        self.robot.stop()
        if self.ibvs_proc and self.ibvs_proc.poll() is None:
            self.ibvs_proc.send_signal(signal.SIGINT)

    def close(self):
        self.cam.close()
