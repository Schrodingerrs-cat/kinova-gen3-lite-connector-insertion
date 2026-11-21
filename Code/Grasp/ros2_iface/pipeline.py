"""Run orchestration of the grasp pipeline for one or several cables. No ROS imports: the ROS side (services, camera, recorder) is injected through `Io`,
so the sequencing is unit-tested with fakes (tests/test_pipeline.py). run_pipeline.py wires the real ROS objects.

One cable:   home check -> gripper open -> [scan -> colour target -> close view | cad_scan] -> grasp plan -> [execute].
Several cables (cad mode, --connector A B ...): each cable gets its own sub-folder; between two cables that was picked up the operator is asked (pause),
the gripper is opened at once (release) or the run stops (stop). A cable that was NOT picked up (perception failure) does not block the next one;
a failed execute (the arm may hold the cable or be anywhere) stops the run.
"""
import argparse
import json
import pathlib
import shutil
import time
from dataclasses import dataclass, field

import numpy as np

MODES = ("head", "cloud", "model", "cad", "strip")
BETWEEN = ("pause", "release", "stop")
MODE_SERVICE = dict(head="head_grasp", cloud="cloud_grasp", model="model_grasp")
TWO_STAGE = dict(cad=("cad_scan", "cad_grasp"), strip=("strip_scan", "strip_grasp"))   # modes without the 7-pose scan: find + close views, then fit + plan
HOME_TOL_START_DEG, HOME_TOL_DONE_DEG = 1.0, 2.0


@dataclass(frozen=True)
class Options:
    run_dir: pathlib.Path
    grasp: bool = False
    skip_scan: bool = False
    mode: str = "head"
    connectors: tuple = ()
    hints: tuple = ()
    between: str = "pause"


class Io:
    """What the pipeline needs from the outside world (the real one is in run_pipeline.py)."""
    def call(self, service, timeout):           # -> (ok, message); raises on timeout / missing service
        raise NotImplementedError

    def set_node_params(self, **values):        # strings only (connector, hint)
        raise NotImplementedError

    def joints_deg(self):
        raise NotImplementedError

    def snap(self, path):                       # save the current colour image
        raise NotImplementedError

    def start_recorder(self, out_dir):          # -> object with .stop()
        raise NotImplementedError

    def ask(self, prompt):                      # operator prompt, returns when the operator answers
        raise NotImplementedError


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Grasp pipeline end to end with data collection into RUN_DIR.")
    ap.add_argument("run_dir", type=pathlib.Path)
    ap.add_argument("--grasp", action="store_true", help="execute the grasp (default: perception and planning only)")
    ap.add_argument("--skip-scan", action="store_true", help="reuse the last 7-pose scan (head, cloud, model modes)")
    ap.add_argument("--mode", choices=MODES, default="head", help="head (default), cloud, model, cad (STEP model fitted to 3 close views), strip (rectangle on the plug, orientation from the white strip)")
    ap.add_argument("--connector", nargs="+", metavar="NAME", help="cad / strip mode: connector(s) of config/connectors.yaml / config/strip.yaml to pick, in this order")
    ap.add_argument("--hint", nargs="+", metavar="X,Y", help="cad / strip mode: approximate base-frame x,y (m) of each connector's sticker, same order as --connector")
    ap.add_argument("--between", choices=BETWEEN, default="pause", help="several cables: after one was picked up, ask the operator (pause), open the gripper at once (release) or stop")
    a = ap.parse_args(argv)
    if a.connector and a.mode not in TWO_STAGE:
        ap.error("--connector needs --mode cad or strip")
    if a.hint and not a.connector:
        ap.error("--hint needs --connector")
    if a.connector and len(set(a.connector)) != len(a.connector):
        ap.error("--connector lists a connector twice")
    hints = []
    for h in a.hint or []:
        try:
            x, y = (float(v) for v in h.split(","))
        except ValueError:
            ap.error("--hint %r is not X,Y in metres" % h)
        hints.append(h)
    if len(hints) > len(a.connector or []):
        ap.error("more --hint values than --connector names")
    return Options(a.run_dir, a.grasp, a.skip_scan, a.mode, tuple(a.connector or ()), tuple(hints), a.between)


PATTERNS = ("runs/2*/plan.json", "scans/*/marker_cloud.npy", "cad/2*/target.json", "strip/2*/target.json")


def snapshot(work):
    """The node's output files that exist now: whatever appears later belongs to the run that is starting."""
    return frozenset(p for pat in PATTERNS for p in work.glob(pat))


def newest(work, pattern, known=frozenset()):
    """Newest file matching `pattern` that is not in `known` (the earlier runs' files)."""
    found = [p for p in work.glob(pattern) if p not in known]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


def collect(run_dir, work, code_root, opts, known):
    """Copy this run's scan / cad views / plan / plots / config / node log into the run folder. Files that existed before the run are never collected."""
    plan = newest(work, "runs/2*/plan.json", known)
    scan = None if opts.mode in TWO_STAGE else newest(work, "scans/*/marker_cloud.npy", known)
    views = newest(work, opts.mode + "/2*/target.json", known) if opts.mode in TWO_STAGE else None
    if views is not None:
        shutil.copytree(views.parent, run_dir / (opts.mode + "_views") / views.parent.name, dirs_exist_ok=True)
    if scan is not None:
        shutil.copytree(scan.parent, run_dir / "scan" / scan.parent.name, dirs_exist_ok=True)
        for p in (scan.parent / "plots").glob("*.png"):
            shutil.copy(p, run_dir / "plots" / ("scan_" + p.name))
    elif opts.skip_scan and opts.mode not in TWO_STAGE:
        last = newest(work, "scans/*/marker_cloud.npy")
        if last is not None:
            (run_dir / "scan_used.txt").write_text(str(last.parent) + "\n")
    if plan is not None:
        shutil.copytree(plan.parent, run_dir / "plan" / plan.parent.name, dirs_exist_ok=True)
        for p in (plan.parent / "plots").glob("*.png"):
            shutil.copy(p, run_dir / "plots" / ("plan_" + p.name))
    mode_cfg = [code_root / "config" / ("strip.yaml" if opts.mode == "strip" else "connectors.yaml")] if opts.mode in TWO_STAGE else []
    for src in [code_root / "config" / "grasp.yaml", *mode_cfg, code_root / "logs" / "grasp_node.log"]:
        if src.exists():
            shutil.copy(src, run_dir / ("config_" + src.name if src.suffix == ".yaml" else src.name))


@dataclass
class Result:
    name: str
    run_dir: pathlib.Path
    outcome: str = "INCOMPLETE"
    picked: bool = False        # the cable is (reported) in the gripper
    stop: bool = False          # do not go on to the next cable


@dataclass
class Pipeline:
    opts: Options
    io: Io
    home_deg: np.ndarray
    work: pathlib.Path
    code_root: pathlib.Path
    clock: callable = time.time
    sleep: callable = time.sleep
    results: list = field(default_factory=list)

    # ---- one cable ----
    def _log(self, run_dir, t0, msg):
        line = "[%6.1f s] %s" % (self.clock() - t0, msg)
        print(line, flush=True)
        with open(run_dir / "controller.log", "a") as f:
            f.write(line + "\n")

    def _call(self, run_dir, t0, metrics, name, timeout):
        t = self.clock()
        ok, msg = self.io.call(name, timeout)
        metrics["stages"][name] = dict(ok=ok, seconds=round(self.clock() - t, 1), message=msg)
        self._log(run_dir, t0, "%s -> %s: %s" % (name, "OK" if ok else "FAILED", msg))
        return ok

    def _preflight(self, call):
        """Every cable starts at home with the gripper fully open."""
        q = np.asarray(self.io.joints_deg(), float)
        if np.abs(q - self.home_deg).max() > HOME_TOL_START_DEG:
            call("home", 120)
            self.sleep(1.0)
            away = float(np.abs(np.asarray(self.io.joints_deg(), float) - self.home_deg).max())
            if away > HOME_TOL_DONE_DEG:
                raise RuntimeError("arm is still %.1f deg from home after /grasp/home (dry run? driver?): not starting from an unknown pose" % away)
        if not call("open_gripper", 60):
            raise RuntimeError("could not open the gripper")

    def _perceive(self, call, snap):
        if self.opts.mode in TWO_STAGE:
            scan, fit = TWO_STAGE[self.opts.mode]
            if not call(scan, 300):
                raise RuntimeError(scan + " failed")
            snap("03_close_view")
            return call(fit, 300)
        if not self.opts.skip_scan and not call("scan", 1500):
            raise RuntimeError("scan failed")
        if not call("color_target", 300):
            raise RuntimeError("coarse colour-target stage failed")
        if not call("view", 300):
            raise RuntimeError("camera could not go over the target")
        self.sleep(1.0)
        snap("03_close_view")
        return call(MODE_SERVICE[self.opts.mode], 300)

    def run_one(self, run_dir, name, hint):
        for sub in ("screenshots", "videos", "plots"):
            (run_dir / sub).mkdir(parents=True, exist_ok=True)
        t0, known = self.clock(), snapshot(self.work)
        metrics = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"), grasp_requested=self.opts.grasp, mode=self.opts.mode, connector=name, hint=hint, stages={}, outcome="INCOMPLETE")
        res = Result(name or self.opts.mode, run_dir)
        call = lambda svc, timeout: self._call(run_dir, t0, metrics, svc, timeout)
        snap = lambda n: self.io.snap(run_dir / "screenshots" / (n + ".png"))
        recorder = self.io.start_recorder(run_dir)
        try:
            snap("00_start")
            if self.opts.mode in TWO_STAGE:
                self.io.set_node_params(connector=name or "", hint=hint or "")
            self._preflight(call)
            planned = self._perceive(call, snap)
            snap("04_after_grasp_plan")
            if not planned:
                metrics["outcome"] = "STOPPED_BEFORE_GRASP: " + metrics["stages"][TWO_STAGE[self.opts.mode][1] if self.opts.mode in TWO_STAGE else MODE_SERVICE[self.opts.mode]]["message"]
            elif not self.opts.grasp:
                metrics["outcome"] = "PERCEPTION_ONLY: grasp computed, not requested"
            else:
                if self.opts.mode == "strip" and not call("strip_refine", 300):
                    raise RuntimeError("hover refit refused: no grasp")
                call("allow_grasp", 20)
                ok = call("execute", 600)
                snap("05_after_grasp")
                metrics["outcome"] = ("GRASP_REPORTED_HELD" if ok else "GRASP_FAILED") + ": " + metrics["stages"]["execute"]["message"]
                res.picked, res.stop = ok, not ok          # a failed execute leaves the arm / cable in an unknown state
        except Exception as e:                              # keep the evidence of whatever happened
            metrics["outcome"] = "ERROR: %s: %s" % (type(e).__name__, e)
            self._log(run_dir, t0, metrics["outcome"])
            res.stop = True
        finally:
            snap("99_end")
            metrics["seconds_total"] = round(self.clock() - t0, 1)
            recorder.stop()
            collect(run_dir, self.work, self.code_root, self.opts, known)
            (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=1))
            self._log(run_dir, t0, "outcome: " + metrics["outcome"])
        res.outcome = metrics["outcome"]
        return res

    # ---- one or several cables ----
    def _between(self, previous, upcoming):
        """The previous cable is in the gripper and another one is next: hand it over according to --between. Returns False to stop the run."""
        print("holding %s; next: %s" % (previous.name, upcoming), flush=True)
        if self.opts.between == "stop":
            return False
        if self.opts.between == "pause":
            self.io.ask("Holding %s. Take it out of the gripper (or clear the way), then press Enter to open the gripper and continue with %s: " % (previous.name, upcoming))
        ok, msg = self.io.call("open_gripper", 60)
        print("open_gripper -> %s: %s" % ("OK" if ok else "FAILED", msg), flush=True)
        return ok

    def run(self):
        names = self.opts.connectors or (None,)
        hints = list(self.opts.hints) + [None] * (len(names) - len(self.opts.hints))
        multi = len(names) > 1
        self.opts.run_dir.mkdir(parents=True, exist_ok=True)
        for i, (name, hint) in enumerate(zip(names, hints)):
            res = self.run_one(self.opts.run_dir / ("%02d_%s" % (i + 1, name)) if multi else self.opts.run_dir, name, hint)
            self.results.append(res)
            if res.stop:
                break
            if res.picked and i + 1 < len(names) and not self._between(res, names[i + 1]):
                break
        if multi:
            summary = [dict(connector=r.name, folder=r.run_dir.name, outcome=r.outcome, picked=r.picked) for r in self.results]
            skipped = [n for n in names[len(self.results):]]
            (self.opts.run_dir / "summary.json").write_text(json.dumps(dict(connectors=summary, not_run=skipped, between=self.opts.between), indent=1))
        return self.results
