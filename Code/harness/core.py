"""Task orchestration for several wires: pick -> align -> insert, with retries, persistent state and resume. No ROS imports: everything that touches the
robot sits behind `Backend` (stages.py is the real one), so this logic is unit-tested with fakes (tests/test_core.py).

Failure kinds a stage reports, and what the harness does about each:
  perception  nothing moved that matters (cable / marker not found, plan refused)   -> retry the same phase
  transient   the arm may be anywhere (service timeout, motion error, IBVS gave up) -> recover (home, gripper), retry the phase
  unsafe      the cable / arm state is unknown (grasp failed after closing)          -> recover, retry the phase; wire is dropped when out of attempts
  fatal       retrying cannot help (bad config, operator stop)                       -> give up on the wire
An insertion that fails goes back to ALIGN (re-align from the post-grasp pose), not to PICK: the cable is still in the gripper.
Too many wires failing in a row means something systemic (calibration, lighting, e-stop): the run stops instead of grinding on.
"""
import json
import os
import pathlib
import time
from dataclasses import dataclass, field

PHASES = ("pick", "align", "insert")
NEXT = dict(pick="align", align="insert", insert=None)
KINDS = ("perception", "transient", "unsafe", "fatal")


@dataclass
class StageResult:
    ok: bool
    detail: str = ""
    kind: str = ""          # one of KINDS when not ok

    @classmethod
    def good(cls, detail=""):
        return cls(True, detail)

    @classmethod
    def bad(cls, kind, detail):
        assert kind in KINDS, kind
        return cls(False, detail, kind)


class Backend:
    """What the harness needs from the robot side."""
    def preflight(self, wires):                  # -> list of problems (strings); empty = good to go
        return []

    def run_stage(self, phase, wire, attempt, out_dir):   # -> StageResult
        raise NotImplementedError

    def recover(self, wire, holding):            # arm home + gripper open (operator hand-over when holding) -> bool
        raise NotImplementedError

    def abort(self):                             # stop motion now (Ctrl-C)
        pass

    def close(self):
        pass


@dataclass
class Wire:
    name: str
    socket: str
    hint: str = ""


@dataclass
class Policy:
    max_attempts: dict = field(default_factory=lambda: dict(pick=3, align=3, insert=2))
    delay_s: float = 2.0
    backoff: float = 2.0
    max_delay_s: float = 30.0
    max_consecutive_wire_failures: int = 2

    def delay(self, n):          # n = failed attempts so far of this phase (1 = first failure)
        return min(self.max_delay_s, self.delay_s * self.backoff ** (n - 1))


class State:
    """state.json in the run folder, rewritten atomically after every stage so a crash loses nothing."""
    def __init__(self, path):
        self.path = pathlib.Path(path)
        self.data = json.loads(self.path.read_text()) if self.path.exists() else dict(created=time.strftime("%Y-%m-%dT%H:%M:%S"), wires={}, status="NEW")

    def wire(self, name):
        return self.data["wires"].setdefault(name, dict(status="pending", phase="pick", attempts={}, history=[], resumes=0))

    def save(self):
        self.data["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.data, indent=1))
        os.replace(tmp, self.path)


class Task:
    def __init__(self, wires, backend, run_dir, policy=None, sleep=time.sleep, log=print):
        self.wires, self.be, self.dir = list(wires), backend, pathlib.Path(run_dir)
        self.policy, self.sleep, self._log = policy or Policy(), sleep, log
        self.dir.mkdir(parents=True, exist_ok=True)
        self.state = State(self.dir / "state.json")

    def log(self, msg):
        line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
        self._log(line)
        with open(self.dir / "task.log", "a") as f:
            f.write(line + "\n")

    # ---- one wire ----
    def _recover(self, w, holding):
        try:
            ok = bool(self.be.recover(w, holding))
        except Exception as e:      # a recovery that raises is a failed recovery
            self.log("recover raised %s: %s" % (type(e).__name__, e))
            ok = False
        self.log("recover (%s) -> %s" % ("holding wire" if holding else "empty gripper", "OK" if ok else "FAILED"))
        return ok

    def run_wire(self, wire):
        """Returns "done", "failed" or "stop" (arm state unknown: do not carry on)."""
        st = self.state.wire(wire.name)
        st.update(status="running", phase="pick", attempts={})
        holding = False
        phase = "pick"
        while phase:
            n = st["attempts"][phase] = st["attempts"].get(phase, 0) + 1
            if n > self.policy.max_attempts[phase]:
                self.log("%s: %s out of attempts (%d)" % (wire.name, phase, n - 1))
                return self._give_up(wire, st, holding, "%s out of attempts" % phase)
            out = self.dir / wire.name / ("%s_%d" % (phase, n))
            out.mkdir(parents=True, exist_ok=True)
            st["phase"] = phase
            self.state.save()
            t0 = time.time()
            try:
                res = self.be.run_stage(phase, wire, n, out)
            except KeyboardInterrupt:
                raise
            except Exception as e:
                res = StageResult.bad("transient", "%s: %s" % (type(e).__name__, e))
            st["history"].append(dict(phase=phase, attempt=n, ok=res.ok, kind=res.kind, detail=res.detail, seconds=round(time.time() - t0, 1)))
            self.state.save()
            self.log("%s %s #%d -> %s %s" % (wire.name, phase, n, "OK" if res.ok else "FAILED(%s)" % res.kind, res.detail))
            if res.ok:
                holding = holding or phase == "pick" and self._holds(res)
                phase = NEXT[phase]
                continue
            if res.kind == "fatal":
                return self._give_up(wire, st, holding, res.detail)
            if res.kind in ("transient", "unsafe") and not self._recover(wire, holding):
                st["status"] = "stopped"
                return "stop"
            if phase == "insert":                       # cable still in the gripper: re-align rather than re-pick
                st["attempts"]["align"] = 0
                phase = "align"
            self.sleep(self.policy.delay(n))
        st["status"], st["phase"] = "done", "done"
        self.log("%s: DONE" % wire.name)
        return "done"

    @staticmethod
    def _holds(res):
        return "PERCEPTION_ONLY" not in res.detail      # dry run: nothing was picked, so nothing to hand over

    def _give_up(self, wire, st, holding, why):
        st.update(status="failed", reason=why)
        if not self._recover(wire, holding):
            st["status"] = "stopped"
            return "stop"
        return "failed"

    # ---- all wires ----
    def run(self, resume=False):
        problems = self.be.preflight(self.wires)
        if problems:
            for p in problems:
                self.log("PREFLIGHT: " + p)
            self.state.data["status"] = "PREFLIGHT_FAILED"
            self.state.save()
            return False
        self.state.data["status"] = "RUNNING"
        streak = 0
        try:
            for wire in self.wires:
                prev = self.state.data["wires"].get(wire.name)
                if prev and prev["status"] in ("done", "failed"):
                    self.log("%s: already %s, skipping" % (wire.name, prev["status"]))
                    continue
                if prev and resume:
                    prev["resumes"] += 1         # in-flight wire: the cable may be anywhere, so it restarts at pick (the next pick homes + opens the gripper)
                if self.state.data["status"] == "STOPPED":
                    break
                r = self.run_wire(wire)
                self.state.save()
                if r == "stop":
                    self.state.data["status"] = "STOPPED"
                    break
                streak = 0 if r == "done" else streak + 1
                if streak >= self.policy.max_consecutive_wire_failures:
                    self.log("%d wires failed in a row: stopping (systemic problem?)" % streak)
                    self.state.data["status"] = "STOPPED"
                    break
        except KeyboardInterrupt:
            self.be.abort()
            self.state.data["status"] = "INTERRUPTED"
            self.log("interrupted: arm stopped, state saved; --resume to continue")
        finally:
            ws = self.state.data["wires"]
            if self.state.data["status"] == "RUNNING":
                self.state.data["status"] = "COMPLETE" if all(ws.get(w.name, {}).get("status") == "done" for w in self.wires) else "INCOMPLETE"
            self.state.save()
            self.be.close()
        self.log("run %s: %s" % (self.state.data["status"], {k: v["status"] for k, v in self.state.data["wires"].items()}))
        return self.state.data["status"] == "COMPLETE"
