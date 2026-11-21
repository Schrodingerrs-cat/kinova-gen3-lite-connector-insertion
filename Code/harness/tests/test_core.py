import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from core import Backend, Policy, StageResult, Task, Wire  # noqa: E402

G, B = StageResult.good, StageResult.bad


class Fake(Backend):
    def __init__(self, script, recover_ok=True, problems=()):
        self.script, self.calls, self.recovers, self.ok, self.problems = script, [], [], recover_ok, list(problems)

    def preflight(self, wires):
        return self.problems

    def run_stage(self, phase, wire, attempt, out):
        self.calls.append((wire.name, phase, attempt))
        r = self.script.get((wire.name, phase), [G()])
        r = r.pop(0) if len(r) > 1 else r[0]
        if isinstance(r, Exception):
            raise r
        return r

    def recover(self, wire, holding):
        self.recovers.append((wire.name, holding))
        return self.ok


def task(tmp_path, be, wires=("a", "b"), **kw):
    return Task([Wire(n, n) for n in wires], be, tmp_path, Policy(delay_s=0, **kw), sleep=lambda s: None, log=lambda s: None)


def test_all_wires_done(tmp_path):
    be = Fake({})
    assert task(tmp_path, be).run()
    assert [c[1] for c in be.calls] == ["pick", "align", "insert"] * 2


def test_perception_retry_without_recover(tmp_path):
    be = Fake({("a", "pick"): [B("perception", "no cable"), G()]})
    assert task(tmp_path, be, ("a",)).run()
    assert be.recovers == [] and be.calls[:2] == [("a", "pick", 1), ("a", "pick", 2)]


def test_exception_is_transient_and_recovers(tmp_path):
    be = Fake({("a", "pick"): [RuntimeError("svc"), G()]})
    assert task(tmp_path, be, ("a",)).run()
    assert be.recovers == [("a", False)]


def test_insert_failure_realigns_holding(tmp_path):
    be = Fake({("a", "insert"): [B("transient", "jam"), G()]})
    assert task(tmp_path, be, ("a",)).run()
    assert [c[1] for c in be.calls] == ["pick", "align", "insert", "align", "insert"]
    assert be.recovers == [("a", True)]


def test_out_of_attempts_drops_wire_and_continues(tmp_path):
    be = Fake({("a", "align"): [B("perception", "marker lost")]})
    t = task(tmp_path, be, max_consecutive_wire_failures=5)
    assert not t.run()
    assert t.state.data["wires"]["a"]["status"] == "failed" and t.state.data["wires"]["b"]["status"] == "done"
    assert be.recovers == [("a", True)]           # wire was held, so hand-over


def test_circuit_breaker(tmp_path):
    be = Fake({(n, "pick"): [B("perception", "x")] for n in "abc"})
    t = task(tmp_path, be, "abc", max_consecutive_wire_failures=2)
    t.run()
    assert t.state.data["status"] == "STOPPED" and "c" not in t.state.data["wires"]


def test_failed_recovery_stops_run(tmp_path):
    be = Fake({("a", "pick"): [B("unsafe", "grasp failed")]}, recover_ok=False)
    t = task(tmp_path, be)
    t.run()
    assert t.state.data["status"] == "STOPPED" and len(be.calls) == 1


def test_resume_skips_finished(tmp_path):
    be = Fake({("b", "pick"): [B("fatal", "bad cfg")]})
    task(tmp_path, be).run()
    be2 = Fake({})
    t = task(tmp_path, be2)
    t.state.data["wires"]["b"]["status"] = "running"
    assert t.run(resume=True)
    assert {c[0] for c in be2.calls} == {"b"}


def test_preflight_blocks(tmp_path):
    be = Fake({}, problems=["no goal"])
    assert not task(tmp_path, be).run() and be.calls == []
