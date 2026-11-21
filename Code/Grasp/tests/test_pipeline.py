import json
import os
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "ros2_iface"))
import pipeline as pl  # noqa: E402

HOME = np.array([3.0, -3.1, 93.4, -88.5, -87.7, -82.6])


class FakeIo(pl.Io):
    def __init__(self, work, q=None, results=None, side_effects=None):
        self.work, self.q = work, HOME.copy() if q is None else np.asarray(q, float)
        self.results, self.effects, self.log, self.params, self.prompts, self.snaps = results or {}, side_effects or {}, [], [], [], []
        self.recorders = []

    def call(self, service, timeout):
        self.log.append(service)
        if service == "home" and "home" not in self.results:
            self.q = HOME.copy()
        if service in self.effects:
            self.effects[service](self)
        ok = self.results.get(service, [True])
        r = ok.pop(0) if isinstance(ok, list) and len(ok) > 1 else (ok[0] if isinstance(ok, list) else ok)
        return (r, "%s message" % service) if isinstance(r, bool) else r

    def set_node_params(self, **values):
        self.params.append(values)

    def joints_deg(self):
        return self.q

    def snap(self, path):
        self.snaps.append(path.name)
        path.write_bytes(b"png")

    def start_recorder(self, out_dir):
        rec = type("Rec", (), {"stopped": False, "stop": lambda s: setattr(s, "stopped", True)})()
        self.recorders.append(rec)
        return rec

    def ask(self, prompt):
        self.prompts.append(prompt)


def make(tmp_path, mode="cad", connectors=(), hints=(), grasp=False, between="pause", skip_scan=False, **io_kw):
    work, code = tmp_path / "work", tmp_path / "code"
    for d in (work, code / "config", code / "logs"):
        d.mkdir(parents=True)
    for f in ("config/grasp.yaml", "config/connectors.yaml", "config/strip.yaml", "logs/grasp_node.log"):
        (code / f).write_text("x")
    old = work / "runs" / "20200101_000000"
    (old / "plots").mkdir(parents=True)
    (old / "plan.json").write_text("{}")
    os.utime(old / "plan.json", (1, 1))                                   # a plan of an earlier run: must never be collected
    io = FakeIo(work, **io_kw)

    def new_plan(i):                                                      # what the node writes while planning
        d = work / "runs" / ("2026%04d_%06d" % (len(i.log), 1))
        (d / "plots").mkdir(parents=True, exist_ok=True)
        (d / "plan.json").write_text("{}")
        (d / "plots" / "a.png").write_bytes(b"p")
    io.effects.setdefault("cad_grasp", new_plan)
    io.effects.setdefault("head_grasp", new_plan)
    opts = pl.Options(tmp_path / "run", grasp, skip_scan, mode, tuple(connectors), tuple(hints), between)
    pipe = pl.Pipeline(opts, io, HOME, work, code, sleep=lambda s: None)
    return pipe, io


def test_arguments_are_validated():
    for bad in (["r", "--connector", "usb_a"], ["r", "--mode", "cad", "--hint", "0.2,0.1"], ["r", "--mode", "cad", "--connector", "a", "a"],
                ["r", "--mode", "cad", "--connector", "a", "--hint", "0.2;0.1"], ["r", "--mode", "cad", "--connector", "a", "--hint", "0.1,0.2", "0.3,0.4"]):
        with pytest.raises(SystemExit):
            pl.parse_args(bad)
    o = pl.parse_args(["r", "--mode", "cad", "--connector", "usb_a", "micro_jack", "--hint", "0.26,-0.02", "--grasp", "--between", "release"])
    assert o.connectors == ("usb_a", "micro_jack") and o.hints == ("0.26,-0.02",) and o.grasp and o.between == "release"
    assert pl.parse_args(["r"]).mode == "head" and pl.parse_args(["r"]).connectors == ()


def test_single_cable_perception_only_runs_the_stages_in_order_and_collects_only_new_files(tmp_path):
    pipe, io = make(tmp_path, connectors=("micro_jack",), hints=("0.26,-0.02",))
    (res,) = pipe.run()
    assert io.log == ["open_gripper", "cad_scan", "cad_grasp"] and io.params == [dict(connector="micro_jack", hint="0.26,-0.02")]
    assert res.outcome.startswith("PERCEPTION_ONLY") and not res.picked and not res.stop
    run = tmp_path / "run"
    assert (run / "metrics.json").exists() and (run / "config_connectors.yaml").exists() and (run / "grasp_node.log").exists()
    assert len(list((run / "plan").glob("*"))) == 1 and "20200101_000000" not in [p.name for p in (run / "plan").iterdir()]    # the old plan is not collected
    assert io.recorders[0].stopped and "99_end.png" in io.snaps
    assert not (run / "summary.json").exists()                                                                                  # one cable: no sub-folders, no summary


def test_arm_away_from_home_goes_home_first_and_a_failed_home_stops_the_run(tmp_path):
    pipe, io = make(tmp_path, connectors=("usb_a",), q=HOME + 30)
    pipe.run()
    assert io.log[:2] == ["home", "open_gripper"]
    pipe, io = make(tmp_path / "b", connectors=("usb_a",), q=HOME + 30, results={"home": [True]}, side_effects={"home": lambda i: None})
    (res,) = pipe.run()
    assert io.log == ["home"] and res.outcome.startswith("ERROR: RuntimeError: arm is still") and res.stop       # never opens the gripper or scans from an unknown pose


def test_two_cables_with_a_pause_between_them(tmp_path):
    pipe, io = make(tmp_path, connectors=("usb_a", "micro_jack"), hints=("0.26,-0.02",), grasp=True,
                    results={"execute": [True]})
    res = pipe.run()
    assert [r.name for r in res] == ["usb_a", "micro_jack"] and all(r.picked for r in res)
    assert io.log == ["open_gripper", "cad_scan", "cad_grasp", "allow_grasp", "execute", "open_gripper", "open_gripper", "cad_scan", "cad_grasp", "allow_grasp", "execute"]
    assert len(io.prompts) == 1 and "usb_a" in io.prompts[0] and "micro_jack" in io.prompts[0]
    assert io.params == [dict(connector="usb_a", hint="0.26,-0.02"), dict(connector="micro_jack", hint="")]
    run = tmp_path / "run"
    assert (run / "01_usb_a" / "metrics.json").exists() and (run / "02_micro_jack" / "metrics.json").exists()
    summary = json.loads((run / "summary.json").read_text())
    assert [c["connector"] for c in summary["connectors"]] == ["usb_a", "micro_jack"] and summary["not_run"] == []


def test_between_release_does_not_ask_and_between_stop_ends_the_run(tmp_path):
    pipe, io = make(tmp_path / "a", connectors=("usb_a", "micro_jack"), grasp=True, between="release")
    assert len(pipe.run()) == 2 and io.prompts == []
    pipe, io = make(tmp_path / "b", connectors=("usb_a", "micro_jack"), grasp=True, between="stop")
    assert len(pipe.run()) == 1 and io.prompts == []
    assert json.loads((tmp_path / "b" / "run" / "summary.json").read_text())["not_run"] == ["micro_jack"]


def test_a_cable_that_was_not_picked_up_does_not_block_the_next_one(tmp_path):
    pipe, io = make(tmp_path, connectors=("usb_a", "micro_jack"), grasp=True, results={"cad_grasp": [False, True]})
    res = pipe.run()
    assert len(res) == 2 and not res[0].picked and res[0].outcome.startswith("STOPPED_BEFORE_GRASP: cad_grasp message") and res[1].picked
    assert io.prompts == [] and io.log.count("execute") == 1


def test_a_failed_execute_stops_the_run(tmp_path):
    pipe, io = make(tmp_path, connectors=("usb_a", "micro_jack"), grasp=True, results={"execute": [False]})
    res = pipe.run()
    assert len(res) == 1 and res[0].stop and res[0].outcome.startswith("GRASP_FAILED") and io.prompts == []


def test_other_modes_keep_their_stage_order(tmp_path):
    pipe, io = make(tmp_path, mode="head")
    pipe.run()
    assert io.log == ["open_gripper", "scan", "color_target", "view", "head_grasp"] and io.params == []
    pipe, io = make(tmp_path / "s", mode="head", skip_scan=True, grasp=True)
    pipe.run()
    assert io.log == ["open_gripper", "color_target", "view", "head_grasp", "allow_grasp", "execute"]


def test_an_exception_in_a_stage_is_recorded_and_the_recorder_is_stopped(tmp_path):
    pipe, io = make(tmp_path, connectors=("usb_a",), results={"cad_scan": [False]})
    (res,) = pipe.run()
    assert res.outcome == "ERROR: RuntimeError: cad_scan failed" and res.stop and io.recorders[0].stopped
    assert json.loads((tmp_path / "run" / "metrics.json").read_text())["stages"]["cad_scan"]["ok"] is False


def test_strip_mode_runs_find_fit_hover_then_grasp_and_collects_its_views(tmp_path):
    pipe, io = make(tmp_path, mode="strip", connectors=("usb_a",), hints=("0.28,0.0",), grasp=True,
                    side_effects={"strip_scan": lambda i: (lambda d: (d.mkdir(parents=True, exist_ok=True), (d / "target.json").write_text("{}")))(i.work / "strip" / "20261120_000000"),
                                  "strip_grasp": lambda i: (lambda d: (d.mkdir(parents=True, exist_ok=True), (d / "plan.json").write_text("{}")))(i.work / "runs" / "20261120_000001")})
    (res,) = pipe.run()
    assert io.log == ["open_gripper", "strip_scan", "strip_grasp", "strip_refine", "allow_grasp", "execute"] and res.picked
    assert io.params == [dict(connector="usb_a", hint="0.28,0.0")]
    run = tmp_path / "run"
    assert (run / "strip_views" / "20261120_000000" / "target.json").exists() and (run / "config_strip.yaml").exists() and not (run / "config_connectors.yaml").exists()


def test_a_refused_hover_refit_means_no_grasp(tmp_path):
    pipe, io = make(tmp_path, mode="strip", connectors=("usb_a",), grasp=True, results={"strip_refine": [False]})
    (res,) = pipe.run()
    assert "execute" not in io.log and res.outcome.startswith("ERROR: RuntimeError: hover refit refused") and res.stop


def test_connector_needs_cad_or_strip_mode():
    with pytest.raises(SystemExit):
        pl.parse_args(["r", "--mode", "head", "--connector", "usb_a"])
    assert pl.parse_args(["r", "--mode", "strip", "--connector", "usb_a", "aux"]).connectors == ("usb_a", "aux")
