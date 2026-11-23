import importlib.util
import json
import pathlib
import sys
import types

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="source ros_env.sh to run the node tests")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "ros2_iface"))
_spec = importlib.util.spec_from_file_location("grasp_node", HERE / "ros2_iface" / "grasp_node.py")
gn = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gn)


def node(**kw):
    n = types.SimpleNamespace(last=None, home_T=np.eye(4), cad_dir=None, cfg={"grasp": {"standoff_m": 0.06, "lift_m": 0.08}}, ran=None)
    n.get_parameter = lambda name: types.SimpleNamespace(value={"max_plan_age_s": 60.0, "max_candidates": 4, "connector": "micro_jack"}[name])
    n._run = lambda grasps: setattr(n, "ran", len(grasps)) or "ran"
    for k, v in kw.items():
        setattr(n, k, v)
    return n


def plan(n_grasps, **extra):
    import time
    return dict(ok=True, grasps=list(range(n_grasps)), stamp=time.time(), **extra)


def test_execute_tries_every_grasp_a_cad_plan_asks_for_and_four_otherwise():
    n = node(last=plan(26, tries=26))
    gn.GraspNode.execute(n)
    assert n.ran == 26
    n = node(last=plan(26))
    gn.GraspNode.execute(n)
    assert n.ran == 4


def test_execute_without_a_plan_is_refused():
    with pytest.raises(RuntimeError, match="plan"):
        gn.GraspNode.execute(node())


def test_grasps_follow_the_candidates_approach_axis_with_the_jaw_made_orthogonal_to_it():
    n = node()
    z = np.array([0.0, 0.2, -1.0]) / np.linalg.norm([0.0, 0.2, -1.0])        # plug axis leaning 11 deg
    cand = dict(p=np.array([0.25, 0.0, 0.0]), jaw=np.array([0.0, 1.0, 0.0]), approach=z, width_m=0.0065, open_width_m=0.0305, offset_m=0.0, cost=0.1)
    g = gn.GraspNode._grasps_from(n, [cand])
    assert len(g) == 2 and {x["flip"] for x in g} == {1, -1}
    for x in g:
        R = x["T_BG"][:3, :3]
        assert np.allclose(R[:, 2], z) and abs(R[:, 1] @ z) < 1e-12 and np.allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert np.allclose(x["T_pre"][:3, 3], x["T_BG"][:3, 3] - z * 0.06) and np.allclose(x["T_lift"][:3, 3], x["T_BG"][:3, 3] - z * 0.08)
    old = gn.GraspNode._grasps_from(n, [dict(cand, approach=None)] if False else [{k: v for k, v in cand.items() if k != "approach"}])
    assert np.allclose(old[0]["T_BG"][:3, 2], [0, 0, -1])                    # the other modes still approach straight down


def test_cad_grasp_refuses_views_of_another_connector_and_clears_the_old_plan(tmp_path):
    (tmp_path / "target.json").write_text(json.dumps(dict(connector="usb_a", target_m=[0.26, -0.02, 0.0])))
    n = node(last="OLD PLAN", cad_dir=tmp_path)
    with pytest.raises(RuntimeError, match="views are of 'usb_a' but the connector is 'micro_jack'"):
        gn.GraspNode.cad_grasp(n)
    assert n.last is None


def test_cad_grasp_needs_the_home_pose_of_this_node_session():
    with pytest.raises(RuntimeError, match="run /grasp/cad_scan first"):
        gn.GraspNode.cad_grasp(node(home_T=None))


def test_strip_grasp_refuses_views_of_another_connector_and_clears_the_old_plan(tmp_path):
    (tmp_path / "target.json").write_text(json.dumps(dict(connector="aux", sticker_mm=[280, 0, 27], stickers_mm=[[280, 0, 27]])))
    n = node(last="OLD PLAN", strip_dir=tmp_path)
    n.get_parameter = lambda name: types.SimpleNamespace(value={"connector": "usb_a"}[name])
    with pytest.raises(RuntimeError, match="views are of 'aux' but the connector is 'usb_a'"):
        gn.GraspNode.strip_grasp(n)
    assert n.last is None


def test_strip_refine_needs_a_strip_plan():
    n = node(last=dict(ok=True, grasps=[], stamp=0))
    n.get_parameter = lambda name: types.SimpleNamespace(value={"connector": "usb_a"}[name])
    with pytest.raises(RuntimeError, match="strip_grasp first"):
        gn.GraspNode.strip_refine(n)


def test_robot_helper_is_not_renamed_by_the_launch_remap():
    import subprocess
    code = ("import sys, rclpy; sys.path.insert(0, %r)\n"
            "from robot_ros import Robot\n"
            "rclpy.init(args=['--ros-args', '-r', '__node:=grasp_node'])\n"
            "print(Robot('grasp_robot').get_fully_qualified_name())") % str(HERE / "ros2_iface")
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert r.stdout.strip().endswith("/grasp_robot"), r.stdout + r.stderr[-300:]


def test_close_views_end_with_the_arm_back_home(tmp_path, monkeypatch):
    monkeypatch.setattr(gn, "WORK", tmp_path)
    moves = []
    n = node(home_q=np.array([3.0, -3.1, 93.4, -88.5, -87.7, -82.6]), T_tool_cam=np.eye(4), info=types.SimpleNamespace(k=np.eye(3).ravel()))
    n.arm = types.SimpleNamespace(go_joints=lambda q: moves.append(np.asarray(q)))
    n._depth_frames = lambda k: ([np.ones((4, 4), np.float32)] * k, None)
    n._tool_pose = lambda stamp: np.eye(4)
    n._color_bgr = lambda: np.zeros((4, 4, 3), np.uint8)
    n.get_logger = lambda: types.SimpleNamespace(info=lambda m: None)
    views = [(0, np.array([10.0, 0, 90, -88, -87, -82])), (90, np.array([20.0, 5, 95, -88, -87, -80]))]
    out = gn.GraspNode._capture_views(n, views, dict(settle_s=0.0, frames=2), "strip")
    assert len(moves) == 3 and np.allclose(moves[0], views[0][1]) and np.allclose(moves[1], views[1][1]) and np.allclose(moves[2], n.home_q)
    assert len(list(out.glob("view_*.npz"))) == 2
