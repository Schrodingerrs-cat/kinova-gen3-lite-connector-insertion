import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import synth_strip as S  # noqa: E402
from housing_grasp import strip_config, strip_fit as F  # noqa: E402

CFG, USB = strip_config.load("usb_a")
G = dict(h=[80, 105], s_min=100, v_min=60)
REGION = np.array([0.18, 0.58, -0.32, 0.32]) * 1000
YAW, CENTRE = 20.0, (0.30, 0.0)


def scene(plugs):
    return [S.strip(CENTRE, YAW)] + plugs


def views(boxes, target, spec_views=None):
    out = []
    for e in (spec_views or CFG["views"]["list"]):
        out.append(S.render(S.cam(target, YAW + e["across_deg"], 0.20, e["tilt_deg"]), boxes))
    return out


def plan(boxes, target, spec=USB, others=()):
    home = S.home_view(boxes)
    poses = [F.strip_pose(home, CFG["strip"], REGION)] + [F.strip_pose(v, CFG["strip"], REGION, np.array(target[:2]) * 1000) for v in views(boxes, target)]
    strip = F.strip_summary(poses, CFG["strip"])
    obs = [F.view_points(v, spec, G, np.array(target) * 1000, strip["top_z"]) for v in views(boxes, target)]
    return strip, F.evaluate(obs, strip, spec, CFG, 10.0, 5.0, list(others))


def test_strip_pose_from_home_is_accurate():
    p = F.strip_pose(S.home_view(scene([])), CFG["strip"], REGION)
    assert p["complete"] and abs(F.wrap180(p["yaw"] - YAW)) < 1.5
    assert np.linalg.norm(p["centre"] - np.array(CENTRE) * 1000) < 3 and abs(p["top_z"] - S.STRIP_TOP * 1000) < 3
    assert np.allclose(p["size"], [150, 51], atol=8)


def test_partial_strip_does_not_count_for_the_yaw():
    close = S.render(S.cam([0.30, 0.0, -0.018], YAW + 90, 0.04, 0), scene([]))
    p = F.strip_pose(close, CFG["strip"], REGION)
    assert p is None or not p["complete"]


def at(u, v):
    return tuple(np.array(CENTRE) + F.from_uv(np.array([u, v]), YAW) * 1e-3)


def test_stickers_found_on_the_strip_only_and_cable_choice():
    off = S.box(at(0, 60), 0, (0.015, 0.0075), S.TABLE_Z, S.STRIP_TOP + 0.045)
    boxes = scene([S.plug(at(-55, 0), YAW), S.plug(at(-30, 0), YAW), S.plug(at(45, 0), YAW), off])
    home = S.home_view(boxes)
    strip = F.strip_summary([F.strip_pose(home, CFG["strip"], REGION)], CFG["strip"])
    found = F.find_stickers(home, strip, G, CFG["strip"], 30)
    assert len(found) == 3
    free = found[F.choose_cable(found)]
    assert np.linalg.norm(free[:2] - np.array(at(45, 0)) * 1000) < 5
    near = found[F.choose_cable(found, np.array(at(-55, 0)) * 1000)]
    assert np.linalg.norm(near[:2] - np.array(at(-55, 0)) * 1000) < 5


def test_usb_rectangle_centre_and_gates():
    c = (0.285, 0.01)
    strip, r = plan(scene([S.plug(c, YAW)]), [c[0], c[1], S.STRIP_TOP + 0.045])
    assert r["passed"], r["gates"]
    assert np.linalg.norm(r["centre_mm"] - np.array(c) * 1000) < 2.0
    assert np.allclose(r["extent_mm"], [15, 7.5], atol=3.0)
    jaw = r["candidate"]["jaw"][:2]
    assert abs(jaw @ F.direction(YAW)) < 1e-3 and np.allclose(r["candidate"]["approach"], [0, 0, -1])
    assert abs(r["candidate"]["p"][2] * 1000 - (strip["top_z"] + 12)) < 1e-6


def test_cable_above_the_plug_does_not_move_the_rectangle():
    c = (0.285, 0.01)
    cable = S.box((c[0] + 0.01, c[1] + 0.012), 0, (0.004, 0.004), S.STRIP_TOP + 0.045, S.STRIP_TOP + 0.12, top_bgr=(20, 20, 20))
    _, r = plan(scene([S.plug(c, YAW), cable]), [c[0], c[1], S.STRIP_TOP + 0.045])
    assert r["passed"] and np.linalg.norm(r["centre_mm"] - np.array(c) * 1000) < 2.5


def test_plug_turned_away_from_the_strip_is_refused():
    c = (0.285, 0.01)
    _, r = plan(scene([S.plug(c, YAW + 40)]), [c[0], c[1], S.STRIP_TOP + 0.045])
    bad = {g["name"] for g in r["gates"] if not g["ok"]}
    assert not r["passed"] and bad & {"plug orientation agrees with the strip", "footprint matches the connector"}


def test_wrong_connector_profile_fails_the_footprint_gate():
    c = (0.285, 0.01)
    _, r = plan(scene([S.plug(c, YAW)]), [c[0], c[1], S.STRIP_TOP + 0.045], strip_config.load("aux")[1])
    assert not r["passed"] and "footprint matches the connector" in {g["name"] for g in r["gates"] if not g["ok"]}


def test_round_jack_needs_position_only():
    spec = strip_config.load("aux")[1]
    c = (0.30, -0.015)
    jack = S.box(c, 0, (0.0065, 0.0065), S.STRIP_TOP, S.STRIP_TOP + 0.024)
    _, r = plan(scene([jack]), [c[0], c[1], S.STRIP_TOP + 0.024], spec)
    assert r["passed"], r["gates"]
    assert np.linalg.norm(r["centre_mm"] - np.array(c) * 1000) < 2.0
    assert abs(abs(r["candidate"]["jaw"][:2] @ F.direction(YAW)) - 1) < 1e-3      # jaws along the strip


def test_neighbour_in_the_finger_path_is_refused_and_one_far_away_is_fine():
    c = (0.285, 0.01)
    target = [c[0], c[1], S.STRIP_TOP + 0.045]
    near = np.array(c) * 1000 + F.from_uv(np.array([0.0, 14.0]), YAW)
    far = np.array(c) * 1000 + F.from_uv(np.array([50.0, 0.0]), YAW)
    boxes = scene([S.plug(c, YAW)])
    assert not plan(boxes, target, others=[near])[1]["passed"]
    assert plan(boxes, target, others=[far])[1]["passed"]


def test_refine_accepts_a_small_shift_and_refuses_a_large_one():
    c = (0.285, 0.01)
    strip, r = plan(scene([S.plug(c, YAW)]), [c[0], c[1], S.STRIP_TOP + 0.045])
    hover = S.render(S.cam([c[0], c[1], S.STRIP_TOP + 0.045], 0, 0.22, 0), scene([S.plug(c, YAW)]))
    base = dict(centre_mm=r["centre_mm"], top_mm=r["top_mm"], strip_top_mm=strip["top_z"], yaw_deg=r["yaw_deg"])
    ok = F.refine(hover, USB, G, dict(base, centre_mm=r["centre_mm"] + [3.0, 0.0]), CFG)
    assert ok["ok"] and 1.0 < ok["shift_mm"] < 5.0 and np.linalg.norm(ok["centre_mm"] - np.array(c) * 1000) < 2.0
    assert not F.refine(hover, USB, G, dict(base, centre_mm=r["centre_mm"] + [15.0, 0.0]), CFG)["ok"]


def test_strip_config_validation(tmp_path):
    import yaml
    assert strip_config.names(ready_only=True) == ["usb_a", "aux"]
    with pytest.raises(ValueError, match="not set up yet"):
        strip_config.load("power")
    with pytest.raises(ValueError, match="unknown strip connector 'x'"):
        strip_config.load("x")
    cfg = yaml.safe_load(open(strip_config.CONFIG))
    del cfg["connectors"]["usb_a"]["clear_mm"]
    (tmp_path / "s.yaml").write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="missing key 'clear_mm'"):
        strip_config.load("usb_a", tmp_path / "s.yaml")
