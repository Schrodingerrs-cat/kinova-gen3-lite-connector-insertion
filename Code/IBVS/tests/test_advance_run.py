"""advance() with a fake robot: dry run, full stroke, trimming, stalled arm, plug at the wall."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
from joint2_priority_servo import advance, run_advance  # noqa: E402

K = np.array([[435., 0, 426.5], [0, 434.4, 238.2], [0, 0, 1]])


def fake(tmp_path, wall=.29, held_depth=.10, follows=True):   # fingers closer than 0.18 m are the held object (live rule)
    """Camera looks straight at a wall `wall` m away; fingers (held_depth) sit at the bottom. Base frame = camera frame, fk(q) = translation q[:3] m."""
    depth = np.full((480, 848), wall)
    depth[400:470, 400:500] = held_depth
    e = SimpleNamespace()
    e.last_observation = dict(depth_m=depth, K=K, distortion=np.zeros(5), color=np.zeros((480, 848, 3), np.uint8),
                              corners_px=np.array([[404, 183], [450, 183], [450, 229], [404, 229]], float))
    e.reference, e.c = dict(camera_frame_id='f'), dict(depth_min_m=.07, depth_max_m=1., min_move_s=.1, arrival_settle_s=.5)
    e.T_EC, e.T_tcp = np.eye(4), np.eye(4)
    e.T_tcp[2, 3] = .035
    e.output, e.moves, e.events, e.paths = tmp_path, [], [], []
    e.r = SimpleNamespace(q=np.zeros(6), grip_pos_msr=.8)
    e.fk = lambda q: np.block([[np.eye(3), np.asarray(q[:3])[:, None]], [np.zeros((1, 3)), 1]])
    e.r.ik = lambda T, seed_deg: np.r_[np.degrees(T[:3, 3]), np.zeros(3)]
    e.check_joint_path = lambda path, local=False: path
    e.event = lambda name, **kw: e.events.append(name)

    def move(path, duration):
        e.moves.append(path[-1].copy())
        e.paths.append(np.array(path))
        if follows:
            e.r.q = path[-1].copy()
    e.move = move
    return e


def args(distance=.15, margin=.015, contact=False):
    return SimpleNamespace(advance_m=distance, advance_step_m=.01, advance_speed_mps=.01, advance_margin_m=margin, allow_contact=contact)


class Scene:
    def __init__(self): self.checks = 0
    def check(self, q): self.checks += 1


def test_dry_run_reports_clearance_and_does_not_move(tmp_path):
    e, report = fake(tmp_path), {}
    advance(e, Scene(), args(), report, execute=False)
    a = report['advance']
    assert not e.moves and a['executed_m'] == 0
    assert a['clearance_m'] == pytest.approx(.19, abs=.003)             # nearest held point at .10 m, wall at .29 m
    assert a['wall_tilt_deg'] < 1 and a['requested_m'] == .15


def test_full_stroke_is_taken_in_guarded_steps_along_the_wall_normal(tmp_path):
    e, report, scene = fake(tmp_path, held_depth=.10), {}, Scene()
    advance(e, scene, args(.12), report, execute=True)
    assert report['advance']['executed_m'] == pytest.approx(.12, abs=.002) and len(e.moves) == 12
    assert np.allclose(e.r.q[:3], [0, 0, .12], atol=.002) and scene.checks >= 12                 # straight along +z, collision-checked
    assert e.events == ['ADVANCE_START', 'ADVANCE_DONE'] and (tmp_path / 'advance.jsonl').exists()


def test_travel_stops_short_of_the_wall_unless_contact_is_allowed(tmp_path):
    e, report = fake(tmp_path, held_depth=.17), {}                       # held object 120 mm from the wall
    advance(e, Scene(), args(.15, margin=.015), report, execute=True)
    assert report['advance']['trimmed'] and report['advance']['executed_m'] == pytest.approx(.105, abs=.003)
    e2, report2, scene2 = fake(tmp_path, held_depth=.17), {}, Scene()
    advance(e2, scene2, args(.15, contact=True), report2, execute=True)
    assert not report2['advance']['trimmed'] and report2['advance']['executed_m'] == pytest.approx(.15, abs=.003) and scene2.checks == 0


def test_stalled_arm_stops_the_advance_and_marks_the_run_failed(tmp_path):
    e, report = fake(tmp_path, follows=False), {'status': 'CONVERGED'}
    run_advance(e, Scene(), args(), report, execute=True)
    assert report['status'] == 'ADVANCE_FAILED' and 'NO_PROGRESS' in report['advance']['error'] and len(e.moves) == 1


def test_missing_wall_refuses_before_moving(tmp_path):
    e, report = fake(tmp_path), {'status': 'CONVERGED'}
    e.last_observation['depth_m'][:] = 0
    run_advance(e, Scene(), args(), report, execute=True)
    assert report['status'] == 'ADVANCE_FAILED' and 'wall not measured' in report['advance']['error'] and not e.moves


def test_advance_off_by_default(tmp_path):
    e, report = fake(tmp_path), {'status': 'CONVERGED'}
    run_advance(e, Scene(), args(0.), report, execute=True)
    assert 'advance' not in report and not e.moves


def test_chunks_flow_through_centimetre_waypoints_and_settle_is_restored(tmp_path):
    e, report = fake(tmp_path), {}
    a = args(.09); a.advance_step_m = .03
    advance(e, Scene(), a, report, execute=True)
    assert len(e.moves) == 3 and all(len(p) == 4 for p in e.paths)             # start + 3 waypoints per 3 cm chunk
    assert np.allclose(np.diff(e.paths[0][:, 2]), .01, atol=1e-4)               # one waypoint per centimetre along +z
    assert report['advance']['executed_m'] == pytest.approx(.09, abs=.002) and e.c['arrival_settle_s'] == .5
