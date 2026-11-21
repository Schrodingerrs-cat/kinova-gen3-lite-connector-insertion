"""Direct joint path keeps the same limits as a planned one; fast profile stays inside the hard guards."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
import alignment_test  # noqa: E402
from joint2_priority_servo import fast_profile, smooth_profile  # noqa: E402
from ibvs.alignment import smooth_servo_path  # noqa: E402


def experiment(tmp_path, limit_deg=5.5):
    e = alignment_test.Experiment.__new__(alignment_test.Experiment)
    e.r = SimpleNamespace(q=np.zeros(6))
    e.c = dict(max_servo_joint_step_deg=limit_deg, max_camera_travel_m=1., max_camera_rotation_deg=90.)
    e.output, e.phase = tmp_path, 'ALIGN'
    e.fk = lambda q: np.eye(4)
    e.T_EC = e.T_tcp = np.eye(4)
    e.floor, e.origin = -1., None
    return e


def test_direct_path_is_start_plus_target(tmp_path):
    e = experiment(tmp_path)
    path = e.direct_path(np.radians([1, 0, 0, 0, 0, 0]))
    assert path.shape == (2, 6) and np.allclose(path[0], 0) and np.isclose(np.degrees(path[1, 0]), 1)


def test_direct_path_rejects_big_joint_step_and_logs_it(tmp_path):
    e = experiment(tmp_path)
    with pytest.raises(RuntimeError, match='step limit'):
        e.direct_path(np.radians([0, 0, 6, 0, 0, 0]))
    assert (tmp_path / 'rejected_paths.jsonl').exists()


def test_direct_path_rejects_non_finite(tmp_path):
    with pytest.raises(RuntimeError, match='non-finite'):
        experiment(tmp_path).direct_path(np.array([np.nan, 0, 0, 0, 0, 0]))


def test_direct_path_applies_geometric_limits(tmp_path):
    e = experiment(tmp_path)
    e.floor = 1.   # fingertip/camera below the floor
    with pytest.raises(ValueError, match='table clearance'):
        e.direct_path(np.radians([1, 0, 0, 0, 0, 0]))


def test_collision_sample_spacing_follows_config():
    path = np.array([np.zeros(6), np.radians([0, 0, 3, 0, 0, 0])])
    fine, coarse = smooth_servo_path(path, .25), smooth_servo_path(path, 1.)
    assert len(coarse) < len(fine)
    for samples, spacing in ((fine, .25), (coarse, 1.)):
        assert np.degrees(np.abs(np.diff(samples, axis=0))).max() <= spacing + 1e-9
        assert np.allclose(samples[0], path[0]) and np.allclose(samples[-1], path[-1])


def test_fast_profile_inside_hard_guards():
    c = {}
    fast_profile(c)
    assert c['gain'] <= 1 and 'control_hz' not in c
    approach, fine = c['approach_limits_deg'], c['fine_limits_deg']
    assert max(approach[i] for i in (0, 2, 3, 4, 5)) <= 3.   # the 3-degree guard on the non-joint-2 axes
    assert max(approach) <= 5.5 and all(f < a for f, a in zip(fine, approach))
    d = {}
    smooth_profile(d)
    assert c['joint_speed_deg_s'] > d['joint_speed_deg_s'] and c['min_move_s'] < d['min_move_s']


def test_velocities_flow_through_waypoints_and_stop_at_the_ends():
    from ibvs.alignment import smooth_velocities
    path = np.array([[0.] * 6, [.01] * 6, [.02] * 6, [.03] * 6]); times = np.array([.1, .2, .3, .4])
    v = smooth_velocities(path, times)
    assert np.allclose(v[0], 0) and np.allclose(v[-1], 0) and np.allclose(v[1:-1], .1)    # constant speed inside, no stops at knots
    back = smooth_velocities(np.array([[0.] * 6, [.01] * 6, [0.] * 6]), np.array([.1, .2, .3]))
    assert np.allclose(back[1], 0)                                                         # reversal: stops at the turning point


def test_continuous_timing_is_faster_than_the_stopping_profile():
    from ibvs.alignment import trajectory_times
    path = np.array([[0.] * 6, [np.radians(20)] + [0.] * 5, [np.radians(40)] + [0.] * 5])
    stop = trajectory_times(path, 10., 0., .1)[-1]
    flow = trajectory_times(path, 10., 0., .1, peak_factor=1.25)[-1]
    assert flow == pytest.approx(.1 + (stop - .1) * 1.25 / 1.875)


def test_speed_scale_multiplies_limits_and_shortens_waits():
    from alignment_test import scale_speeds
    c = dict(joint_speed_deg_s=5., linear_speed_m_s=.005, angular_speed_deg_s=2., min_move_s=.6, arrival_settle_s=.5,
             max_step_m=.002, max_step_deg=.4)
    scale_speeds(c, 2.)
    assert (c['joint_speed_deg_s'], c['linear_speed_m_s'], c['angular_speed_deg_s']) == (10., .01, 4.)
    assert c['min_move_s'] == .3 and c['arrival_settle_s'] == .25
    assert c['max_step_m'] == .004 and c['max_step_deg'] == .8                              # bigger steps: lateral moves of cm are not 2 mm at a time
    assert c['approach_limits_deg'] == [2., 5., 2., 2., 2., 2.] and c['fine_limits_deg'] == [.5, 1.5, .5, .5, .5, .5]
    d = dict(joint_speed_deg_s=5., linear_speed_m_s=.005, angular_speed_deg_s=2., min_move_s=.6, arrival_settle_s=.5, max_step_m=.002, max_step_deg=.4)
    scale_speeds(d, 1.)
    assert d['joint_speed_deg_s'] == 5. and d['max_step_m'] == .002 and d['min_move_s'] == .6    # 1.0 changes no speed or step


def test_scaled_step_limits_stay_under_the_hard_guards():
    from alignment_test import scale_speeds
    for scale in (1., 2., 3.):
        c = dict(joint_speed_deg_s=5., linear_speed_m_s=.005, angular_speed_deg_s=2., min_move_s=.6, arrival_settle_s=.5)
        scale_speeds(c, scale)
        assert max(c['approach_limits_deg']) <= 5. and max(c['approach_limits_deg'][i] for i in (0, 2, 3, 4, 5)) <= 2.5   # 3 deg guard on joints 1,3-6
        assert max(c['fine_limits_deg']) <= 1.5


def test_sequenced_path_moves_one_group_at_a_time_in_small_steps():
    from ibvs.alignment import sequenced_joint_path
    q0 = np.radians([0, 0, 0, 0, 0, 0.]); q1 = np.radians([30, -80, 40, -20, 200, -15.])
    path = sequenced_joint_path(q0, q1, [(0, 5), (4,), (2, 3), (1,)], step_deg=1.)
    assert np.allclose(path[0], q0) and np.allclose(path[-1], q1)
    assert np.degrees(np.abs(np.diff(path, axis=0))).max() <= 1.0 + 1e-9                  # sampled every degree
    first = path[:np.argmax(np.degrees(np.abs(path[:, 0] - q1[0])) < 1e-6) + 1]           # while joints 1 and 6 move, the others are still
    assert np.allclose(first[:, [1, 2, 3, 4]], 0)
    mid = path[(np.abs(path[:, 4] - q0[4]) > 1e-9) & (np.abs(path[:, 4] - q1[4]) > 1e-9)]   # while joint 5 moves, joints 1,6 are done, 2,3,4 not started
    assert np.allclose(mid[:, [0, 5]], q1[[0, 5]]) and np.allclose(mid[:, [1, 2, 3]], 0)
    assert len(np.unique(np.round(path, 9), axis=0)) == len(path)                         # no repeated points


def positioning_fixture(tmp_path, reject):
    e = alignment_test.Experiment.__new__(alignment_test.Experiment)
    e.r = SimpleNamespace(q=np.zeros(6)); e.checked, e.planned = [], []
    e.check_joint_path = lambda path, local=False, limits=True: e.checked.append(len(path))
    e.skip_checks, e.fk, e.T_tcp, e.T_EC, e.wrist_fk = False, lambda q: np.eye(4), np.eye(4), np.eye(4), None
    e.joint_plan = lambda target_deg: (e.planned.append(1), np.array([e.r.q, np.radians(target_deg)]))[1]

    class Scene:
        def check(self, q):
            if reject(q):
                raise RuntimeError("observed-scene collision")
    return e, Scene()


def test_positioning_takes_the_first_clear_candidate(tmp_path):
    e, scene = positioning_fixture(tmp_path, lambda q: False)
    path = e.positioning_path([30, -80, 40, -20, 200, -15], scene)
    assert not e.planned and np.allclose(path[-1], np.radians([30, -80, 40, -20, 200, -15]))
    assert np.allclose(path[1:30, 4], 0)                                                    # first candidate: joint 5 waits for joints 1 and 6 (30 samples)


def test_positioning_falls_back_to_the_next_candidate_then_straight_then_moveit(tmp_path):
    # reject any sample where joint 5 has moved but joints 1 and 6 have not: kills the second candidate only
    e, scene = positioning_fixture(tmp_path, lambda q: abs(q[4]) > .05 and abs(q[0]) < 1e-9)
    e.positioning_path([30, -80, 40, -20, 200, -15], scene)
    assert not e.planned
    e, scene = positioning_fixture(tmp_path, lambda q: True)                                 # everything collides
    path = e.positioning_path([30, -80, 40, -20, 200, -15], scene)
    assert e.planned == [1] and len(path) == 2


def test_positioning_error_lists_why_every_path_failed(tmp_path):
    e, scene = positioning_fixture(tmp_path, lambda q: True)
    e.joint_plan = lambda target_deg: (_ for _ in ()).throw(RuntimeError("MoveIt joint plan failed: 99999"))
    with pytest.raises(RuntimeError, match="no path to the target pose.*observed-scene collision.*MoveIt joint plan failed: 99999"):
        e.positioning_path([30, -80, 40, -20, 200, -15], scene)


def gripper_fixture(server=True, answers=(True,)):
    """Fake gripper action client: answers[i] False = no handle yet (controller still spawning)."""
    e = alignment_test.Experiment.__new__(alignment_test.Experiment)
    e.args, e.events, e.sent = SimpleNamespace(live=True), [], []
    e.grasp = dict(gripper=dict(close_cmd=.81, close_effort=30.))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: SimpleNamespace(result=lambda: "done"))

    def send(goal):
        i = len(e.sent); e.sent.append((goal.command.position, goal.command.max_effort))
        return SimpleNamespace(result=lambda: handle if answers[min(i, len(answers) - 1)] else None)
    e.r = SimpleNamespace(grip=SimpleNamespace(wait_for_server=lambda timeout_sec: server, send_goal_async=send),
                          spin_until=lambda *a: None, grip_pos_msr=.835)
    e.wait = lambda future, timeout=10.0, monitor=False: future.result()
    e.event = lambda name, **kw: e.events.append(name)
    return e


def test_close_gripper_retries_until_the_controller_answers():
    e = gripper_fixture(answers=(False, False, True))
    e.close_gripper()
    assert e.sent == [(.81, 30.)] * 3 and e.events == ["GRIPPER_CLOSED"]


def test_close_gripper_gives_clear_errors_when_the_controller_never_answers():
    with pytest.raises(RuntimeError, match="did not accept"):
        gripper_fixture(answers=(False,)).close_gripper()
    with pytest.raises(RuntimeError, match="action server is not available"):
        gripper_fixture(server=False).close_gripper()


def stale_fixture(skip):
    e = alignment_test.Experiment.__new__(alignment_test.Experiment)
    e.skip_checks, e.c = skip, dict(feedback_stale_s=.5)
    e.r = SimpleNamespace(q=np.zeros(6), joint_rx=0., get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10 * 10**9)),
                          latest_joints=None, _img={})
    return e


def test_stale_data_stops_the_run_unless_checks_are_off():
    with pytest.raises(RuntimeError, match="JOINTS_STALE"):
        stale_fixture(False).fresh(camera=True)
    stale_fixture(True).fresh(camera=True)                                    # --no-checks: stale joints and missing images do not stop it
    e = stale_fixture(True); e.r.q = np.full(6, np.nan)
    with pytest.raises(RuntimeError, match="no finite arm joint feedback"):   # but unknown joints still do: nothing can be commanded
        e.fresh()


def test_collision_scene_check_is_skipped_with_no_checks():
    from observed_scene import ObservedScene
    scene = ObservedScene.__new__(ObservedScene)
    scene.e = SimpleNamespace(skip_checks=True, state=lambda q: 1 / 0)         # would blow up if it queried MoveIt
    scene.check(np.zeros(6))


def test_positioning_takes_the_route_whose_lowest_point_is_highest(tmp_path):
    e, scene = positioning_fixture(tmp_path, lambda q: False)
    # a low spot where joint 2 is near -0.5 while joint 5 has already moved: routes that move joint 5 before joint 2 pass through it
    def fk(q):
        T = np.eye(4)
        T[2, 3] = 1.0 - .9 * np.exp(-50 * (q[1] + .5) ** 2) * (q[4] > .5)
        return T
    e.fk = fk
    target = np.degrees([0, -1., 0, 0, 1., 0])
    path = e.positioning_path(target, scene)
    assert e.lowest_height(path) > .99                                                       # avoided the low spot
    assert np.allclose(path[path[:, 1] > -.99][:, 4], 0, atol=1e-9) or e.lowest_height(path) > .99
    first_j5_move = np.argmax(np.abs(path[:, 4]) > 1e-9)
    assert np.isclose(path[first_j5_move, 1], -1.)                                           # joint 2 finished before joint 5 started


def test_no_checks_skips_the_floor_limit_for_positioning_only(tmp_path):
    seen = []
    e, scene = positioning_fixture(tmp_path, lambda q: False)
    e.check_joint_path = lambda path, local=False, limits=True: seen.append(limits)
    e.positioning_path([30, -80, 40, -20, 200, -15], scene)
    e.skip_checks = True
    e.positioning_path([30, -80, 40, -20, 200, -15], scene)
    assert seen == [True, False]
