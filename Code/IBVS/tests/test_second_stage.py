"""Second target after the forward move: servo to a saved goal (photo of the insertion pose) on the centre marker only."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
import joint2_priority_servo as j2  # noqa: E402
import alignment_test as at  # noqa: E402
from test_capture_path import experiment  # noqa: E402


def run(monkeypatch, first_status='CONVERGED', advance=None, then_goal='audio_insertion'):
    calls, e = [], SimpleNamespace(new_marker=lambda *a: calls.append(('new_marker', a)))

    def servo_one(_e, args, report):
        calls.append(('servo_one', args.advance_m))
        if len([c for c in calls if c[0] == 'servo_one']) == 1:
            report.update(status=first_status, **({'advance': advance} if advance else {}))
        else:
            report.update(status='CONVERGED')
    monkeypatch.setattr(j2, 'servo_one', servo_one)
    args = SimpleNamespace(all_markers=False, then_goal=then_goal, advance_m=.15)
    report = {}
    j2.servo(e, args, report)
    return calls, report


def test_second_stage_follows_a_good_first_stage_with_the_saved_goal_and_no_more_forward_move(monkeypatch):
    calls, report = run(monkeypatch)
    assert [c[0] for c in calls] == ['servo_one', 'new_marker', 'servo_one'] and calls[2][1] == 0.        # second stage: advance 0
    marker_id, order, goal = calls[1][1]
    assert marker_id == 11 and order == 'nearest' and np.allclose(goal, at.taught_goal('audio_insertion')['corners_image_px'])
    assert 'then' in report and report['status'] == 'CONVERGED'


@pytest.mark.parametrize('first_status,advance,then_goal', [('FAILED', None, 'audio_insertion'), ('CONVERGED', {'error': 'x'}, 'audio_insertion'),
                                                          ('PREVIEW', None, 'audio_insertion'), ('CONVERGED', None, None)])
def test_no_second_stage_after_a_failed_or_previewed_first_stage_or_without_a_goal(monkeypatch, first_status, advance, then_goal):
    calls, report = run(monkeypatch, first_status, advance, then_goal)
    assert [c[0] for c in calls] == ['servo_one'] and 'then' not in report


def test_the_insertion_goal_saved_from_the_photo_is_centre_marker_11():
    goal = at.taught_goal('audio_insertion')
    corners = np.array(goal['corners_image_px'])
    assert goal['marker_id'] == 11 and corners.shape == (4, 2)
    assert np.allclose(np.linalg.norm(corners[1] - corners[0]), 103, atol=3) and np.allclose(corners.mean(0), [457, 252.5], atol=3)
    with pytest.raises(ValueError, match='no taught goal named nope'):
        at.taught_goal('nope')


def test_new_marker_points_the_tracker_at_the_goal_and_the_display_is_reset(tmp_path):
    e = experiment(tmp_path)
    goal = at.taught_goal('audio_insertion')['corners_image_px']
    e.r.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
    e.new_marker(11, 'nearest', goal)
    assert e.tracker.forced == 11 and e.tracker.m['taught_goal_px'] == goal and e.display.marker_id == -1 and e.phase == 'ALIGN'


def test_no_scene_stand_in_does_nothing():
    scene = j2.NoScene()
    assert scene.install(1., None) is None and scene.check(np.zeros(6)) is None and scene.close() is None
