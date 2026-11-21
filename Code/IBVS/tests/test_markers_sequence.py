"""All-markers loop: every marker left to right, align -> forward -> back out. Fake callables: logic checks only."""
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
from joint2_priority_servo import advance, retract, run_markers  # noqa: E402
from test_advance_run import Scene, args, fake  # noqa: E402


def drive(statuses, sequence=(10, 11, 12), advance_error_at=None):
    calls, report = [], {}

    def next_marker(marker_id):
        calls.append(('next', marker_id))

    def servo_one(sub):
        k = sum(c[0] == 'servo' for c in calls)
        calls.append(('servo', k))
        sub.update(status=statuses[k], sequence=list(sequence), advance=dict(executed_m=.15))
        if advance_error_at == k:
            sub['advance']['error'], sub['status'] = 'wall not measured', 'ADVANCE_FAILED'      # what run_advance records

    run_markers(report, next_marker, servo_one, lambda sub: calls.append(('retract', sub['advance']['executed_m'])))
    return calls, report


def test_every_marker_is_visited_in_order_with_a_retract_after_each():
    calls, report = drive(['CONVERGED', 'ALREADY_ALIGNED', 'CONVERGED'])
    assert [c for c in calls if c[0] == 'next'] == [('next', None), ('next', 11), ('next', 12)]      # first by order, then the sequence
    assert [c[0] for c in calls] == ['next', 'servo', 'retract'] * 3
    assert report['status'] == 'CONVERGED' and report['sequence'] == [10, 11, 12] and len(report['markers']) == 3


def test_preview_looks_only_at_the_first_marker():
    calls, report = drive(['PREVIEW'])
    assert report['status'] == 'PREVIEW' and report['sequence'] == [10, 11, 12] and not any(c[0] == 'retract' for c in calls)


def test_stops_at_the_first_marker_that_does_not_converge():
    calls, report = drive(['CONVERGED', 'FAILED'])
    assert report['status'] == 'FAILED' and len(report['markers']) == 2 and [c for c in calls if c[0] == 'retract'] == [('retract', .15)]


def test_failed_forward_move_stops_the_sequence_without_a_retract_or_next_marker():
    calls, report = drive(['CONVERGED', 'CONVERGED'], advance_error_at=1)
    assert len(report['markers']) == 2 and sum(c[0] == 'retract' for c in calls) == 1 and report['status'] == 'ADVANCE_FAILED'
    assert 'error' in report['markers'][1]['advance']                          # the failure is on record; the run stops here


def test_exception_inside_a_marker_propagates_but_keeps_what_was_done(tmp_path):
    report = {}

    def servo_one(sub):
        if len(report['markers']) == 2:
            raise RuntimeError('tracking recovery limit reached')
        sub.update(status='CONVERGED', sequence=[10, 11, 12])
    with pytest.raises(RuntimeError, match='tracking recovery'):
        run_markers(report, lambda i: None, servo_one, lambda sub: None)
    assert len(report['markers']) == 2 and report['markers'][0]['status'] == 'CONVERGED'


def test_retract_returns_to_the_standoff_without_collision_checks(tmp_path):
    e, report = fake(tmp_path), {}
    advance(e, Scene(), args(.09), report, execute=True)
    start = np.zeros(6)
    assert np.allclose(e.r.q[:3], [0, 0, .09], atol=.002)
    a = args(.09); a.advance_step_m = .03
    retract(e, a, report['advance'])
    assert np.allclose(e.r.q[:3], start[:3], atol=.003) and report['advance']['retracted_m'] == pytest.approx(.09, abs=.003)
    assert e.events[-2:] == ['RETRACT_START', 'RETRACT_DONE']


def test_retract_does_nothing_when_the_arm_never_advanced(tmp_path):
    e, report = fake(tmp_path), {}
    advance(e, Scene(), args(.15, margin=1.0), report, execute=True)          # margin 1 m: trimmed to nothing
    n = len(e.moves)
    retract(e, args(), report['advance'])
    assert len(e.moves) == n and 'retracted_m' not in report['advance']
