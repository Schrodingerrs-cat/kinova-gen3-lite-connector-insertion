"""Joint limit, fault and arrival checks of arm.go_joints (fake robot, no ROS graph)."""
import pathlib, sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_iface"))
from arm import Arm  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402

CFG = load_config(ROOT / "config" / "grasp.yaml", allow_unscanned=True)
MOTION, SCAN = CFG["motion"], CFG["scan"]


class FakeRobot:
    def __init__(self, q_deg, follow=True):
        self.q, self.fault, self.follow, self.sent = np.radians(q_deg), False, follow, []

    def move_joints(self, q_deg, duration):
        self.sent.append(q_deg)
        if self.follow:
            self.q = np.radians(q_deg)


def make(q_deg, follow=True):
    return Arm(FakeRobot(np.array(q_deg, float), follow), MOTION, None, 0.0, 0.8)


HOME = [3.0, -3.1, 93.4, -88.5, -87.7, -82.6]


def test_the_pose_that_faulted_the_arm_is_refused():
    arm = make(HOME)
    assert "joint 3" in arm.joint_limit_error([18.164, 29.433, 161.099, 267.225, 340.453, 293.253])
    with pytest.raises(RuntimeError, match="outside the limits"):
        arm.go_joints([18.2, 29.4, 161.1, -92.8, -19.5, -66.7])
    assert not arm.r.sent


def test_joint_3_value_that_worked_before_is_allowed():
    assert make(HOME).joint_limit_error([3, -3, 145.2, -88, -87, -82]) is None


def test_angles_are_wrapped_before_the_check():
    assert make(HOME).joint_limit_error([3.0, 356.9, 93.4, 271.5, 272.3, 277.4]) is None


def test_other_scan_poses_are_inside_the_limits():
    arm = make(HOME)
    bad = [k for k, q in enumerate(SCAN["views_joints_deg"]) if arm.joint_limit_error(q)]
    assert bad in ([], [0])      # pose 1 (joint 3 = 161.1) is the known bad one until it is re-recorded


def test_faulted_arm_does_not_move():
    arm = make(HOME)
    arm.r.fault = True
    with pytest.raises(RuntimeError, match="fault"):
        arm.go_joints([10, 0, 90, -88, -87, -82])
    assert not arm.r.sent


def test_arm_that_does_not_follow_is_caught():
    arm = make(HOME, follow=False)
    with pytest.raises(RuntimeError, match="stopped short"):
        arm.go_joints([40, 20, 100, -88, -87, -82])


def test_normal_move_passes():
    arm = make(HOME)
    arm.go_joints([40, 20, 100, -88, -87, -82])
    assert len(arm.r.sent) == 1


def test_move_into_the_fixture_is_refused_before_it_starts():
    arm = make(HOME)
    arm.r.path_collision = lambda q0, q1, step: "collision at 60 % of the move: fixture_NAUO43 / right_finger_dist_link"
    with pytest.raises(RuntimeError, match="refused.*fixture_NAUO43"):
        arm.go_joints([40, 20, 100, -88, -87, -82])
    assert not arm.r.sent


def test_collision_check_can_be_switched_off():
    arm = make(HOME)
    arm.r.path_collision = lambda q0, q1, step: "collision"
    arm.m = dict(arm.m, scene_collision_check=False)
    arm.go_joints([40, 20, 100, -88, -87, -82])
    assert len(arm.r.sent) == 1
