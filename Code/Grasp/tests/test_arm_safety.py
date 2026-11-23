"""Joint limit, fault and arrival checks of arm.go_joints (fake robot, no ROS graph). Idea and values from Sahil Gajera's branch."""
import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_iface"))
from arm import Arm  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402

CFG = load_config(ROOT / "config" / "grasp.yaml", allow_unscanned=True)
HOME = [3.0, -3.1, 93.4, -88.5, -87.7, -82.6]
GOTO = [40, 20, 100, -88, -87, -82]


class FakeRobot:
    def __init__(self, q_deg, follow=True):
        self.q, self.fault, self.follow, self.sent = np.radians(q_deg), False, follow, []

    def move_joints(self, q_deg, duration):
        self.sent.append(q_deg)
        if self.follow:
            self.q = np.radians(q_deg)


def make(follow=True):
    return Arm(FakeRobot(np.array(HOME, float), follow), CFG["motion"], None, 0.0, 0.8)


def test_scan_pose_with_joint_3_at_161_deg_is_refused():
    arm = make()
    assert "joint 3" in arm.joint_limit_error([18.164, 29.433, 161.099, 267.225, 340.453, 293.253])
    with pytest.raises(RuntimeError, match="outside the limits"):
        arm.go_joints([18.2, 29.4, 161.1, -92.8, -19.5, -66.7])
    assert not arm.r.sent


def test_joint_3_value_that_worked_before_is_allowed_and_angles_are_wrapped():
    assert make().joint_limit_error([3, -3, 145.2, -88, -87, -82]) is None
    assert make().joint_limit_error([3.0, 356.9, 93.4, 271.5, 272.3, 277.4]) is None


def test_the_recorded_scan_poses_inside_the_limits_are_known():
    arm = make()
    assert [k for k, q in enumerate(CFG["scan"]["views_joints_deg"]) if arm.joint_limit_error(q)] == [0]   # re-record pose 1 (joint 3 = 161.1), then this must be []


def test_faulted_arm_does_not_move():
    arm = make()
    arm.r.fault = True
    with pytest.raises(RuntimeError, match="fault"):
        arm.go_joints(GOTO)
    assert not arm.r.sent


def test_arm_that_ignores_the_move_is_caught():
    with pytest.raises(RuntimeError, match="stopped short"):
        make(follow=False).go_joints(GOTO)


def test_normal_move_passes():
    arm = make()
    arm.go_joints(GOTO)
    assert len(arm.r.sent) == 1
