"""Offline checks of ROS request fields and stopping on missing feedback."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("controller_manager_msgs.srv")
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("ibvs_node", ROOT / "ros2_iface" / "ibvs_node.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_controller_switch_uses_installed_ros_request_fields():
    node = module.Ibvs.__new__(module.Ibvs)
    node.live, node.r, node.sw = True, Mock(), Mock()
    node.r.spin_until.return_value = True
    node.sw.call_async.return_value.result.return_value = SimpleNamespace(ok=True)
    assert node.switch("twist_controller", "joint_trajectory_controller")
    request = node.sw.call_async.call_args.args[0]
    if hasattr(request, "activate_controllers"):
        assert request.activate_controllers == ["twist_controller"]
        assert request.deactivate_controllers == ["joint_trajectory_controller"]
    else:
        assert request.start_controllers == ["twist_controller"]
        assert request.stop_controllers == ["joint_trajectory_controller"]


def test_controller_switch_timeout_cancels_pending_request():
    node = module.Ibvs.__new__(module.Ibvs)
    node.live, node.r, node.sw = True, Mock(), Mock()
    node.r.spin_until.return_value = False
    assert not node.switch("twist_controller", "joint_trajectory_controller")
    node.sw.call_async.return_value.cancel.assert_called_once()


@pytest.mark.parametrize("last_stamp", [0.0, 99.95])
def test_camera_loss_stops_without_waiting_for_another_frame(last_stamp):
    node = module.Ibvs.__new__(module.Ibvs)
    clock = SimpleNamespace(now=100.0)
    node.now = lambda: clock.now
    node.c = dict(robot=dict(twist_controller="twist", traj_controller="joints", workspace_box_B_m=[-1,1,-1,1,-1,1]),
                  ibvs=dict(tol_px=2, converge_frames=3, timeout_s=10),
                  guards=dict(camera_stale_s=0.2, joint_stale_s=1), markers=dict(max_missing_frames=3))
    node.joint_t, node.last_stamp = 100.0, last_stamp
    node.switch, node.zero = Mock(return_value=True), Mock()
    node.tool_xyz, node.frame = Mock(return_value=[0,0,0]), Mock(return_value=None)
    node.r = Mock()
    node.r.spin_until.side_effect = lambda *args: setattr(clock, "now", clock.now + 0.3)
    assert node.servo_align() == "CAMERA_STALE"
    node.zero.assert_called()
    assert node.frame.call_count == 1
    assert node.switch.call_args.args == ("joints", "twist")
