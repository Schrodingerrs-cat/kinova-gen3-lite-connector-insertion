import pathlib
import subprocess
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]


def test_scan_uses_seven_recorded_joint_poses():
    sc = yaml.safe_load(open(ROOT / "config" / "grasp.yaml"))["scan"]
    jv = sc["views_joints_deg"]
    assert len(jv) == 7 and all(len(q) == 6 and all(0 <= a < 360 for a in q) for q in jv)
    assert "views" not in sc                                   # the old Cartesian list is gone: the node must not mix the two


def test_scan_poses_check_tool_runs_and_every_pose_sees_the_head():
    out = subprocess.run([sys.executable, str(ROOT / "tools" / "check_scan_poses.py")], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert out.stdout.count("in view") == 7 and "NOT in view" not in out.stdout
