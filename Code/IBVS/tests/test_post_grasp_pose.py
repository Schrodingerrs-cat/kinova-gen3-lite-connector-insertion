"""Keep the configured pose consistent with its recorded leveling evidence."""
import json
from pathlib import Path

import numpy as np
import yaml


def test_post_grasp_uses_recorded_leveled_pose():
    project = Path(__file__).resolve().parents[3]
    config = yaml.safe_load((project/'Code/Grasp/config/grasp.yaml').read_text())
    record = json.loads((project/'Media/IBVS/run_20261121_172647_leveling_post_grasp/leveled_pose.json').read_text())
    np.testing.assert_allclose(config['post_grasp']['joint_waypoints_deg'][-1],
                               record['joints_deg'], rtol=0, atol=1e-4)
    assert record['history_max_angle_deg'][-1] < .5
