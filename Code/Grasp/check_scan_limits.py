from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

path = Path(get_package_share_directory("kortex_description"))
robot = ET.parse(path / "robots/gen3_lite.urdf").getroot()

with open("config/grasp.yaml") as f:
    scan = yaml.safe_load(f)["scan"]

poses = [scan["home_joints_deg"]] + scan["views_joints_deg"]
failures = 0

for index, pose in enumerate(poses):
    name = "home" if index == 0 else f"view {index}"
    angles = np.radians((np.array(pose) + 180) % 360 - 180)
    for joint, angle in enumerate(angles, 1):
        limit = robot.find(f"joint[@name='joint_{joint}']/limit")
        low, high = float(limit.get("lower")), float(limit.get("upper"))
        if not low <= angle <= high:
            print(name, f"joint {joint}: OUTSIDE LIMITS")
            failures += 1

print("Joint-limit violations:", failures)
