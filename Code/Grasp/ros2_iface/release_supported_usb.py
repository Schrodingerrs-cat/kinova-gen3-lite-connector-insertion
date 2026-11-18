#!/usr/bin/env python3
"""Open gripper ONLY after operator supports the extracted USB; no arm commands."""
import argparse,json
from pathlib import Path
import numpy as np,rclpy
from scipy.spatial.transform import Rotation
from robot_ros import Robot
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
if not args.execute:print('No commands. Support USB with fingers clear, then use --execute.');raise SystemExit(0)
root=Path(__file__).resolve().parents[1];record=json.loads((root/'logs/cad_lift_50mm_attempt.json').read_text())
if record.get('status')!='returned':raise RuntimeError('No completed50mm lift record')
expected=np.array(record['measured']);rclpy.init();robot=Robot('release_supported_usb_no_arm')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    actual=robot.tool_matrix()
    if np.linalg.norm(actual[:3,3]-expected[:3,3])>.003 or Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Arm no longer at recorded lift endpoint')
    print('OPENING GRIPPER ONLY. Operator must support USB; arm remains stationary.',flush=True)
    result=robot.gripper(0.,20.,timeout=20.)
    robot.spin_until(lambda:False,2.)
    print('Result:',result,flush=True);print('Measured gripper position:',robot.grip_pos_msr,flush=True)
    if robot.grip_pos_msr is None or robot.grip_pos_msr>.05:raise RuntimeError('Open state not confirmed; do not pull connector or home')
    print('Gripper open. No arm movement commanded.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
