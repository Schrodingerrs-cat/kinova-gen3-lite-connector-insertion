#!/usr/bin/env python3
"""Supervised additional50mm vertical lift. No gripper commands; no automatic retry."""
import argparse,json
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import GetParameters
from robot_ros import Robot
from run_seven_view_scan import call
ROOT=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
record=json.loads((ROOT/'logs/cad_lift_5mm_attempt.json').read_text())
closure=json.loads((ROOT/'logs/final_cad_grasp_attempt.json').read_text())
if record.get('status')!='returned':raise RuntimeError('No completed5mm lift record')
expected=np.array(record['measured']);marker=ROOT/'logs/cad_lift_50mm_attempt.json'
if args.execute and marker.exists():raise RuntimeError('Lift already attempted; inspect before another command')
rclpy.init();robot=Robot('cad_50mm_lift')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp'];gates=call(robot,GetParameters,'/grasp_node/get_parameters',req)
    if len(gates.values)!=3 or any(v.type!=1 for v in gates.values) or [v.bool_value for v in gates.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False grasp gates')
    start=robot.tool_matrix();q=robot.q.copy()
    if np.linalg.norm(start[:3,3]-expected[:3,3])>.003 or Rotation.from_matrix(expected[:3,:3].T@start[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Not at recorded5mm lift endpoint; repeated lift refused')
    if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5):raise RuntimeError('No gripper feedback')
    if abs(robot.grip_pos_msr-closure['position'])>.02:raise RuntimeError('Gripper position changed after closure')
    goal=start.copy();goal[2,3]+=.050
    path=robot.line_path(goal,q,max_step_m=.001,min_fraction=1-1e-6)
    if path is None or len(path)<2:raise RuntimeError('No complete Cartesian lift path')
    joints=np.vstack([q,np.asarray(path)])
    if not np.isfinite(joints).all() or np.degrees(abs(np.diff(joints,axis=0))).max()>5:raise RuntimeError('Lift path joint steps rejected')
    print('Start tool XYZ:',np.round(start[:3,3],4),flush=True);print('Goal tool XYZ:',np.round(goal[:3,3],4),flush=True)
    print('Additional50mm vertical lift; gripper command unchanged; path points:',len(path),flush=True)
    if args.execute:
        if np.max(abs(robot.q-q))>np.deg2rad(.5):raise RuntimeError('Starting joints changed')
        with marker.open('x') as stream:json.dump(dict(status='requested',start=start.tolist(),goal=goal.tolist()),stream,indent=2)
        print('MOVING50mm upward over5seconds. Stop if board moves, housing slips or cable tightens.',flush=True)
        robot.send_line(path,5.)
        end=robot.tool_matrix();print('Measured tool XYZ:',np.round(end[:3,3],4),flush=True)
        marker.write_text(json.dumps(dict(status='returned',goal=goal.tolist(),measured=end.tolist()),indent=2))
        if np.linalg.norm(end[:3,3]-goal[:3,3])>.003:raise RuntimeError('Lift endpoint mismatch; inspect')
        print('Stopped. Physical extraction/holding must be checked visually.',flush=True)
    else:print('PLAN ONLY: no movement. Grip not verified by encoder reading; visual supervision required.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
