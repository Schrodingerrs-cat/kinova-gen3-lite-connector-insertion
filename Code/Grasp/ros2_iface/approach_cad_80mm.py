#!/usr/bin/env python3
"""Cartesian 80mm approach from the verified100mm pregrasp. No gripper commands.
Default is plan-only. --execute moves, leaving20mm to the fitted grasp.
"""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import GetParameters
from robot_ros import Robot
from run_seven_view_scan import call
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from housing_grasp.config import load_config
from housing_grasp.cartesian_sampling import pose_samples
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
saved=json.loads((ROOT/'logs/joint_cad_pregrasp_plan.consumed.json').read_text());expected=np.array(saved['target_tool_matrix']);axis=np.array(saved['target_tcp_matrix'])[:3,2]
if not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1)>.001 or saved['standoff_m']!=.1:raise RuntimeError('Unexpected source pregrasp')
goal=expected.copy();goal[:3,3]+=.08*axis
cfg=load_config(ROOT/'config/grasp.yaml');table=np.array(cfg['gripper']['position_to_width_m'])
rclpy.init();robot=Robot('cad_80mm_approach')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp'];res=call(robot,GetParameters,'/grasp_node/get_parameters',req)
    if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False grasp gates')
    actual=robot.tool_matrix();qstart=robot.q.copy()
    if np.linalg.norm(actual[:3,3]-expected[:3,3])>.003 or Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Not at verified100mm pregrasp; repeated descent refused')
    if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5):raise RuntimeError('No gripper feedback')
    gap=float(np.interp(robot.grip_pos_msr,table[:,0],table[:,1]))
    if abs(gap-.0315)>.005:raise RuntimeError('Jaw opening is not near31.5mm; no descent')
    samples=pose_samples(actual,goal,translation_step_m=.001,rotation_step_deg=1.)
    path=robot.line_path(goal,qstart,max_step_m=.001,min_fraction=1-1e-6,waypoints=samples)
    if path is None or len(path)<2:raise RuntimeError('No complete Cartesian approach')
    points=np.array(path,float);joints=np.vstack([qstart,points])
    if not np.isfinite(joints).all():raise RuntimeError('Invalid joint path')
    peaks=np.degrees(abs(np.diff(joints,axis=0))).max(axis=0)
    if np.any(peaks>5):raise RuntimeError('Approach joint step exceeds5deg: '+str(peaks))
    print('Start tool XYZ:',np.round(actual[:3,3],4),flush=True)
    print('Goal tool XYZ:',np.round(goal[:3,3],4),flush=True)
    print('Cartesian approach:80mm along fitted axis; remaining standoff20mm',flush=True)
    print('Jaw gap estimate:%.1fmm | path points:%d | max joint steps deg:%s'%(gap*1000,len(path),np.round(peaks,2)),flush=True)
    record=dict(execute_requested=args.execute,start_tool=actual.tolist(),goal_tool=goal.tolist(),path_rad=points.tolist(),remaining_standoff_m=.02,planned_at=time.time(),source_fit=saved['source_fit'])
    (ROOT/'logs/cad_80mm_approach_plan.json').write_text(json.dumps(record,indent=2))
    if args.execute:
        if np.max(abs(robot.q-qstart))>np.deg2rad(.5):raise RuntimeError('Starting joints changed during planning')
        print('MOVING:80mm approach over8seconds. No gripper command.',flush=True)
        robot.send_line(path,8.)
        measured=robot.tool_matrix();print('Measured tool XYZ:',np.round(measured[:3,3],4),flush=True)
        if np.linalg.norm(measured[:3,3]-goal[:3,3])>.003:raise RuntimeError('Endpoint differs by >3mm; stop and inspect')
        print('Stopped20mm before fitted grasp. No closure or lift.',flush=True)
    else:print('PLAN ONLY: no motion commanded. Kinematic check only; scene/cable clearance requires supervision.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
