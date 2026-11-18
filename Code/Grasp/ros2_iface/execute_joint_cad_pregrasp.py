#!/usr/bin/env python3
"""Explicit, slow execution of the saved Cartesian pregrasp only. No gripper calls."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPositionFK
from robot_ros import Robot,JOINTS,BASE,TOOL,matrix
from run_seven_view_scan import call

ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');ap.add_argument('--speed-deg-s',type=float,default=6.,help='conservative joint timing scale, 1..10 deg/s; default6');args=ap.parse_args()
if not np.isfinite(args.speed_deg_s) or not 1<=args.speed_deg_s<=10:
    ap.error('speed must be in [1,10] deg/s')
if not args.execute:
    print('No motion. --execute moves ONLY to the saved pregrasp.');raise SystemExit(0)
path=Path(__file__).resolve().parents[1]/'logs/joint_cad_pregrasp_plan.json'
saved=json.loads(path.read_text())
if not 0<=time.time()-saved['planned_at']<=120:
    raise RuntimeError('Plan older than120 seconds; rerun plan_joint_cad_pregrasp.py --joint6-step-deg 6')
if saved.get('route')!='direct Cartesian position and orientation interpolation' or saved.get('standoff_m',0)<.1:
    raise RuntimeError('Unexpected plan type or standoff')
points=np.array(saved['path_rad'],float);expected=np.array(saved['start_joints_rad'],float)
if points.ndim!=2 or points.shape[1]!=6 or len(points)<2 or not np.isfinite(points).all():raise RuntimeError('Invalid joint path')
limits=np.array(saved['joint_step_limits_deg'],float)
if limits.shape!=(6,) or np.any(limits>[5,5,5,5,5,6]) or np.any(limits<=0):raise RuntimeError('Unexpected waypoint screens')
if np.any(np.degrees(abs(np.diff(np.vstack([expected,points]),axis=0))).max(axis=0)>limits+1e-8):raise RuntimeError('Stored path exceeds waypoint screens')
rclpy.init();robot=Robot('execute_cartesian_pregrasp_only')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp']
    gates=call(robot,GetParameters,'/grasp_node/get_parameters',req)
    if len(gates.values)!=3 or any(v.type!=1 for v in gates.values) or [v.bool_value for v in gates.values]!=[True,True,False]:raise RuntimeError('Requires dry_run=True, pregrasp_only=True, allow_grasp=False')
    actual=robot.tool_matrix();start=np.array(saved['start_tool_matrix']);goal=np.array(saved['target_tool_matrix'])
    if np.max(abs(robot.q-expected))>np.deg2rad(.5):raise RuntimeError('Starting joints changed; replan')
    if np.linalg.norm(actual[:3,3]-start[:3,3])>.002 or Rotation.from_matrix(start[:3,:3].T@actual[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Starting tool pose changed; replan')
    fk=GetPositionFK.Request();fk.header.frame_id=BASE;fk.fk_link_names=[TOOL];fk.robot_state.joint_state.name=JOINTS;fk.robot_state.joint_state.position=points[-1].tolist()
    res=call(robot,GetPositionFK,'/compute_fk',fk)
    if res.error_code.val!=1 or len(res.pose_stamped)!=1:raise RuntimeError('Endpoint FK check unavailable')
    p=res.pose_stamped[0].pose;end=matrix([p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    if np.linalg.norm(end[:3,3]-goal[:3,3])>.002 or Rotation.from_matrix(end[:3,:3].T@goal[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Path endpoint does not match pregrasp')
    # Zero-velocity knots, with conservative durations for the largest joint change.
    dt=np.maximum(.25,2*np.max(abs(np.diff(np.vstack([robot.q,points]),axis=0)),axis=1)/np.deg2rad(args.speed_deg_s))
    times=np.cumsum(dt);print('PREGRASP ONLY: %.1f seconds; no gripper commands. Node dry-run remains True.'%times[-1],flush=True)
    if time.time()-saved['planned_at']>120:raise RuntimeError('Plan expired during checks; replan')
    path.rename(path.with_suffix('.consumed.json'))
    robot.send_joint_path(points.tolist(),times.tolist(),velocities=np.zeros_like(points).tolist(),timeout=float(times[-1]+30))
    measured=robot.tool_matrix();print('Measured tool XYZ:',np.round(measured[:3,3],4),flush=True)
    if np.linalg.norm(measured[:3,3]-goal[:3,3])>.003:raise RuntimeError('Measured endpoint differs by >3 mm; stop and inspect')
    print('PREGRASP ONLY completed. No descent, closure or lift.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
