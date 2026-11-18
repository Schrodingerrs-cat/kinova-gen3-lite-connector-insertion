#!/usr/bin/env python3
"""Final20mm Cartesian approach then one gripper closure. Never lifts.
Default: plan only. --execute performs BOTH stages, with no automatic retry.
"""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import GetParameters
from control_msgs.action import GripperCommand
from robot_ros import Robot,JOINTS,BASE,TOOL,matrix
from moveit_msgs.srv import GetPositionFK
from run_seven_view_scan import call
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from housing_grasp.config import load_config
from housing_grasp.cartesian_sampling import pose_samples

ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
saved=json.loads((ROOT/'logs/joint_cad_pregrasp_plan.consumed.json').read_text());pre=np.array(saved['target_tool_matrix']);axis=np.array(saved['target_tcp_matrix'])[:3,2]
if not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1)>.001 or saved['standoff_m']!=.1:raise RuntimeError('Unexpected source pregrasp')
expected=pre.copy();expected[:3,3]+=.08*axis;goal=pre.copy();goal[:3,3]+=.10*axis
cfg=load_config(ROOT/'config/grasp.yaml');table=np.array(cfg['gripper']['position_to_width_m'])
marker=ROOT/'logs/final_cad_grasp_attempt.json'
if args.execute and marker.exists():raise RuntimeError('Final-grasp attempt already recorded; inspect outcome before any retry')
rclpy.init();robot=Robot('finish_cad_grasp_no_lift')

def gap():
    if robot.grip_pos_msr is None:raise RuntimeError('No gripper feedback')
    return float(np.interp(robot.grip_pos_msr,table[:,0],table[:,1]))

def pose_error(actual,target):
    return np.linalg.norm(actual[:3,3]-target[:3,3]),Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude()

try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp'];res=call(robot,GetParameters,'/grasp_node/get_parameters',req)
    if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False grasp gates')
    actual=robot.tool_matrix();qstart=robot.q.copy();error,angle=pose_error(actual,expected)
    if error>.003 or angle>np.deg2rad(1):raise RuntimeError('Not at verified20mm standoff; repeated approach refused')
    if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5):raise RuntimeError('No gripper feedback')
    if abs(gap()-.0315)>.005:raise RuntimeError('Jaw gap differs from verified31.5mm opening')
    samples=pose_samples(actual,goal,translation_step_m=.001,rotation_step_deg=1.)
    path=robot.line_path(goal,qstart,max_step_m=.001,min_fraction=1-1e-6,waypoints=samples)
    if path is None or len(path)<2:raise RuntimeError('No complete final Cartesian path')
    points=np.array(path,float);joints=np.vstack([qstart,points])
    if not np.isfinite(joints).all() or np.any(np.degrees(abs(np.diff(joints,axis=0))).max(axis=0)>5):raise RuntimeError('Final approach joint path rejected')
    fk=GetPositionFK.Request();fk.header.frame_id=BASE;fk.fk_link_names=[TOOL];fk.robot_state.joint_state.name=JOINTS;fk.robot_state.joint_state.position=points[-1].tolist()
    fkres=call(robot,GetPositionFK,'/compute_fk',fk)
    if fkres.error_code.val!=1 or len(fkres.pose_stamped)!=1:raise RuntimeError('Final endpoint FK unavailable')
    p=fkres.pose_stamped[0].pose;end=matrix([p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);error,angle=pose_error(end,goal)
    if error>.002 or angle>np.deg2rad(1):raise RuntimeError('Final path endpoint does not match target')
    print('Start tool XYZ:',np.round(actual[:3,3],4),flush=True);print('Grasp tool XYZ:',np.round(goal[:3,3],4),flush=True)
    print('Final approach:20mm, then closure position %.3f, effort20. NO LIFT.'%cfg['gripper']['close_cmd'],flush=True)
    if not args.execute:
        print('PLAN ONLY: no arm or gripper commands sent.',flush=True)
    else:
        if np.max(abs(robot.q-qstart))>np.deg2rad(.5):raise RuntimeError('Starting joints changed during planning')
        record=dict(started_at=time.time(),status='approach_requested',start_tool=actual.tolist(),goal_tool=goal.tolist(),path_rad=points.tolist(),source_fit=saved['source_fit'])
        # Exclusive creation prevents a second instance from repeating the approach.
        with marker.open('x') as stream:json.dump(record,stream,indent=2)
        print('MOVING: final20mm approach over3seconds.',flush=True)
        robot.send_line(path,3.)
        measured=robot.tool_matrix();error,angle=pose_error(measured,goal)
        print('Measured tool XYZ:',np.round(measured[:3,3],4),flush=True)
        if error>.003 or angle>np.deg2rad(1):raise RuntimeError('Endpoint mismatch: gripper closure NOT sent')
        if abs(gap()-.0315)>.005:raise RuntimeError('Opening changed: closure NOT sent')
        record['status']='closure_requested';marker.write_text(json.dumps(record,indent=2))
        command=GripperCommand.Goal();command.command.position=float(cfg['gripper']['close_cmd']);command.command.max_effort=20.
        future=robot.grip.send_goal_async(command)
        if not robot.spin_until(future.done,5):raise RuntimeError('Gripper acknowledgment timed out; do not retry')
        handle=future.result()
        if handle is None or not handle.accepted:raise RuntimeError('Gripper closure rejected; arm stays at grasp pose')
        result_future=handle.get_result_async()
        if not robot.spin_until(result_future.done,20):
            cancellation=handle.cancel_goal_async();robot.spin_until(cancellation.done,3)
            raise RuntimeError('Closure result timeout; cancellation requested. No lift or retry')
        wrapped=result_future.result();result=wrapped.result
        # Read feedback after settling; action completion previously preceded mechanical settling.
        robot.spin_until(lambda:False,2.)
        width=gap();record.update(status='closure_result_received_no_lift',action_status=int(wrapped.status),reached_goal=bool(result.reached_goal),stalled=bool(result.stalled),position=float(robot.grip_pos_msr),gap_estimate_mm=width*1000)
        marker.write_text(json.dumps(record,indent=2))
        print('Closure action status:',wrapped.status,'reached_goal:',result.reached_goal,'stalled:',result.stalled,flush=True)
        print('Settled position %.4f; estimated gap %.1fmm'%(robot.grip_pos_msr,width*1000),flush=True)
        print('NO LIFT commanded. Confirm both pads contact the rigid housing before any lift.',flush=True)
        if width<.003:print('WARNING: near-empty closure reading; do not infer a successful grasp.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
