#!/usr/bin/env python3
"""Set jaw opening at the verified pregrasp; no arm/descent/lift commands."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from robot_ros import Robot
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from housing_grasp.config import load_config
from rcl_interfaces.srv import GetParameters
from run_seven_view_scan import call
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
cfg=load_config(ROOT/'config/grasp.yaml');table=np.array(cfg['gripper']['position_to_width_m']);width=.0075+2*cfg['grasp']['open_margin_m'];position=float(np.interp(width,table[::-1,1],table[::-1,0]))
print('Requested jaw gap %.1f mm, gripper position %.4f'%(width*1000,position),flush=True)
if not args.execute:print('PLAN ONLY: no command sent');raise SystemExit(0)
saved=json.loads((ROOT/'logs/joint_cad_pregrasp_plan.consumed.json').read_text());goal=np.array(saved['target_tool_matrix'])
rclpy.init();robot=Robot('set_cad_pregrasp_opening')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp'];res=call(robot,GetParameters,'/grasp_node/get_parameters',req)
    if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False grasp gates')
    actual=robot.tool_matrix()
    if np.linalg.norm(actual[:3,3]-goal[:3,3])>.003 or Rotation.from_matrix(goal[:3,:3].T@actual[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Not at verified pregrasp; no gripper command sent')
    if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5):raise RuntimeError('No gripper feedback')
    goal_msg=GripperCommand.Goal()
    goal_msg.command.position=position;goal_msg.command.max_effort=20.
    print('Sending ONE gripper action; no ramp/contact heuristic.',flush=True)
    future=robot.grip.send_goal_async(goal_msg)
    if not robot.spin_until(future.done,5):
        raise RuntimeError('Goal acknowledgment timed out; do not retry until controller state is checked')
    handle=future.result()
    if handle is None or not handle.accepted:raise RuntimeError('Gripper goal rejected')
    result_future=handle.get_result_async()
    if not robot.spin_until(result_future.done,20):
        cancel=handle.cancel_goal_async();robot.spin_until(cancel.done,3)
        raise RuntimeError('Gripper result timed out; cancellation requested; inspect controller before retrying')
    wrapped=result_future.result()
    result=wrapped.result
    recent=[];deadline=time.monotonic()+8;settled=False
    while time.monotonic()<deadline:
        robot.spin_until(lambda:False,.1)
        if robot.grip_pos_msr is None:continue
        now=time.monotonic();recent.append((now,float(robot.grip_pos_msr)))
        recent=[r for r in recent if r[0]>=now-1.]
        if len(recent)>=8 and recent[-1][0]-recent[0][0]>=.8:
            values=np.array([r[1] for r in recent]);measured_gap=float(np.interp(values[-1],table[:,0],table[:,1]))
            if np.ptp(values)<.005 and abs(measured_gap-width)<=.005:
                settled=True;break
    pos=robot.grip_pos_msr
    if pos is None:raise RuntimeError('Gripper feedback missing after action')
    print('Controller status:',wrapped.status,'reached_goal:',result.reached_goal,'stalled:',result.stalled,'reported position:',result.position,flush=True)
    gap=float(np.interp(pos,table[:,0],table[:,1]))
    print('Measured gripper position %.4f; calibrated gap %.1f mm'%(pos,gap*1000),flush=True)
    if wrapped.status!=GoalStatus.STATUS_SUCCEEDED or not result.reached_goal or result.stalled or not settled or abs(gap-width)>.005:raise RuntimeError('Opening did not settle near requested gap; inspect before descending')
    print('Jaw opening set. No arm movement or grasp closure commanded.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
