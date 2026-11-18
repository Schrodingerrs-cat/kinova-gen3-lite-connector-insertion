#!/usr/bin/env python3
"""Explicit supervised project-home move; no grasp or scan."""
import argparse
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from robot_ros import Robot
from run_seven_view_scan import call,dry

ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
if not args.execute:
    print('No motion. --execute requests the project home pose.');raise SystemExit(0)
rclpy.init();robot=Robot('project_home_operator');changed=False
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    if robot.ik(robot.tool_matrix(),np.degrees(robot.q)) is None:
        raise RuntimeError('Current-pose IK failed; no home request sent')
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp']
    res=call(robot,GetParameters,'/grasp_node/get_parameters',req)
    if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:
        raise RuntimeError('Requires dry_run=True, pregrasp_only=True, allow_grasp=False')
    changed=True;dry(robot,False)
    print('Moving to project home. No grasp.',flush=True)
    result=call(robot,Trigger,'/grasp/home',Trigger.Request(),120)
    print('success:',result.success,flush=True);print(result.message,flush=True)
finally:
    if changed:
        try:dry(robot,True)
        except Exception as exc:print('DRY-RUN RESTORE UNCONFIRMED:',exc,flush=True)
    robot.destroy_node();rclpy.shutdown()
