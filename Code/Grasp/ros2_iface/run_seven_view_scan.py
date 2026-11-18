#!/usr/bin/env python3
"""Supervised scan only. --execute moves the arm; never commands a grasp."""
import argparse
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import Trigger
from robot_ros import Robot


def call(node, kind, service, request, seconds=5):
    client=node.create_client(kind,service)
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(service+' unavailable')
        future=client.call_async(request)
        if hasattr(node,'spin_until'):
            node.spin_until(future.done,seconds)
        else:
            rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
        if not future.done():
            raise RuntimeError(service+' timed out. Do not repeat; inspect launcher and robot state.')
        return future.result()
    finally:
        node.destroy_client(client)


def dry(node,value):
    req=SetParameters.Request()
    req.parameters=[Parameter(name='dry_run',value=ParameterValue(type=1,bool_value=value))]
    res=call(node,SetParameters,'/grasp_node/set_parameters',req)
    if len(res.results)!=1 or not res.results[0].successful:
        raise RuntimeError('Could not set dry_run='+str(value))
    print('dry_run:',value,flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    if not args.execute:
        print('No motion. Add --execute for the supervised seven-view scan.');return
    rclpy.init();robot=Robot('seven_view_scan_operator');changed=False
    try:
        if not robot.wait_ready(10):
            raise RuntimeError('Robot/controller feedback unavailable')
        expected=np.array([104,-44,107.3,-109.7,-88.1,-20])
        if not np.isfinite(robot.q).all() or np.max(abs(np.degrees(robot.q)-expected))>2:
            raise RuntimeError('Not at the checked rear starting pose; no scan requested')
        req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp']
        res=call(robot,GetParameters,'/grasp_node/get_parameters',req)
        if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:
            raise RuntimeError('Requires dry_run=True, pregrasp_only=True, allow_grasp=False')
        changed=True;dry(robot,False)
        print('Scanning: arm will move through the seven poses and retrace the route. No grasp.',flush=True)
        res=call(robot,Trigger,'/grasp/scan',Trigger.Request(),600)
        print('success:',res.success,flush=True);print(res.message,flush=True)
        if not res.success:
            raise RuntimeError('Scan failed; inspect launcher before any further command')
    finally:
        if changed:
            try:
                dry(robot,True)
            except Exception as exc:
                print('DRY-RUN RESTORE UNCONFIRMED:',exc,flush=True)
        robot.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
