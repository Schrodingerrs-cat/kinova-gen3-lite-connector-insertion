#!/usr/bin/env python3
"""Ask for CAD consensus and a plan only; never calls execute."""
import rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from check_scan_dry_run import request
rclpy.init();node=rclpy.create_node('plan_cad_grasp_operator')
try:
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp']
    res=request(node,GetParameters,'/grasp_node/get_parameters',req,5)
    if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:
        raise RuntimeError('Requires dry_run=True, pregrasp_only=True, allow_grasp=False')
    result=request(node,Trigger,'/grasp/cad_grasp',Trigger.Request(),15)
    print('success:',result.success,flush=True);print(result.message,flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
