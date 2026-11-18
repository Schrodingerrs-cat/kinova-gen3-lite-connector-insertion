#!/usr/bin/env python3
"""Request a fresh CAD view only. Never calls motion services."""
import rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from check_scan_dry_run import request

rclpy.init()
node=rclpy.create_node('capture_cad_view_operator')
try:
    req=GetParameters.Request();req.names=['dry_run','pregrasp_only','allow_grasp']
    res=request(node,GetParameters,'/grasp_node/get_parameters',req,5)
    if len(res.values)!=3 or any(v.type!=1 for v in res.values) or [v.bool_value for v in res.values]!=[True,True,False]:
        raise RuntimeError('Requires dry_run=True, pregrasp_only=True, allow_grasp=False')
    print('Capturing and fitting a fresh view. No motion.',flush=True)
    res=request(node,Trigger,'/grasp/cad_capture',Trigger.Request(),120)
    print('success:',res.success,flush=True);print(res.message,flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
