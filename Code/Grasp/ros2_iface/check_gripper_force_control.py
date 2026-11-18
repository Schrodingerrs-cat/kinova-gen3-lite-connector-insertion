#!/usr/bin/env python3
"""Inspect gripper controller/interfaces and loaded hardware defaults. No commands."""
import xml.etree.ElementTree as ET
import rclpy
from controller_manager_msgs.srv import ListControllers,ListHardwareInterfaces
from rcl_interfaces.srv import GetParameters
from check_scan_dry_run import request
rclpy.init();node=rclpy.create_node('inspect_gripper_force_control')
try:
    controllers=request(node,ListControllers,'/controller_manager/list_controllers',ListControllers.Request(),5)
    for c in controllers.controller:
        if 'gripper' in c.name:
            print('Controller:',c.name,'| type:',c.type,'| state:',c.state,flush=True)
            print('Claimed command interfaces:',list(c.claimed_interfaces),flush=True)
    hardware=request(node,ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces',ListHardwareInterfaces.Request(),5)
    for interface in hardware.command_interfaces:
        if 'gripper' in interface.name or 'finger' in interface.name:
            print(interface.name,'| available:',interface.is_available,'| claimed:',interface.is_claimed,flush=True)
    query=GetParameters.Request();query.names=['robot_description']
    res=request(node,GetParameters,'/robot_state_publisher/get_parameters',query,5)
    if res.values and res.values[0].type==4:
        root=ET.fromstring(res.values[0].string_value)
        for p in root.findall('.//ros2_control/hardware/param'):
            if p.get('name') in ('gripper_max_force','gripper_max_velocity','gripper_joint_name'):
                print('Loaded hardware default',p.get('name')+':',p.text,flush=True)
    print('Read-only inspection. No arm or gripper commands sent.',flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
