#!/usr/bin/env python3
"""Read joint-state feedback for five seconds. No commands or action goals."""
import time,sys
from pathlib import Path
import numpy as np,yaml
import rclpy
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parents[1]
table=np.array(yaml.safe_load((ROOT/'config/gripper_scan.yaml').read_text())['position_to_width_m'])
rclpy.init();node=rclpy.create_node('check_gripper_settling');samples=[];last_print=[0.]
def receive(msg):
    if 'right_finger_bottom_joint' not in msg.name:return
    i=msg.name.index('right_finger_bottom_joint');p=float(msg.position[i]);gap=float(np.interp(p,table[:,0],table[:,1]))
    now=time.monotonic();samples.append((now,p,gap))
    if now-last_print[0]>=.5:
        print('position %.4f | calibrated gap %.1f mm'%(p,gap*1000),flush=True);last_print[0]=now
sub=node.create_subscription(JointState,'/joint_states',receive,10)
try:
    end=time.monotonic()+5
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    if not samples:raise RuntimeError('No gripper joint-state feedback')
    tail=[s for s in samples if s[0]>=samples[-1][0]-2]
    print('Last2 seconds: position range %.5f rad | gap range %.2f mm'%(np.ptp([s[1] for s in tail]),np.ptp([s[2] for s in tail])*1000),flush=True)
    print('No commands sent.',flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
