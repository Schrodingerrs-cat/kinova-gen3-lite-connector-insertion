#!/usr/bin/env python3
"""Fresh seven-view scan -> joint CAD registration -> Cartesian pickup.
Requires a reset fixed setup, fully open empty gripper, and project home.
All failure gates stop the run. --execute commands real motion; no replay fallback.
"""
import argparse,json,re,subprocess,sys,time,shutil
from pathlib import Path
import numpy as np,rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from robot_ros import Robot
from run_seven_view_scan import call,dry
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from housing_grasp.config import load_config
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
if not args.execute:
    print('NO COMMANDS. With --execute: fresh7-view scan -> target detection -> one joint CAD fit -> gated Cartesian pickup.')
    print('Requires reset USB seated, fixed board/base/camera, empty fully open gripper, project home. Failed new fit stops before pickup.')
    raise SystemExit(0)
cfg=load_config(ROOT/'config/grasp.yaml')
if len(cfg['scan']['views_joints_deg'])!=7:raise RuntimeError('Expected7 scan poses')
home=(np.array(cfg['scan']['home_joints_deg'])+180)%360-180
rclpy.init();robot=Robot('fresh_cad_pickup_scan');changed=False
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    if np.max(abs(np.degrees(robot.q)-home))>2:raise RuntimeError('Start at project home; no automatic homing while possibly holding cable')
    if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5) or robot.grip_pos_msr>.05:raise RuntimeError('Requires empty fully open gripper')
    q=GetParameters.Request();q.names=['dry_run','pregrasp_only','allow_grasp'];g=call(robot,GetParameters,'/grasp_node/get_parameters',q)
    if len(g.values)!=3 or any(v.type!=1 for v in g.values) or [v.bool_value for v in g.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False node gates')
    changed=True;dry(robot,False);print('REAL MOTION: fresh seven-view scan; pickup waits for fit checks.',flush=True)
    result=call(robot,Trigger,'/grasp/scan',Trigger.Request(),600)
    print(result.message,flush=True)
    if not result.success:raise RuntimeError('Scan failed; stopping')
    match=re.search(r'scan (\d{8}_\d{6}): 7 views',result.message)
    if not match:raise RuntimeError('Did not receive a complete new7-view scan record')
    scan=ROOT.parents[1]/'Media/Grasp/work/scans'/match[1]
finally:
    if changed:
        try:dry(robot,True)
        except Exception as exc:print('DRY-RUN RESTORE UNCONFIRMED:',exc,flush=True);raise
    robot.destroy_node();rclpy.shutdown()
run=ROOT.parents[1]/'Media/Grasp/work/fresh_pickup_runs'/str(time.time_ns());run.mkdir(parents=True);fit=run/'fit';detection=run/'detection'
metadata=dict(scan=str(scan),started=time.time(),outcome='INCOMPLETE',mode='fresh scan, no saved-fit replay')
def stage(script,*flags):
    with (run/'console.log').open('a') as log:
        proc=subprocess.Popen([sys.executable,str(script),*map(str,flags)],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        try:
            for line in proc.stdout:print(line,end='',flush=True);log.write(line);log.flush()
            code=proc.wait()
        except BaseException:
            proc.send_signal(2);raise
        if code:raise RuntimeError(str(script.name)+' failed; no fallback or automatic retry')
try:
    stage(ROOT/'tools/find_scan_target_rois.py',scan,'--out',detection)
    stage(ROOT/'tools/fit_cad_multiview.py',scan,'--rois',detection/'rois.json','--clean-depth','--out',fit)
    result=json.loads((fit/'fit.json').read_text())
    if result.get('accepted') is not True:
        metadata['outcome']='NEW_FIT_REJECTED_NO_PICKUP'
        raise RuntimeError('New CAD fit rejected. No approach/closure/lift. Inspect '+str(fit))
    shutil.copy2(fit/'fit.npz',fit/'joint_fit.npz')
    stage(ROOT/'ros2_iface/run_pickup_video.py','--execute','--fit-dir',fit)
    metadata['outcome']='COMMAND_SEQUENCE_COMPLETED_PHYSICAL_GRIP_REQUIRES_VIDEO_CONFIRMATION'
finally:
    metadata['finished']=time.time();(run/'run.json').write_text(json.dumps(metadata,indent=2))
