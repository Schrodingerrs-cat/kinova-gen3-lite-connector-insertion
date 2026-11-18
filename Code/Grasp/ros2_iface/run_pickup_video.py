#!/usr/bin/env python3
"""Repeat supervised saved-fit pickup from home. No new scan or CAD fitting.
No release, insertion or home motion is performed by this runner.
"""
import argparse,json,os,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np,rclpy
from robot_ros import Robot
from rcl_interfaces.srv import GetParameters
from run_seven_view_scan import call
ROOT=Path(__file__).resolve().parents[1];HERE=Path(__file__).parent
sys.path.insert(0,str(ROOT))
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');ap.add_argument('--fit-dir',type=Path);args=ap.parse_args()
if not args.execute:
    print('No motion. Requires USB reseated, fixed board/base/camera, empty open gripper and robot at project home.')
    print('With --execute: pregrasp -> opening ->80mm ->20mm+close ->5mm lift ->50mm lift; stop holding.')
    raise SystemExit(0)
from housing_grasp.config import load_config
cfg=load_config(ROOT/'config/grasp.yaml');home=(np.array(cfg['scan']['home_joints_deg'])+180)%360-180
rclpy.init();robot=Robot('pickup_video_preflight')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    if np.max(abs(np.degrees(robot.q)-home))>2:raise RuntimeError('Not at project home; runner refuses to home while possibly holding cable')
    if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5) or robot.grip_pos_msr>.05:raise RuntimeError('Requires empty, fully open gripper')
    query=GetParameters.Request();query.names=['dry_run','pregrasp_only','allow_grasp'];gates=call(robot,GetParameters,'/grasp_node/get_parameters',query)
    if len(gates.values)!=3 or any(v.type!=1 for v in gates.values) or [v.bool_value for v in gates.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False node gates')
finally:
    robot.destroy_node();rclpy.shutdown()
run=ROOT.parents[1]/'Media/Grasp/work/video_runs'/str(time.time_ns());run.mkdir(parents=True);archive=run/'prior_logs';archive.mkdir()
# Preserve previous evidence before clearing single-use attempt markers.
for name in ['joint_cad_pregrasp_plan.json','joint_cad_pregrasp_plan.consumed.json','cad_80mm_approach_plan.json','final_cad_grasp_attempt.json','cad_lift_5mm_attempt.json','cad_lift_50mm_attempt.json']:
    p=ROOT/'logs'/name
    if p.exists():shutil.move(str(p),archive/name)
plan_flags=['--joint6-step-deg','6']
if args.fit_dir:plan_flags+=['--fit-dir',str(args.fit_dir.resolve()),'--require-fit-pass']
stages=[('plan_joint_cad_pregrasp.py',plan_flags),('execute_joint_cad_pregrasp.py',['--execute']),('set_cad_pregrasp_opening.py',['--execute']),('approach_cad_80mm.py',['--execute']),('finish_cad_grasp.py',['--execute']),('test_cad_lift_5mm.py',['--execute']),('lift_cad_50mm.py',['--execute'])]
metadata=dict(started=time.time(),mode=('fresh fitted CAD pose supplied by parent perception runner' if args.fit_dir else 'supervised replay of previously fitted CAD pose; no fresh perception'),stages=[],outcome='INCOMPLETE')
print('VIDEO RUN:',run,flush=True);print('REAL MOTION: saved-fit pickup and55mm total lift. Supervise cable/board/grip. No automatic physical holding verification.',flush=True)
try:
    with (run/'console.log').open('w') as log:
        for name,flags in stages:
            print('STAGE:',name,flush=True);proc=subprocess.Popen([sys.executable,str(HERE/name),*flags],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            try:
                for line in proc.stdout:print(line,end='',flush=True);log.write(line);log.flush()
                code=proc.wait()
            except BaseException:
                proc.send_signal(2)
                raise
            metadata['stages'].append(dict(script=name,exit_code=code));(run/'run.json').write_text(json.dumps(metadata,indent=2))
            if code:raise RuntimeError(name+' failed; stopped sequence. Inspect before retrying; no automatic recovery.')
    metadata['outcome']='COMMAND_SEQUENCE_COMPLETED_PHYSICAL_GRIP_NOT_AUTOMATICALLY_VERIFIED'
    print('Finished holding position. Verify USB extracted and board stationary. No release/home commanded.',flush=True)
finally:
    metadata['finished']=time.time();(run/'run.json').write_text(json.dumps(metadata,indent=2))
    for p in (ROOT/'logs').glob('*cad*json'):shutil.copy2(p,run/p.name)
