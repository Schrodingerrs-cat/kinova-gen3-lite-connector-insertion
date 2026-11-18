#!/usr/bin/env python3
"""Fresh three-nearby-view RGB-D capture -> joint CAD fit -> Cartesian pickup.
--execute commands real motion. All camera-pose moves follow Cartesian paths.
Taught viewpoints are reused; connector pose and grasp are freshly estimated.
"""
import argparse,json,subprocess,sys,time,shutil
from pathlib import Path
import numpy as np,rclpy
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import GetParameters
from robot_ros import Robot,BASE,TOOL,matrix
from run_seven_view_scan import call
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from housing_grasp.config import load_config
from housing_grasp.cartesian_sampling import pose_samples


def stage(script,flags,folder):
    with (folder/'console.log').open('a') as log:
        proc=subprocess.Popen([sys.executable,str(script),*map(str,flags)],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        try:
            for line in proc.stdout:print(line,end='',flush=True);log.write(line);log.flush()
            code=proc.wait()
        except BaseException:
            proc.send_signal(2);raise
        if code:raise RuntimeError(script.name+' failed; stopped with no automatic retry')


def fresh_frame(robot,X):
    frames=[];stamps=[]
    for _ in range(5):
        color,depth,K=robot.grab()
        msg=robot._img['depth'];colour=robot._img['color'];stamp=msg.header.stamp
        ns=stamp.sec*10**9+stamp.nanosec;cns=colour.header.stamp.sec*10**9+colour.header.stamp.nanosec
        if stamps and ns<=stamps[-1]:raise RuntimeError('Repeated depth frame during fresh capture')
        if not 0<=(robot.get_clock().now().nanoseconds-ns)/1e9<1 or abs(cns-ns)>100000000:raise RuntimeError('Stale/misaligned frame timestamps')
        frames.append(depth);stamps.append(ns)
    when=rclpy.time.Time.from_msg(stamp)
    if not robot.spin_until(lambda:robot.tfbuf.can_transform(BASE,TOOL,when),2):raise RuntimeError('No tool TF at depth image timestamp')
    tr=robot.tfbuf.lookup_transform(BASE,TOOL,when).transform;B=matrix([tr.translation.x,tr.translation.y,tr.translation.z],[tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
    stack=np.stack(frames)
    depth=np.ma.median(np.ma.masked_where((stack<=0)|~np.isfinite(stack),stack),axis=0).filled(0).astype('float32')
    return dict(depth=depth,color=color,K=K,T_BT=B,T_BC=B@X,depth_stamp_ns=np.array(stamps,dtype='int64'),color_stamp_ns=np.array(cns,dtype='int64'))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--fit-profile',choices=['strict','supervised_pickup'],default='strict');ap.add_argument('--execute',action='store_true');ap.add_argument('--plan-scan',action='store_true',help='check camera paths only, never move');ap.add_argument('--sideways',action='store_true',help='use proposed50mm base+Y middle view');args=ap.parse_args()
    if args.execute and args.plan_scan:ap.error('choose --plan-scan or --execute')
    if not args.execute and not args.plan_scan:
        print('NO MOTION. --execute: home ->3 nearby Cartesian camera poses ->home ->fresh CAD fit ->gated Cartesian pickup.')
        print('Requires fixed board/base/mount, USB reseated, empty open gripper, project home. Failed fit stops before pickup.');return
    cfg=load_config(ROOT/'config/grasp.yaml');cam=cfg['camera'];X=np.eye(4);X[:3,:3]=Rotation.from_quat(cam['hand_eye_quat_xyzw']).as_matrix();X[:3,3]=cam['hand_eye_xyz_m']
    pose_file=ROOT/'config'/('nearby_camera_poses_sideways.json' if args.sideways else 'nearby_camera_poses.json')
    taught=json.loads(pose_file.read_text())
    print('Camera pose file:',pose_file,flush=True)
    if not np.allclose(X,np.array(taught['T_tool_camera_at_teaching']),atol=1e-7):raise RuntimeError('Hand-eye changed since camera poses were taught; revalidate views first')
    targets=[np.array(p['T_base_tool']) for p in taught['poses']]
    home=(np.array(cfg['scan']['home_joints_deg'])+180)%360-180
    run=ROOT.parents[1]/'Media/Grasp/work/fresh_nearby_runs'/str(time.time_ns());run.mkdir(parents=True);scan=run/'scan';scan.mkdir()
    metadata=dict(fit_profile=args.fit_profile,started=time.time(),mode='fresh three-view perception and Cartesian pickup',outcome='INCOMPLETE',source_camera_poses=taught)
    rclpy.init();robot=Robot('fresh_nearby_cartesian_scan')
    try:
        if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
        if np.max(abs(np.degrees(robot.q)-home))>2:raise RuntimeError('Must start at project home; no automatic home from an unknown/holding pose')
        if not robot.spin_until(lambda:robot.grip_pos_msr is not None,5) or robot.grip_pos_msr>.05:raise RuntimeError('Requires empty fully open gripper')
        query=GetParameters.Request();query.names=['dry_run','pregrasp_only','allow_grasp'];g=call(robot,GetParameters,'/grasp_node/get_parameters',query)
        if len(g.values)!=3 or any(v.type!=1 for v in g.values) or [v.bool_value for v in g.values]!=[True,True,False]:raise RuntimeError('Requires True/True/False node gates')
        start=robot.tool_matrix();qstart=robot.q.copy();seed=qstart.copy();leg_start=start.copy();plans=[]
        # Validate every outbound and return Cartesian leg before any scan movement.
        for i,target in enumerate(targets+[start]):
            tcp=(target@matrix(cfg['gripper']['tcp_in_tool_xyz_m'],cfg['gripper']['tcp_in_tool_quat_xyzw']))[:3,3]
            if tcp[2]<.10:raise RuntimeError('Camera pose fingertip below100mm base-Z floor')
            samples=pose_samples(leg_start,target,translation_step_m=.002,rotation_step_deg=1.)
            path=robot.line_path(target,seed,max_step_m=.002,min_fraction=1-1e-6,waypoints=samples)
            if path is None or len(path)<2:raise RuntimeError('Camera route leg%d has no complete Cartesian path; nothing moved'%i)
            Q=np.vstack([seed,np.array(path)]);peaks=np.degrees(abs(np.diff(Q,axis=0))).max(axis=0)
            if not np.isfinite(Q).all() or np.any(peaks>[5,5,5,5,5,6]):raise RuntimeError('Camera leg%d joint-step screen rejected: %s; nothing moved'%(i,peaks))
            dt=np.maximum(.25,2*np.max(abs(np.diff(Q,axis=0)),axis=1)/np.deg2rad(6));times=np.cumsum(dt)
            plans.append(dict(target=target,path=path,times=times.tolist(),seed=seed.copy()))
            print('Camera route leg%d: toolXYZ %s, maximum joint steps deg %s, %.1fs'%(i+1,np.round(target[:3,3],4),np.round(peaks,2),times[-1]),flush=True)
            seed=np.array(path[-1]);leg_start=target
        if args.plan_scan:
            metadata['outcome']='CAMERA_ROUTE_PLAN_ONLY_NO_MOTION'
            (run/'run.json').write_text(json.dumps(metadata,indent=2))
            print('PLAN ONLY: all4 Cartesian camera legs passed. No images captured and no motion commanded.',flush=True)
            print('Checks are kinematic; arm/cable/scene clearance still requires supervision.',flush=True)
            return
        # Camera availability checked before moving, and initial joints must not drift.
        robot.grab()
        if np.max(abs(robot.q-qstart))>np.deg2rad(.5):raise RuntimeError('Starting joints changed during planning')
        print('REAL MOTION: three nearby Cartesian camera views, then Cartesian return home. Gripper unchanged.',flush=True)
        for i,plan in enumerate(plans):
            if np.max(abs(robot.q-plan['seed']))>np.deg2rad(1):raise RuntimeError('Starting state differs before camera leg%d'%i)
            print('Camera leg%d /4, %.1f seconds'%(i+1,plan['times'][-1]),flush=True)
            robot.send_joint_path(plan['path'],plan['times'],velocities=np.zeros_like(plan['path']).tolist(),timeout=plan['times'][-1]+30)
            measured=robot.tool_matrix();target=plan['target']
            if np.linalg.norm(measured[:3,3]-target[:3,3])>.003 or Rotation.from_matrix(target[:3,:3].T@measured[:3,:3]).magnitude()>np.deg2rad(1):raise RuntimeError('Camera route endpoint mismatch')
            if i<3:
                robot.spin_until(lambda:False,1.)
                before=robot.q.copy();frame=fresh_frame(robot,X)
                if np.max(abs(robot.q-before))>np.deg2rad(.2):raise RuntimeError('Arm moved during image capture')
                np.savez_compressed(scan/f'view_{i:02d}.npz',**frame);print('Fresh view%d /3 saved'% (i+1),flush=True)
        (scan/'fusion.json').write_text(json.dumps(dict(views=[0,1,2],skipped=[],per_view=[dict(view=i,T=np.eye(4).tolist()) for i in range(3)],method='nearby raw calibrated camera poses; no table ICP'),indent=2))
        metadata['scan_completed']=True
    except BaseException:
        metadata['outcome']='SCAN_STOPPED';(run/'run.json').write_text(json.dumps(metadata,indent=2));raise
    finally:
        robot.destroy_node();rclpy.shutdown()
    try:
        detection=run/'detection';fit=run/'fit'
        stage(ROOT/'tools/find_scan_target_rois.py',[scan,'--radius-mm','15','--out',detection],run)
        stage(ROOT/'tools/fit_cad_multiview.py',[scan,'--rois',detection/'rois.json','--dark-v','90','--fit-profile',args.fit_profile,'--clean-depth','--raw-poses','--out',fit],run)
        result=json.loads((fit/'fit.json').read_text())
        if result.get('accepted') is not True:
            metadata['outcome']='NEW_FIT_REJECTED_NO_PICKUP'
            raise RuntimeError('Fresh CAD fit rejected; no pickup. Inspect '+str(fit))
        shutil.copy2(fit/'fit.npz',fit/'joint_fit.npz')
        stage(ROOT/'ros2_iface/run_pickup_video.py',['--execute','--fit-dir',fit],run)
        metadata['outcome']='SEQUENCE_COMPLETED_PHYSICAL_HOLD_REQUIRES_VIDEO_CONFIRMATION'
    finally:
        metadata['finished']=time.time();(run/'run.json').write_text(json.dumps(metadata,indent=2))
    print('Finished. Holding endpoint; no release or extra home move.',flush=True)

if __name__=='__main__':main()
