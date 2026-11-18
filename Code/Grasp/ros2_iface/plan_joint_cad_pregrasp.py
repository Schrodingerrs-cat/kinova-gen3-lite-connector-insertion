#!/usr/bin/env python3
"""Plan-only supervised pregrasp from a diagnostic CAD fit. Never sends motion.
The diagnostic fit has NOT passed full grasp validation. No descent/close/lift.
"""
from pathlib import Path
import sys,json,argparse
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from robot_ros import Robot

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from housing_grasp.config import load_config
from housing_grasp.cad_execution import place_grasps
from housing_grasp.cartesian_sampling import pose_samples

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--fit-dir',type=Path,help='fresh joint-fit directory')
parser.add_argument('--require-fit-pass',action='store_true')
parser.add_argument('--joint6-step-deg',type=float,default=5.,help='joint6 per-waypoint change screen, not a hardware limit or speed')
args=parser.parse_args()
if not np.isfinite(args.joint6_step_deg) or not 0<args.joint6_step_deg<=10:
    parser.error('joint6 step screen must be in (0,10] degrees')
limits=np.array([5.,5.,5.,5.,5.,args.joint6_step_deg])
print('Cartesian path; per-waypoint joint screens deg:',limits,flush=True)
rclpy.init();robot=Robot('joint_cad_pregrasp_plan')
try:
    if not robot.wait_ready(10):raise RuntimeError('Robot not ready')
    old_plan=ROOT/'logs/joint_cad_pregrasp_plan.json'
    old_plan.unlink(missing_ok=True)
    folder=args.fit_dir.resolve() if args.fit_dir else ROOT.parents[1]/'diagnostics/cad_fit/recent_three_1794783802114166114'
    if args.require_fit_pass:
        report=json.loads((folder/'fit.json').read_text())
        if report.get('accepted') is not True:raise RuntimeError('Fresh CAD fit did not pass; no pregrasp plan')
    T=np.load(folder/'joint_fit.npz')['T_base_plug']
    cfg=load_config(ROOT/'config/grasp.yaml');cfg['grasp']['standoff_m']=.100
    candidates=json.loads((ROOT.parents[1]/'CAD_files/USB_A_Plug_grasp_candidates_top200.json').read_text())
    grasps=place_grasps(T,candidates,cfg)
    tcp=np.eye(4);tcp[:3,:3]=Rotation.from_quat(cfg['gripper']['tcp_in_tool_quat_xyzw']).as_matrix();tcp[:3,3]=cfg['gripper']['tcp_in_tool_xyz_m']
    start=robot.tool_matrix();qstart=robot.q.copy();chosen=None
    print('Current joints deg:',np.round(np.degrees(qstart),1),flush=True)
    print('Current tool XYZ:',np.round(start[:3,3],4),flush=True)
    print('Current pose IK:',robot.ik(start,np.degrees(qstart)),flush=True)
    for i,g in enumerate(grasps[:8]):
        target=g['T_pre']@np.linalg.inv(tcp)
        q=robot.ik(target,np.degrees(qstart))
        print('Candidate',i,'flip',g['flip'],'target XYZ',np.round(target[:3,3],4),flush=True)
        if q is None:
            print('  rejected: no target IK',flush=True)
            continue
        print('  target IK deg:',np.round(q,1),flush=True)
        samples=pose_samples(start,target,translation_step_m=.002,rotation_step_deg=1.)
        print('  direct Cartesian samples:',len(samples),'(<=2 mm / 1 deg)',flush=True)
        path=robot.line_path(target,qstart,max_step_m=.002,min_fraction=1-1e-6,waypoints=samples)
        if path is None or len(path)<2:
            print('  rejected: no complete Cartesian path',flush=True)
            continue
        joints=np.vstack([qstart,np.asarray(path)])
        if not np.isfinite(joints).all():
            print('  rejected: nonfinite joint values',flush=True)
            continue
        steps=np.degrees(abs(np.diff(joints,axis=0)))
        peaks=steps.max(axis=0)
        print('  maximum step per joint deg:',np.round(peaks,2),flush=True)
        if np.any(peaks>limits):
            print('  rejected: per-joint waypoint screen exceeded',flush=True)
            continue
        jump=float(peaks.max())
        print('  accepted under requested waypoint screens; not a speed/clearance certification',flush=True)
        chosen=dict(route="direct Cartesian position and orientation interpolation",joint_step_limits_deg=limits.tolist(),maximum_steps_per_joint_deg=peaks.tolist(),rotation_step_deg=1.,explicit_pose_samples=len(samples),max_step_m=.002,max_joint_step_deg=jump,candidate=i,flip=g['flip'],target_tool_matrix=target.tolist(),target_tcp_matrix=g['T_pre'].tolist(),start_tool_matrix=start.tolist(),start_joints_rad=qstart.tolist(),path_rad=np.asarray(path).tolist(),standoff_m=.1,source_fit=str(folder/'joint_fit.npz'),status='PLAN_ONLY_DIAGNOSTIC_FIT_NOT_VALIDATED_FOR_GRASP')
        break
    if chosen is None:raise RuntimeError('No complete Cartesian pregrasp path passed the requested per-joint screens')
    import time
    chosen['planned_at']=time.time()
    out=ROOT/'logs/joint_cad_pregrasp_plan.json';out.parent.mkdir(exist_ok=True);out.write_text(json.dumps(chosen,indent=2))
    print('PLAN ONLY: no motion commanded',flush=True)
    print('Start tool XYZ:',np.round(start[:3,3],4),flush=True)
    print('Pregrasp tool XYZ:',np.round(np.array(chosen['target_tool_matrix'])[:3,3],4),flush=True)
    print('Pregrasp TCP XYZ:',np.round(np.array(chosen['target_tcp_matrix'])[:3,3],4),flush=True)
    print('Standoff: 100 mm | path points:',len(chosen['path_rad']),flush=True)
    print('Saved:',out,flush=True)
    print('This checks kinematics only, not arm/finger/cable collision clearance.',flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
