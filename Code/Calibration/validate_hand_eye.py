#!/usr/bin/env python3
"""Held-out fixed-target errors against frozen training data. Offline, no ROS."""
import argparse,json
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from calib_lib import to_T

ap=argparse.ArgumentParser();ap.add_argument('samples',type=Path);ap.add_argument('frozen',type=Path);a=ap.parse_args()
all_data=json.loads(a.samples.read_text());training=json.loads((a.frozen/'training_samples.json').read_text());result=json.loads((a.frozen/'training_result.json').read_text());n=len(training['T_base_tool'])
if len(all_data['T_base_tool'])<=n:raise RuntimeError('No held-out samples')
for key in ('T_base_tool','T_cam_marker'):
 if not np.allclose(np.array(all_data[key][:n]),np.array(training[key]),atol=1e-12):raise RuntimeError('Training data changed')
cfg=yaml.safe_load((Path(__file__).parent/'calibration.yaml').read_text())['camera'];old=to_T(cfg['hand_eye_xyz_m'],cfg['hand_eye_quat_xyzw']);new=to_T(result['hand_eye_xyz_m'],result['hand_eye_quat_xyzw'])
report=dict(training_count=n,validation_count=len(all_data['T_base_tool'])-n,method='Predict fixed target pose from training only; evaluate new poses without refitting hand-eye or target pose',calibrations={})
for label,X in [('current',old),('candidate',new)]:
 targets=[np.array(b)@X@np.array(c) for b,c in zip(all_data['T_base_tool'],all_data['T_cam_marker'])]
 center=np.mean([T[:3,3] for T in targets[:n]],axis=0);R=Rotation.from_matrix(np.array([T[:3,:3] for T in targets[:n]])).mean().as_matrix()
 errors=[dict(sample=i+1,position_error_mm=float(np.linalg.norm(T[:3,3]-center)*1000),angle_error_deg=float(np.degrees(Rotation.from_matrix(R.T@T[:3,:3]).magnitude()))) for i,T in enumerate(targets) if i>=n]
 report['calibrations'][label]=dict(training_target_xyz_m=center.tolist(),held_out=errors,position_rmse_mm=float(np.sqrt(np.mean([r['position_error_mm']**2 for r in errors]))),max_position_mm=max(r['position_error_mm'] for r in errors))
(a.frozen/'validation_result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
