"""Audit burst poses and cross-validate image-based hand-eye fitting. No ROS."""
import argparse,json
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from calib_lib import to_T,hand_eye_residual
ap=argparse.ArgumentParser();ap.add_argument('samples',type=Path);a=ap.parse_args();d=json.loads(a.samples.read_text());folder=a.samples.parent/a.samples.name.removesuffix('_samples.json');cfg=yaml.safe_load((Path(__file__).parent/'calibration.yaml').read_text());old=to_T(cfg['camera']['hand_eye_xyz_m'],cfg['camera']['hand_eye_quat_xyzw']);h=d['aruco']['marker_size_m']/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float);views=[]
for B,M,c in zip(d['T_base_tool'],d['T_cam_marker'],d['captures']):
 r=json.load(open(folder/f"attempt_{c['attempt']:03d}"/'capture.json'));f=r['frames'][r['reference_frame_index']]
 views.append(dict(B=np.array(B),M=np.array(M),uv=np.array(r['quality']['median_corners_px']),K=np.array(f['K']),dist=np.array(f['distortion'])))
def pack(T):return np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_rotvec()]
def unpack(x):return to_T(x[:3],Rotation.from_rotvec(x[3:6]).as_quat())
targets=[v['B']@old@v['M'] for v in views];Y=np.eye(4);Y[:3,3]=np.mean([t[:3,3] for t in targets],axis=0);Y[:3,:3]=Rotation.from_matrix(np.array([t[:3,:3] for t in targets])).mean().as_matrix();initial=np.r_[pack(old),pack(Y)]
def residual(x,indices):
 X,Y=unpack(x[:6]),unpack(x[6:]);res=[]
 for i in indices:
  v=views[i];C=np.linalg.inv(v['B']@X)@Y
  uv=cv2.projectPoints(obj,Rotation.from_matrix(C[:3,:3]).as_rotvec(),C[:3,3],v['K'],v['dist'])[0].reshape(4,2)
  res.extend((uv-v['uv']).ravel())
 return np.array(res)
allidx=list(range(len(views)));fit=least_squares(residual,initial,args=(allidx,),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=500);X=unpack(fit.x[:6]);cv=[]
for fold in range(5):
 test=[i for i in allidx if i%5==fold];train=[i for i in allidx if i not in test]
 new=least_squares(residual,initial,args=(train,),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=300)
 baseline=least_squares(lambda y:residual(np.r_[initial[:6],y],train),initial[6:],loss='soft_l1',f_scale=.5,max_nfev=300)
 cv.append(dict(held_out_samples=[i+1 for i in test],current_rms_px=float(np.sqrt(np.mean(residual(np.r_[initial[:6],baseline.x],test)**2))),image_refined_rms_px=float(np.sqrt(np.mean(residual(new.x,test)**2)))))
report=dict(method='Image reprojection bundle fit of one hand-eye and one fixed target, with five-fold held-out-pose validation',folds=cv,X_candidate=X.tolist(),translation_change_mm=((X[:3,3]-old[:3,3])*1000).tolist(),rotation_change_deg=float(np.degrees(Rotation.from_matrix(old[:3,:3].T@X[:3,:3]).magnitude())),training_residual_current=hand_eye_residual([v['B'] for v in views],[v['M'] for v in views],old),training_residual_refined=hand_eye_residual([v['B'] for v in views],[v['M'] for v in views],X),status='OFFLINE_DIAGNOSTIC_ONLY_NOT_APPLIED',note='Cross-validation from this dataset is not a new physical test; cannot certify absolute accuracy.')
out=folder/'solver_audit.json';out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
