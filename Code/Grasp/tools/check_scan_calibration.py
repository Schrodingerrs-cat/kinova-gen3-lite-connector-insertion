#!/usr/bin/env python3
"""Offline tag reprojection bundle adjustment and independent depth-plane checks.
Writes a calibration candidate only; never edits live configuration.
"""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from housing_grasp.color_marker import ransac_plane
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
obj=np.array([[-.05,.05,0],[.05,.05,0],[.05,-.05,0],[-.05,-.05,0]],float)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),cv2.aruco.DetectorParameters())
views=[]
for p in sorted(a.scan.glob('view_*.npz')):
 f=np.load(p);corners,ids,_=det.detectMarkers(f['color'])
 if ids is None or 0 not in ids:
  print(p.stem+': tag not decoded; excluded from calibration',flush=True)
  continue
 uv=corners[list(ids.ravel()).index(0)].reshape(4,2).astype(float)
 uv=cv2.cornerSubPix(cv2.cvtColor(f['color'],cv2.COLOR_BGR2GRAY),uv.astype('float32'),(3,3),(-1,-1),(3,30,.001)).astype(float)
 ok,rv,tv=cv2.solvePnP(obj,uv,f['K'],None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
 if not ok:raise RuntimeError('PnP failed')
 CT=np.eye(4);CT[:3,:3]=cv2.Rodrigues(rv)[0];CT[:3,3]=tv.ravel()
 views.append(dict(name=p.stem,f=f,uv=uv,CT=CT))
if len(views)<4:raise RuntimeError('Need at least four decoded tag views')
X0=np.linalg.inv(views[0]['f']['T_BT'])@views[0]['f']['T_BC'];Y0=views[0]['f']['T_BT']@X0@views[0]['CT']
def pack(T):return np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_rotvec()]
def unpack(x):
 T=np.eye(4);T[:3,3]=x[:3];T[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix();return T
initial=np.r_[pack(X0),pack(Y0)]
def residual(x,selected):
 X,Y=unpack(x[:6]),unpack(x[6:]);out=[]
 for v in selected:
  C=np.linalg.inv(v['f']['T_BT']@X)@Y
  pc=obj@C[:3,:3].T+C[:3,3];K=v['f']['K'];uv=pc[:,:2]/pc[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]
  out.extend((uv-v['uv']).ravel())
 return np.array(out)
fit=least_squares(residual,initial,args=(views,),loss='soft_l1',f_scale=1,max_nfev=500,x_scale='jac');X=unpack(fit.x[:6]);Y=unpack(fit.x[6:])
loo=[]
for i,v in enumerate(views):
 train=[q for k,q in enumerate(views) if k!=i];test=least_squares(residual,initial,args=(train,),loss='soft_l1',f_scale=1,max_nfev=300,x_scale='jac')
 old=least_squares(lambda y:residual(np.r_[initial[:6],y],train),initial[6:],loss='soft_l1',f_scale=1,max_nfev=300)
 loo.append(dict(view=v['name'],old_calibration_held_out_rms_px=float(np.sqrt(np.mean(residual(np.r_[initial[:6],old.x],[v])**2))),candidate_held_out_rms_px=float(np.sqrt(np.mean(residual(test.x,[v])**2)))))
checks=[]
for v in views:
 f=v['f'];d,c,K,B=[f[k] for k in ('depth','color','K','T_BT')];valid=np.isfinite(d)&(d>0);vv,uu=np.nonzero(valid);z=d[vv,uu]
 pc=np.c_[(uu-K[0,2])*z/K[0,0],(vv-K[1,2])*z/K[1,1],z];pt=(pc-v['CT'][:3,3])@v['CT'][:3,:3]
 hsv=cv2.cvtColor(c,cv2.COLOR_BGR2HSV)[vv,uu];board=(pt[:,2]>.025)&(pt[:,2]<.045)&(abs(pt[:,0])<.18)&(abs(pt[:,1])<.2)&(hsv[:,1]<100)&(hsv[:,2]>100)
 row=dict(view=v['name'],saved_X_difference_max=float(np.max(abs(np.linalg.inv(B)@f['T_BC']-X0))),board_points=int(board.sum()))
 if board.sum()>200:
  n,off,count=ransac_plane(pc[board],thresh=.002,iters=500)
  # RANSAC points toward camera? Orient normal toward the tag normal in camera frame.
  if n@v['CT'][:3,2]<0:n,off=-n,-off
  centroid=np.median(pc[board],axis=0);err=abs(pc[board]@n+off)*1000
  for label,Xc in [('before',X0),('candidate',X)]:
   C=B@Xc;nb=C[:3,:3]@n;pb=C[:3,:3]@centroid+C[:3,3]
   row[label]=dict(normal_base=nb.tolist(),plane_offset_m=float(off-nb@C[:3,3]),centroid_base_m=pb.tolist(),tilt_deg=float(np.degrees(np.arccos(np.clip(nb[2],-1,1)))))
  row.update(plane_residual_median_p95_mm=np.percentile(err,[50,95]).tolist(),inlier_fraction=count/board.sum())
 checks.append(row)
report=dict(method='Joint optimization of tool-to-optical-camera and one stationary tag pose from tag image corners; tag side100mm',X_before=X0.tolist(),X_candidate=X.tolist(),translation_change_mm=((X[:3,3]-X0[:3,3])*1000).tolist(),rotation_change_deg=float(np.degrees(Rotation.from_matrix(X0[:3,:3].T@X[:3,:3]).magnitude())),leave_one_view_out=loo,board_checks=checks,status='OFFLINE_CALIBRATION_CANDIDATE_NOT_APPLIED',limitations=['Assumes fixed tag/robot base and correct kinematics','Saved distortion and exact colour-depth timestamps unavailable','Board depth selection uses tag-relative25–45mm band around supplied35mm CAD height; check independent plane residuals','Tag fit is a calibration diagnostic, not a tag-based connector grasp'])
(a.out/'calibration.json').write_text(json.dumps(report,indent=2));np.savez(a.out/'candidate.npz',T_tool_camera=X,T_base_tag=Y)
# Reproject tag corners with old and candidate camera calibration.
rows=[]
for v in views:
 c=v['f']['color'].copy()
 for x,col in [(initial,(0,0,255)),(fit.x,(255,0,255))]:
  C=np.linalg.inv(v['f']['T_BT']@unpack(x[:6]))@unpack(x[6:]);pc=obj@C[:3,:3].T+C[:3,3];K=v['f']['K'];uv=np.rint(pc[:,:2]/pc[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int);cv2.polylines(c,[uv],True,col,2)
 for uv in v['uv']:cv2.circle(c,tuple(np.rint(uv).astype(int)),3,(0,255,0),-1)
 cv2.putText(c,v['name'],(10,25),0,.7,(0,0,255),2);rows.append(cv2.resize(c,(424,240)))
if len(rows)%2:rows.append(np.zeros_like(rows[0]))
cv2.imwrite(str(a.out/'tag_reprojection.png'),np.vstack([np.hstack(rows[i:i+2]) for i in range(0,len(rows),2)]))
print(json.dumps(report,indent=2))
