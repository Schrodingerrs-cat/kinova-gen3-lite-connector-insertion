#!/usr/bin/env python3
"""Joint CAD fit to segmented, registered RGB-D views; offline only."""
import argparse,json
from pathlib import Path
import cv2,numpy as np,trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.ndimage import maximum_filter, minimum_filter
from fit_cad_cloud import register,rotation
from housing_grasp.fit_quality import assess_fit

ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--rois',type=Path,required=True);ap.add_argument('--fit-profile',choices=['strict','supervised_pickup'],default='strict');ap.add_argument('--dark-v',type=int,default=100);ap.add_argument('--marker-s-min',type=int,default=140);ap.add_argument('--hand-eye',type=Path,help='offline candidate.npz T_tool_camera; disables old scan corrections');ap.add_argument('--raw-poses',action='store_true');ap.add_argument('--clean-depth',action='store_true');ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
Xoverride=np.load(a.hand_eye)['T_tool_camera'] if a.hand_eye else None
root=Path(__file__).resolve().parents[3];rois=json.loads(a.rois.read_text());fusion=json.loads((a.scan/'fusion.json').read_text());corrections={k:np.array(v.get('T',np.eye(4))) for k,v in zip(fusion['views'],fusion['per_view'])}
mesh=trimesh.load(root/'CAD_files/USB_A_Plug.glb',force='mesh');P,_=trimesh.sample.sample_surface(mesh,40000,seed=0);P=P[P[:,2]<=.010001]*1000;model=np.c_[P[:,0],-P[:,1],10-P[:,2]]
views=[];rng=np.random.default_rng(0)
for p in sorted(a.scan.glob('view_*.npz')):
 k=int(p.stem[-2:]);f=np.load(p);d,c,K,T=[f[n] for n in ('depth','color','K','T_BC')];x0,y0,x1,y1=rois[p.stem]
 if Xoverride is not None:T=f['T_BT']@Xoverride
 hsv=cv2.cvtColor(c,cv2.COLOR_BGR2HSV);mask=np.zeros(d.shape,bool);mask[y0:y1,x0:x1]=True
 mask &= (hsv[...,2]<a.dark_v)|((hsv[...,0]>=75)&(hsv[...,0]<=110)&(hsv[...,1]>=a.marker_s_min))
 valid=mask&np.isfinite(d)&(d>0)
 if a.clean_depth:
  good=np.isfinite(d)&(d>0);hi=maximum_filter(np.where(good,d,0),3);lo=minimum_filter(np.where(good,d,np.inf),3);valid &= (hi-lo)<.006
 v,u=np.nonzero(valid);z=d[v,u]
 points=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]@T[:3,:3].T+T[:3,3]
 C=np.eye(4) if a.raw_poses or Xoverride is not None else corrections[k];points=(points@C[:3,:3].T+C[:3,3])*1000
 if a.clean_depth:points=points[np.linalg.norm(points-np.median(points,axis=0),axis=1)<40]
 if len(points)<100:raise RuntimeError(f'{p.stem}: only {len(points)} points; check ROI')
 order=rng.permutation(len(points));split=int(.8*len(points));train=points[order[:split]];test=points[order[split:]]
 if len(train)>400:train=train[rng.choice(len(train),400,replace=False)]
 views.append(dict(name=p.stem,points=points,train=train,test=test,color=c,K=K,T=T,C=C,mask=mask))
train=np.vstack([v['train'] for v in views]);solutions=register(train,model,six_dof=True);score,x,_=solutions[0];R=rotation(x[3],*x[4:]);aligned=model@R.T+x[:3];tree=cKDTree(model);metrics=[]
fig,axes=plt.subplots(1,2,figsize=(12,6),layout='constrained')
for v in views:
 d=tree.query((v['test']-x[:3])@R)[0];metrics.append(dict(view=v['name'],points=len(v['points']),median_p95_mm=np.percentile(d,[50,95]).tolist(),within_2mm_percent=float(100*np.mean(d<2))))
 for ax,(i,j) in zip(axes,[(0,1),(0,2)]):ax.scatter(v['points'][:,i],v['points'][:,j],s=3,label=v['name'],alpha=.5)
 # Undo scan registration before projecting onto each original image.
 C,T=v['C'],v['T'];raw=(aligned/1000-C[:3,3])@C[:3,:3];cam=(raw-T[:3,3])@T[:3,:3];K=v['K'];uv=np.rint(cam[:,:2]/cam[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int)
 img=v['color'].copy();img[v['mask']]=(img[v['mask']]*.7+[0,60,0]).clip(0,255).astype('uint8')
 for q in uv[::40]:
  if 0<=q[0]<img.shape[1] and 0<=q[1]<img.shape[0]:cv2.circle(img,tuple(q),1,(255,0,255),-1)
 cv2.imwrite(str(a.out/(v['name']+'_overlay.png')),img)
for ax,(i,j) in zip(axes,[(0,1),(0,2)]):
 ax.scatter(aligned[::5,i],aligned[::5,j],s=2,c='black',alpha=.2,label='Joint CAD fit');ax.set(xlabel='Base X (mm)',ylabel=('Base Y (mm)' if j==1 else 'Base Z (mm)'));ax.set_aspect('equal')
axes[0].legend(loc='upper center',bbox_to_anchor=(.5,1.2),ncol=4,fontsize=8);fig.suptitle(f'{len(views)}-view connector clouds; one joint CAD fit');fig.savefig(a.out/'joint_cloud.png',dpi=150);plt.close(fig)
Tfit=np.eye(4);Tfit[:3,:3]=R;Tfit[:3,3]=x[:3]/1000
near=[s for s in solutions if s[0]<=score*1.2+1e-6];spread=max(abs((np.degrees(s[1][3]-x[3])+90)%180-90) for s in near)
quality=assess_fit(metrics,spread,a.fit_profile)
report=dict(fit_quality=quality,marker_s_min=a.marker_s_min,dark_v=a.dark_v,hand_eye_override=str(a.hand_eye) if a.hand_eye else None,raw_poses=bool(a.raw_poses or a.hand_eye),clean_depth=a.clean_depth,scan=str(a.scan.resolve()),rois=rois,method=('One six-DOF CAD pose fitted to balanced per-view samples using raw camera poses; no tag pose' if a.raw_poses or a.hand_eye else 'One six-DOF CAD pose fitted to balanced per-view samples after recorded ICP corrections; no tag pose'),translation_mm=x[:3].tolist(),rotation_matrix=R.tolist(),per_view=metrics,near_optimal_yaw_spread_deg=float(spread),accepted=quality['accepted'],status='OFFLINE_ONLY_NO_ROBOT_COMMANDS',limitations=['Manual image ROI and colour segmentation','Scan table ICP does not constrain in-plane shifts well','Same-view random pixel holdout is not independent ground truth','Hidden model surfaces not penalized; partial geometry may remain ambiguous'])
(a.out/'fit.json').write_text(json.dumps(report,indent=2));np.savez_compressed(a.out/'fit.npz',T_base_plug=Tfit,aligned_cad_m=aligned/1000,observed_m=np.vstack([v['points'] for v in views])/1000);print(json.dumps(report,indent=2))
