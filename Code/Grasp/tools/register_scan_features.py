#!/usr/bin/env python3
"""Offline RGB-D feature registration, excluding connector regions.
Produces a separate scan copy only if all views connect. Never commands robot.
"""
import argparse,json,shutil
from pathlib import Path
import cv2,numpy as np
from scipy.ndimage import maximum_filter,minimum_filter
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--rois',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--reference',type=int,default=2);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
rois=json.loads(a.rois.read_text());views=[];orb=cv2.ORB_create(nfeatures=5000,fastThreshold=8,edgeThreshold=15);rng=np.random.default_rng(0)
for p in sorted(a.scan.glob('view_*.npz')):
 f=np.load(p);c,d,K,T=[f[k] for k in ('color','depth','K','T_BC')];valid=np.isfinite(d)&(d>0);hi=maximum_filter(np.where(valid,d,0),3);lo=minimum_filter(np.where(valid,d,np.inf),3);valid&=(hi-lo)<.005
 # Feature matching excludes the grasp target and its cable neighbourhood.
 x0,y0,x1,y1=rois[p.stem];valid[max(0,y0-20):min(d.shape[0],y1+20),max(0,x0-20):min(d.shape[1],x1+20)]=False
 keypoints,desc=orb.detectAndCompute(cv2.cvtColor(c,cv2.COLOR_BGR2GRAY),None);keep=[];points=[]
 for i,k in enumerate(keypoints):
  u,v=np.rint(k.pt).astype(int)
  if not valid[v,u]:continue
  z=float(d[v,u]);pc=np.array([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]);base=T[:3,:3]@pc+T[:3,3]
  if not (.12<base[0]<.5 and -.3<base[1]<.3 and -.12<base[2]<.12):continue
  keep.append(i);points.append(pc)
 views.append(dict(file=p,T=T,P=np.array(points),D=desc[keep] if desc is not None else None));print(p.stem,'usable scene features:',len(points),flush=True)

def rigid(src,dst):
 cs,cd=src.mean(0),dst.mean(0);U,_,Vt=np.linalg.svd((src-cs).T@(dst-cd));R=Vt.T@np.diag([1,1,np.linalg.det(Vt.T@U.T)])@U.T;T=np.eye(4);T[:3,:3]=R;T[:3,3]=cd-R@cs;return T

def errors(T,src,dst):return np.linalg.norm(src@T[:3,:3].T+T[:3,3]-dst,axis=1)
edges=[];adj={i:[] for i in range(len(views))};matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
for i in range(len(views)):
 for j in range(i+1,len(views)):
  A,B=views[i],views[j]
  if A['D'] is None or B['D'] is None or min(len(A['D']),len(B['D']))<8:continue
  matches=[m for pair in matcher.knnMatch(A['D'],B['D'],k=2) if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance]
  if len(matches)<10:continue
  src=A['P'][[m.queryIdx for m in matches]];dst=B['P'][[m.trainIdx for m in matches]];order=rng.permutation(len(src));split=max(8,int(.8*len(src)));train=order[:split];test=order[split:];best=None;count=0
  for _ in range(1200):
   ix=rng.choice(train,3,replace=False)
   if np.linalg.norm(np.cross(src[ix[1]]-src[ix[0]],src[ix[2]]-src[ix[0]]))<.00005:continue
   T=rigid(src[ix],dst[ix]);inlier=errors(T,src[train],dst[train])<.003
   if inlier.sum()>count:best,count=inlier,int(inlier.sum())
  if best is None or count<8:continue
  chosen=train[best]
  if np.linalg.norm(np.ptp(src[chosen],axis=0))<.07:continue
  T=rigid(src[chosen],dst[chosen]);held=errors(T,src[test],dst[test]) if len(test) else np.array([]);fraction=float(np.mean(held<.003)) if len(test) else 0
  entry=dict(i=i,j=j,training_matches=len(train),inliers=count,heldout_matches=len(test),heldout_inlier_fraction=fraction,heldout_median_mm=float(np.median(held)*1000) if len(test) else None,T_j_i=T.tolist())
  edges.append(entry)
  if fraction<.5:continue
  weight=count*fraction;adj[i].append((j,np.linalg.inv(T),weight));adj[j].append((i,T,weight))
  print('edge',i,j,'inliers',count,'held-out agreement %.0f%%'%(fraction*100),flush=True)
if not 0<=a.reference<len(views):ap.error('invalid reference')
poses={a.reference:views[a.reference]['T']};tree=[]
while len(poses)<len(views):
 choices=[(weight,i,j,T) for i in poses for j,T,weight in adj[i] if j not in poses]
 if not choices:break
 weight,i,j,T=max(choices,key=lambda q:q[0]);poses[j]=poses[i]@T;tree.append([i,j])
report=dict(method='ORB colour features plus depth rigid registration; target ROI excluded; 80/20 match split',reference_view=a.reference,reference_world_pose='unchanged saved hand-eye-based pose; absolute calibration not independently validated',edges=edges,tree_edges=tree,connected_views=sorted(poses),all_views_connected=len(poses)==len(views),scene_support_degree={i:len(adj[i]) for i in adj},status='OFFLINE_DIAGNOSTIC_ONLY')
(a.out/'feature_registration.json').write_text(json.dumps(report,indent=2))
if len(poses)!=len(views):raise SystemExit('Registration graph disconnected; no registered scan produced. Inspect '+str(a.out/'feature_registration.json'))
scan=a.out/'registered_scan';scan.mkdir(exist_ok=True);per=[]
for i,v in enumerate(views):
 shutil.copy2(v['file'],scan/v['file'].name);C=poses[i]@np.linalg.inv(v['T']);per.append(dict(view=i,T=C.tolist(),rot_deg=float(np.degrees(np.arccos(np.clip((np.trace(C[:3,:3])-1)/2,-1,1)))),shift_mm=float(np.linalg.norm(C[:3,3])*1000)))
(scan/'fusion.json').write_text(json.dumps(dict(views=list(range(len(views))),skipped=[],per_view=per,method='independent scene RGB-D feature registration'),indent=2));print('Saved diagnostic registered scan:',scan,flush=True)
print('This registration still requires independent connector/board overlap validation. No pickup plan created.',flush=True)
