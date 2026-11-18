"""Compare approximately horizontal depth planes across saved views, offline."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--candidate',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);X=np.load(a.candidate)['T_tool_camera'];rng=np.random.default_rng(0);rows=[]
for file in sorted(a.scan.glob('view*.npz')):
 f=np.load(file);d,c,K,C,B=[f[k] for k in ('depth','color','K','T_BC','T_BT')];h=cv2.cvtColor(c,cv2.COLOR_BGR2HSV);v,u=np.nonzero((d>0)&np.isfinite(d)&(h[...,1]<100)&(h[...,2]>100));z=d[v,u];pc=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z];base=pc@C[:3,:3].T+C[:3,3]
 mask=(base[:,0]>.18)&(base[:,0]<.35)&(base[:,1]>-.08)&(base[:,1]<.18)&(base[:,2]>-.04)&(base[:,2]<.005);pc=pc[mask];P=base[mask]
 if len(P)>12000:
  idx=rng.choice(len(P),12000,replace=False);pc,P=pc[idx],P[idx]
 best=None;bestcount=0
 for _ in range(800):
  q=P[rng.choice(len(P),3,replace=False)];n=np.cross(q[1]-q[0],q[2]-q[0]);length=np.linalg.norm(n)
  if length<1e-10:continue
  n/=length
  if n[2]<0:n=-n
  if n[2]<np.cos(np.deg2rad(15)):continue
  err=abs((P-q[0])@n);inside=err<.002
  if inside.sum()>bestcount:best,bestcount=inside,int(inside.sum())
 if best is None:raise RuntimeError(file.stem+': no board plane')
 c0=pc[best].mean(0);n=np.linalg.svd(pc[best]-c0,full_matrices=False)[2][2]
 if (C[:3,:3]@n)[2]<0:n=-n
 off=-n@c0;res=abs(pc@n+off)*1000
 row=dict(view=file.stem,selected_points=len(P),inlier_fraction=bestcount/len(P),inlier_residual_p95_mm=float(np.percentile(abs(pc[best]@n+off)*1000,95)))
 for label,T in [('old',C),('candidate',B@X)]:
  nb=T[:3,:3]@n;db=off-nb@T[:3,3];height=-(db+nb[:2]@np.array([.25,-.02]))/nb[2]
  row[label]=dict(normal=nb.tolist(),offset_m=float(db),height_at_reference_xy_mm=float(height*1000),tilt_deg=float(np.degrees(np.arccos(nb[2]))))
 rows.append(row)
 preview=c.copy();selected=np.zeros(d.shape,bool);selected[v[mask],u[mask]]=True;preview[selected]=(0,255,0);cv2.imwrite(str(a.out/(file.stem+'_selection.png')),preview)
report=dict(reference_xy_m=[.25,-.02],selection='White points in old-base workspace X .18:.35 Y -.08:.18 Z -.04:.005; RANSAC normal within15deg of old base up. Selects a likely board-top plane; inspect masks.',planes=rows,status='DIAGNOSTIC_NOT_GROUND_TRUTH',height_spread_mm={key:float(np.ptp([r[key]['height_at_reference_xy_mm'] for r in rows])) for key in ('old','candidate')})
(a.out/'planes.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
