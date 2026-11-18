#!/usr/bin/env python3
"""Fresh target detection: strict colour/depth first, then cross-view image search.
No tag pose or previous grasp/fit is used. Detection does not authorize motion.
"""
import argparse,json
from pathlib import Path
import cv2,numpy as np,yaml
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--radius-mm',type=float,default=18.);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/grasp.yaml').read_text());g=cfg['color_marker'];bounds=cfg['workspace']['region'];views=[];centers=[]
for file in sorted(a.scan.glob('view_*.npz')):
 f=np.load(file);d,c,K,T=[f[k] for k in ('depth','color','K','T_BC')];hsv=cv2.cvtColor(c,cv2.COLOR_BGR2HSV);h=hsv[...,0];h0,h1=g['h'];hue=(h>=h0)&(h<=h1) if h0<=h1 else (h>=h0)|(h<=h1)
 strict=hue&(hsv[...,1]>=g['s_min'])&(hsv[...,2]>=g['v_min'])&np.isfinite(d)&(d>0);v,u=np.nonzero(strict);z=d[v,u];P=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]@T[:3,:3].T+T[:3,3]
 good=(P[:,0]>=bounds[0])&(P[:,0]<=bounds[1])&(P[:,1]>=bounds[2])&(P[:,1]<=bounds[3])&(P[:,2]>cfg['scan']['table_z_m']+.003)&(P[:,2]<cfg['scan']['table_z_m']+.12)
 limited=np.zeros(d.shape,np.uint8);limited[v[good],u[good]]=1;limited=cv2.morphologyEx(limited,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));n,lab,stats,_=cv2.connectedComponentsWithStats(limited);choices=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>=15];center=None
 if choices:
  i=max(choices,key=lambda j:stats[j,cv2.CC_STAT_AREA]);vv,uu=np.nonzero((lab==i)&(d>0));zz=d[vv,uu];center=np.median(np.c_[(uu-K[0,2])*zz/K[0,0],(vv-K[1,2])*zz/K[1,1],zz]@T[:3,:3].T+T[:3,3],axis=0);centers.append(center)
 views.append(dict(name=file.stem,depth=d,color=c,K=K,T=T,hsv=hsv,hue=hue,center=center))
if len(centers)<3:raise RuntimeError('Fewer than3 supported strict detections; no reliable fresh cross-view search seed')
coarse=np.median(centers,axis=0);rois={};audit=[]
for v in views:
 K,T,c,hsv=v['K'],v['T'],v['color'],v['hsv'];center=v['center'];mode='strict colour and depth/workspace'
 camera=np.linalg.inv(T)@np.r_[coarse if center is None else center,1]
 if camera[2]<=0:raise RuntimeError(v['name']+': search point behind camera')
 cx=float(K[0,0]*camera[0]/camera[2]+K[0,2]);cy=float(K[1,1]*camera[1]/camera[2]+K[1,2])
 if center is None:
  mode='fresh cross-view seed plus local dark-body/low-saturation colour support'
  radius=int(np.ceil(.05*K[0,0]/camera[2]));region=np.zeros(c.shape[:2],bool);region[max(0,int(cy)-radius):min(c.shape[0],int(cy)+radius+1),max(0,int(cx)-radius):min(c.shape[1],int(cx)+radius+1)]=True
  dark=(hsv[...,2]<100)&region;dark=cv2.morphologyEx(dark.astype('uint8'),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));n,lab,stats,_=cv2.connectedComponentsWithStats(dark)
  blue=v['hue']&(hsv[...,1]>=60)&(hsv[...,2]>=g['v_min'])&region;choices=[]
  for i in range(1,n):
   if stats[i,cv2.CC_STAT_AREA]<20:continue
   near=cv2.dilate((lab==i).astype('uint8'),np.ones((5,5),np.uint8))>0;yy,xx=np.nonzero(near&blue)
   if len(xx)<5:continue
   distance=float(np.hypot(np.mean(xx)-cx,np.mean(yy)-cy));choices.append((len(xx)/(1+distance),float(np.median(xx)),float(np.median(yy)),len(xx)))
  if not choices:raise RuntimeError(v['name']+': local marker/body support missing; no target guessed')
  _,cx,cy,support=max(choices)
 radius=max(4,int(round(a.radius_mm/1000*K[0,0]/camera[2])));box=[max(0,int(cx)-radius),max(0,int(cy)-radius),min(c.shape[1],int(cx)+radius+1),min(c.shape[0],int(cy)+radius+1)]
 rois[v['name']]=box;preview=c.copy();cv2.rectangle(preview,tuple(box[:2]),tuple(box[2:]),(0,255,0),2);cv2.imwrite(str(a.out/(v['name']+'_target.png')),preview);audit.append(dict(view=v['name'],mode=mode,roi=box))
(a.out/'rois.json').write_text(json.dumps(rois,indent=2));(a.out/'target_detection.json').write_text(json.dumps(dict(method='Fresh marker/depth search; low-saturation fallback only in projected fresh-consensus neighbourhood. CAD pose is still measured downstream.',coarse_seed_base_m=coarse.tolist(),views=audit),indent=2));print('Fresh target regions found:',len(rois));print(a.out/'rois.json')
