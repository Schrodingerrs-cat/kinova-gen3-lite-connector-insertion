"""Audit competing CAD poses against colour silhouettes, offline only."""
import argparse,json
from pathlib import Path
import cv2,numpy as np,trimesh
ap=argparse.ArgumentParser();ap.add_argument('fit',type=Path);a=ap.parse_args();d=json.load(open(a.fit/'fit.json'));alternatives=json.load(open(a.fit/'alternatives.json'));root=Path(__file__).resolve().parents[3];mesh=trimesh.load(root/'CAD_files/USB_A_Plug.glb',force='mesh');P,_=trimesh.sample.sample_surface(mesh,100000,seed=0);P=P[P[:,2]<=.010001]*1000;model=np.c_[P[:,0],-P[:,1],10-P[:,2]];views=[]
for v in d['per_view']:
 f=np.load(Path(d['scan'])/(v['view']+'.npz'));c,K,T=[f[k] for k in ('color','K','T_BC')];h=cv2.cvtColor(c,cv2.COLOR_BGR2HSV);x0,y0,x1,y1=d['rois'][v['view']];roi=np.zeros(c.shape[:2],bool);roi[y0:y1,x0:x1]=True;mask=roi&((h[...,2]<d['dark_v'])|((h[...,0]>=75)&(h[...,0]<=110)&(h[...,1]>=d['marker_s_min'])))
 mask=cv2.morphologyEx(mask.astype('uint8'),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)).astype(bool);views.append((c,K,T,roi,mask))
rows=[]
for q in alternatives:
 if q['relative_cost']>1.201:continue
 aligned=(model@np.array(q['rotation_matrix']).T+np.array(q['translation_mm']))/1000;ious=[]
 for c,K,T,roi,mask in views:
  cam=(aligned-T[:3,3])@T[:3,:3];uv=np.rint(cam[:,:2]/cam[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int);inside=(cam[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<c.shape[1])&(uv[:,1]>=0)&(uv[:,1]<c.shape[0]);uv=uv[inside];render=np.zeros(c.shape[:2],np.uint8);render[uv[:,1],uv[:,0]]=1;render=cv2.morphologyEx(render,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)).astype(bool)&roi
  ious.append(float(np.sum(render&mask)/max(1,np.sum(render|mask))))
 rows.append(dict(yaw_deg=q['yaw_deg'],relative_depth_cost=q['relative_cost'],silhouette_iou_per_view=ious,mean_iou=float(np.mean(ious))))
(a.fit/'silhouette_audit.json').write_text(json.dumps(rows,indent=2));print([(round(q['yaw_deg'],1),round(q['mean_iou'],3),[round(x,3) for x in q['silhouette_iou_per_view']]) for q in rows])
