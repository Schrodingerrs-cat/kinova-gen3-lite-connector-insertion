#!/usr/bin/env python3
"""Diagnostic six-view fit; omitted view06 is independently evaluated, not hidden."""
import json,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import maximum_filter,minimum_filter
import cv2
ROOT=Path(__file__).resolve().parents[3];scan=ROOT/'Media/Grasp/work/scans/20261115_191507';original=ROOT/'diagnostics/cad_fit/saved_scan_20261115_191507_1794788499408395737';out=ROOT/'diagnostics/cad_fit'/('drop_view06_'+str(time.time_ns()));subset=out/'six_view_scan';subset.mkdir(parents=True)
rois=json.loads((original/'detection/rois.json').read_text());fusion=json.loads((scan/'fusion.json').read_text());omit=6
for k in fusion['views']:
 if k!=omit:shutil.copy2(scan/f'view_{k:02d}.npz',subset/f'view_{k:02d}.npz')
kept=[i for i,k in enumerate(fusion['views']) if k!=omit];small=dict(views=[fusion['views'][i] for i in kept],per_view=[fusion['per_view'][i] for i in kept],skipped=[omit]);(subset/'fusion.json').write_text(json.dumps(small,indent=2));(out/'rois.json').write_text(json.dumps(rois,indent=2))
print('OFFLINE: fit views00–05; view06 retained as an entire held-out camera view. No robot commands.',flush=True)
subprocess.run([sys.executable,str(Path(__file__).parent/'fit_cad_multiview.py'),str(subset),'--rois',str(out/'rois.json'),'--clean-depth','--out',str(out/'fit')],check=True)
f=np.load(scan/'view_06.npz');d,c,K,T=[f[k] for k in ('depth','color','K','T_BC')];h=cv2.cvtColor(c,cv2.COLOR_BGR2HSV);x0,y0,x1,y1=rois['view_06'];mask=np.zeros(d.shape,bool);mask[y0:y1,x0:x1]=True;good=np.isfinite(d)&(d>0);hi=maximum_filter(np.where(good,d,0),3);lo=minimum_filter(np.where(good,d,np.inf),3);mask &= good&((hi-lo)<.006)&((h[...,2]<100)|((h[...,0]>=75)&(h[...,0]<=110)&(h[...,1]>80)));v,u=np.nonzero(mask);z=d[v,u];P=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]@T[:3,:3].T+T[:3,3];C=np.array(fusion['per_view'][fusion['views'].index(omit)].get('T',np.eye(4)));P=P@C[:3,:3].T+C[:3,3];P=P[np.linalg.norm(P-np.median(P,axis=0),axis=1)<.04]
aligned=np.load(out/'fit/fit.npz')['aligned_cad_m'];distance=cKDTree(aligned).query(P)[0]*1000;fit=json.loads((out/'fit/fit.json').read_text());report=dict(excluded_view='view_06',reason_for_experiment='worst residual in prior seven-view fit; diagnostic selection, not an independent corruption criterion',retained_view_metrics=fit['per_view'],retained_views_pass=fit['accepted'],excluded_view_points=len(P),excluded_view_median_p95_mm=np.percentile(distance,[50,95]).tolist(),excluded_view_within_2mm_percent=float(100*np.mean(distance<2)),status='OFFLINE_ABLATION_ONLY_NO_EXECUTION_AUTHORIZATION')
(out/'comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print('Results:',out,flush=True)
