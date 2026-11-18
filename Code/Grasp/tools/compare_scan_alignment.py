#!/usr/bin/env python3
"""Offline image-edge agreement; not a camera calibration estimator."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from scipy.ndimage import distance_transform_edt
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
rows=[];metrics=[]
for p in sorted(a.scan.glob('view_*.npz')):
 f=np.load(p);c=f['color'];d=f['depth'];valid=np.isfinite(d)&(d>0)
 ce=cv2.Canny(cv2.cvtColor(c,cv2.COLOR_BGR2GRAY),50,120)>0
 de=np.zeros(d.shape,bool);de[:,1:]=valid[:,1:]&valid[:,:-1]&(abs(np.diff(d,axis=1))>.006);de[1:]|=valid[1:]&valid[:-1]&(abs(np.diff(d,axis=0))>.006)
 # Image-wide diagnostic: colour texture has no depth counterpart; test depth -> colour.
 dist=distance_transform_edt(~ce);vals=dist[de]
 metrics.append(dict(view=p.stem,depth_edge_pixels=int(de.sum()),median_p95_depth_to_color_edge_px=np.percentile(vals,[50,95]).tolist(),within_2px_percent=float(100*np.mean(vals<=2))))
 out=c.copy();out[ce]=(0,255,0);out[de]=(255,0,255);cv2.putText(out,p.stem,(15,25),0,.7,(0,0,255),2)
 rows.append(cv2.resize(out,(636,360)))
(a.out/'alignment.json').write_text(json.dumps(dict(metrics=metrics,caveats=['6 mm depth discontinuity threshold; Canny 50/120','No raw native depth, distortion, extrinsics, or image timestamps in NPZ','Edge agreement cannot prove correct metric registration','Missing depth is excluded; clutter and flying pixels affect this whole-image statistic']),indent=2))
cv2.imwrite(str(a.out/'edges.png'),np.vstack([np.hstack(rows[i:i+2]) for i in range(0,len(rows),2)]))
print(json.dumps(metrics,indent=2))
