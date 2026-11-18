"""Reproduce a saved joint fit and expose all multistart pose hypotheses. Offline."""
import argparse,json
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
from fit_cad_cloud import register,rotation
ap=argparse.ArgumentParser();ap.add_argument('fit',type=Path);a=ap.parse_args();d=json.loads((a.fit/'fit.json').read_text());f=np.load(a.fit/'fit.npz');P=f['observed_m']*1000;rng=np.random.default_rng(0);train=[];offset=0
for view in d['per_view']:
 n=view['points'];points=P[offset:offset+n];offset+=n;order=rng.permutation(n);q=points[order[:int(.8*n)]]
 if len(q)>400:q=q[rng.choice(len(q),400,replace=False)]
 train.append(q)
root=Path(__file__).resolve().parents[3];mesh=trimesh.load(root/'CAD_files/USB_A_Plug.glb',force='mesh');surface,_=trimesh.sample.sample_surface(mesh,40000,seed=0);surface=surface[surface[:,2]<=.010001]*1000;model=np.c_[surface[:,0],-surface[:,1],10-surface[:,2]]
solutions=register(np.vstack(train),model,True);best=solutions[0][0];records=[]
for score,x,dist in solutions:
 R=rotation(x[3],*x[4:]);q=dict(cost=score,relative_cost=score/best,yaw_deg=float(np.degrees(x[3])%180),tilt_deg=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),translation_mm=x[:3].tolist(),rotation_matrix=R.tolist(),median_p95_train_mm=np.percentile(dist,[50,95]).tolist());records.append(q)
print(json.dumps(records,indent=2));(a.fit/'alternatives.json').write_text(json.dumps(records,indent=2))
