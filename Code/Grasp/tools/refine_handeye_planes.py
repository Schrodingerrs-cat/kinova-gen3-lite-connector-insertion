"""Offline hand-eye candidate from a stationary horizontal board; no live writes."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('--planes',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
data=json.loads(a.planes.read_text());obs=[]
for row in data['planes']:
 f=np.load(a.scan/(row['view']+'.npz'));C=f['T_BC'];B=f['T_BT'];nb=np.array(row['old']['normal']);n=C[:3,:3].T@nb;off=row['old']['offset_m']+nb@C[:3,3];obs.append((B,n,off,row['view']))
X0=np.linalg.inv(np.load(a.scan/'view_00.npz')['T_BT'])@np.load(a.scan/'view_00.npz')['T_BC']
def unpack(x):
 X=np.eye(4);X[:3,3]=x[:3];X[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix();return X
x0=np.r_[X0[:3,3],Rotation.from_matrix(X0[:3,:3]).as_rotvec(),-.02]
def residual(x,records):
 X=unpack(x);r=[]
 for B,n,d,_ in records:
  C=B@X;nb=C[:3,:3]@n;db=d-nb@C[:3,3]
  r.extend(nb[:2]*50);r.append((db+nb[2]*x[6])*1000)
 return r
f=least_squares(residual,x0,args=(obs,),loss='soft_l1',f_scale=2,x_scale='jac',max_nfev=500);X=unpack(f.x)
report=dict(method='Stationary horizontal board plane normals and offsets; no connector points used',X_candidate=X.tolist(),translation_change_mm=((X[:3,3]-X0[:3,3])*1000).tolist(),rotation_change_deg=float(np.degrees(Rotation.from_matrix(X0[:3,:3].T@X[:3,:3]).magnitude())),board_z_m=float(f.x[6]),normal_weight_mm=50,robust_scale_mm=2,status='UNVALIDATED_OFFLINE_CANDIDATE',assumptions=['same plane selected in all seven views','board exactly horizontal in robot base','does not independently validate robot kinematics or depth calibration'])
np.savez(a.out/'candidate.npz',T_tool_camera=X);(a.out/'refinement.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
