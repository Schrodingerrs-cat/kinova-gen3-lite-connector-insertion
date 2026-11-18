"""Offline geometry and failure-gate checks; no ROS or hardware access."""
import sys,copy,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from housing_grasp.cad_execution import consistent_pose,place_grasps


def records():
    rows=[]
    for x in (0,.03,.06):
        camera=np.eye(4);camera[0,3]=x
        T=np.eye(4);T[:3,3]=[.27,-.02,-.005]
        rows.append(dict(T=T,camera=camera,captured_at=100,fit=dict(fit_gate_pass=True)))
    return rows


def reject(rows,now=110):
    try:
        consistent_pose(rows,now)
    except ValueError:
        return
    raise AssertionError('Invalid fit accepted')


def main():
    r=records();np.testing.assert_allclose(consistent_pose(r,110),r[-1]['T'])
    reject(r[:2]);reject(r,now=230)
    x=records();x[2]['T'][0,3]+=.01;reject(x)
    x=records();x[2]['camera']=x[1]['camera'];reject(x)
    x=records();x[2]['fit']['fit_gate_pass']=False;reject(x)
    x=records();x[2]['T'][:3,:3]=np.diag([-1,-1,1]);consistent_pose(x,110)
    x=records();x[2]['T'][0,0]=float('nan');reject(x)
    cfg=dict(gripper=dict(max_open_m=.085),grasp=dict(open_margin_m=.012,standoff_m=.06,lift_m=.01))
    c=dict(yaw_deg=90,mid_mm=0,s_mm=0,u0_mm=3,gap_mm=7.5,cost=0)
    g=place_grasps(r[-1]['T'],[c],cfg)[0]
    np.testing.assert_allclose(g['T_BG'][:3,3],[.27,-.02,-.002])
    np.testing.assert_allclose(g['T_pre'][:3,3],[.27,-.02,.058])
    np.testing.assert_allclose(g['T_BG'][:3,:3].T@g['T_BG'][:3,:3],np.eye(3),atol=1e-12)
    assert np.linalg.det(g['T_BG'][:3,:3])>.999
    root=Path(__file__).resolve().parents[3]
    real=[]
    for k in (1,2,3):
        folder=root/f'diagnostics/cad_fit/view_comparison/view_{k:02d}_6d'
        f=json.loads((folder/'fit.json').read_text());T=np.load(folder/'fit.npz')['T_base_plug']
        frame=np.load(f['frame']);real.append(dict(T=T,camera=frame['T_BC'],captured_at=100,fit=f))
    reject(real)
    print('Passed geometry, stale/poor/inconsistent-fit, repeated-view and saved-cloud rejection checks')

if __name__=='__main__':
    main()
