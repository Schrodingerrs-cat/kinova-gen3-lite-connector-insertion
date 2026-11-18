"""Pure geometry gates and grasp placement for live CAD registration."""
import numpy as np
from scipy.spatial.transform import Rotation


def consistent_pose(records, now, max_age=120, position_tol=.003, angle_tol_deg=5):
    if len(records) < 3:
        raise ValueError('Need three live CAD fits from distinct camera views')
    records = records[-3:]
    if any(not 0 <= now-r['captured_at'] <= max_age for r in records):
        raise ValueError('CAD captures expired; collect three fresh views')
    for r in records:
        if not r['fit']['fit_gate_pass']:
            raise ValueError('CAD surface residual gate failed')
        T = np.asarray(r['T'],float)
        if T.shape != (4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
            raise ValueError('Invalid CAD transform')
        if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(T[:3,:3])<.99:
            raise ValueError('Invalid CAD rotation')
        if T[2,2] < np.cos(np.deg2rad(20)):
            raise ValueError('Connector tilt exceeds 20 degrees')
    for i,a in enumerate(records):
        for b in records[i+1:]:
            A,B=np.asarray(a['T']),np.asarray(b['T'])
            if np.linalg.norm(A[:3,3]-B[:3,3])>position_tol:
                raise ValueError('CAD positions disagree by more than 3 mm')
            # The rectangular housing permits a half-turn about its long axis.
            angles=[Rotation.from_matrix(A[:3,:3].T@B[:3,:3]@S).magnitude()
                    for S in (np.eye(3),np.diag([-1,-1,1]))]
            if min(angles)>np.deg2rad(angle_tol_deg):
                raise ValueError('CAD orientations disagree by more than 5 degrees')
            C,D=np.asarray(a['camera']),np.asarray(b['camera'])
            baseline=np.linalg.norm(C[:3,3]-D[:3,3])
            turn=Rotation.from_matrix(C[:3,:3].T@D[:3,:3]).magnitude()
            if baseline<.02 and turn<np.deg2rad(10):
                raise ValueError('Repeat camera pose; use distinct views (20 mm or 10 deg apart)')
    return np.asarray(records[-1]['T'])


def place_grasps(T, candidates, cfg):
    gr,gc=cfg['gripper'],cfg['grasp'];out=[]
    for c in candidates[:40]:
        yaw=np.deg2rad(c['yaw_deg']);b=np.array([np.cos(yaw),np.sin(yaw),0]);a=np.array([-np.sin(yaw),np.cos(yaw),0])
        p=(b*c['mid_mm']+a*c['s_mm']+np.array([0,0,c['u0_mm']]))*.001
        width=c['gap_mm']*.001;opening=width+2*gc['open_margin_m']
        if not 0<opening<=gr['max_open_m']:
            continue
        for flip in (1,-1):
            y=T[:3,:3]@b*flip;z=-T[:3,2]
            G=np.eye(4);G[:3,:3]=np.column_stack([np.cross(y,z),y,z]);G[:3,3]=T[:3,:3]@p+T[:3,3]
            pre,lift=G.copy(),G.copy();pre[:3,3]-=z*gc['standoff_m'];lift[:3,3]-=z*gc['lift_m']
            out.append(dict(T_BG=G,T_pre=pre,T_lift=lift,open_width_m=opening,
                            expected_closed_width_m=width,zone_offset_m=abs(c['s_mm'])*.001,flip=flip,cost=c['cost']))
    if not out:
        raise ValueError('No CAD grasp fits gripper opening')
    return out
