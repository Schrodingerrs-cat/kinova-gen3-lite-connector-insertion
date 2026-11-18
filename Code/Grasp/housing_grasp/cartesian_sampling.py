"""Explicit position/rotation samples for a Cartesian path request."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose_samples(start, goal, translation_step_m=.002, rotation_step_deg=2.):
    start,goal=np.asarray(start,float),np.asarray(goal,float)
    for T in (start,goal):
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
            raise ValueError('Invalid pose')
        if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(T[:3,:3])<.99:
            raise ValueError('Invalid rotation')
    if not np.isfinite([translation_step_m,rotation_step_deg]).all() or min(translation_step_m,rotation_step_deg)<=0:
        raise ValueError('Sampling steps must be positive and finite')
    distance=np.linalg.norm(goal[:3,3]-start[:3,3])
    angle=np.degrees(Rotation.from_matrix(start[:3,:3].T@goal[:3,:3]).magnitude())
    count=max(1,int(np.ceil(distance/translation_step_m)),int(np.ceil(angle/rotation_step_deg)))
    fractions=np.linspace(0,1,count+1)[1:]
    rotations=Slerp([0,1],Rotation.from_matrix(np.stack([start[:3,:3],goal[:3,:3]])))(fractions).as_matrix()
    poses=np.tile(np.eye(4),(count,1,1));poses[:,:3,:3]=rotations
    poses[:,:3,3]=start[:3,3]+fractions[:,None]*(goal[:3,3]-start[:3,3])
    return poses
