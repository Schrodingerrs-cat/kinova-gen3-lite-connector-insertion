"""Pure tag measurement and stationary-burst checks, independent of ROS."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def corners_refined(color, dictionary, marker_id):
    params=cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,dictionary)),params)
    corners,ids,_=detector.detectMarkers(color)
    if ids is None or marker_id not in ids.ravel():
        raise ValueError(f'Marker {marker_id} not found; no sample saved')
    return corners[list(ids.ravel()).index(marker_id)].reshape(4,2).astype(float)


def square_pose(corners,K,dist,size):
    h=size/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float)
    _,rvecs,tvecs,_=cv2.solvePnPGeneric(obj,np.asarray(corners,float),K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates=[]
    for rv,tv in zip(rvecs,tvecs):
        R=cv2.Rodrigues(rv)[0];pc=obj@R.T+tv.ravel()
        if np.min(pc[:,2])<=0:continue
        uv=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
        err=float(np.sqrt(np.mean(np.sum((uv-corners)**2,axis=1))))
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=tv.ravel();candidates.append((err,T))
    if not candidates:raise ValueError('No positive-depth tag pose')
    candidates.sort(key=lambda c:c[0]);return candidates[0][1],[c[0] for c in candidates]


def stationary(poses,max_mm=.5,max_deg=.2):
    reference=poses[0]
    mm=max(np.linalg.norm(p[:3,3]-reference[:3,3])*1000 for p in poses)
    deg=max(np.degrees(Rotation.from_matrix(reference[:3,:3].T@p[:3,:3]).magnitude()) for p in poses)
    if mm>max_mm or deg>max_deg:raise ValueError(f'Arm moved during capture ({mm:.2f} mm, {deg:.2f} deg); stop before capturing')
    return dict(tool_motion_mm=float(mm),tool_motion_deg=float(deg))


def burst_measurement(corners,poses,K,dist,size):
    motion=stationary(poses)
    corners=np.asarray(corners);spread=float(np.max(np.std(corners,axis=0)))
    if spread>.7:raise ValueError(f'Tag corners unstable ({spread:.2f} px); no sample saved')
    uv=np.median(corners,axis=0);T,errors=square_pose(uv,K,dist,size)
    if errors[0]>1:raise ValueError(f'Tag reprojection error {errors[0]:.2f} px > 1 px')
    return poses[len(poses)//2],T,dict(**motion,corner_std_max_px=spread,reprojection_rms_px=errors[0],pnp_candidate_errors_px=errors,median_corners_px=uv.tolist())
