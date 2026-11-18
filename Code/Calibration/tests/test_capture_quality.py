"""Synthetic checks for burst quality; no hardware or ROS."""
import sys
from pathlib import Path
import cv2,numpy as np,pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from capture_quality import square_pose,burst_measurement,corners_refined


def test_projected_square_recovery_and_stationary_burst():
    K=np.array([[435.,0,424],[0,435,240],[0,0,1]])
    R=Rotation.from_euler('xyz',[165,8,12],degrees=True).as_matrix();rv=cv2.Rodrigues(R)[0];t=np.array([.01,-.02,.35]);obj=np.array([[-.05,.05,0],[.05,.05,0],[.05,-.05,0],[-.05,-.05,0]])
    uv=cv2.projectPoints(obj,rv,t,K,np.zeros(5))[0].reshape(4,2)
    T,errors=square_pose(uv,K,np.zeros(5),.1)
    np.testing.assert_allclose(T[:3,3],t,atol=1e-6)
    assert errors[0]<1e-5
    poses=[np.eye(4) for _ in range(8)];corners=np.repeat(uv[None],8,axis=0)
    _,M,q=burst_measurement(corners,poses,K,np.zeros(5),.1)
    np.testing.assert_allclose(M,T,atol=1e-6);assert q['tool_motion_mm']==0
    poses[-1][0,3]=.002
    with pytest.raises(ValueError,match='Arm moved'):burst_measurement(corners,poses,K,np.zeros(5),.1)


def test_unstable_corners_rejected():
    corners=np.tile(np.array([[100,100],[200,100],[200,200],[100,200]],float),(8,1,1));corners[-1]+=10
    with pytest.raises(ValueError,match='corners unstable'):
        burst_measurement(corners,[np.eye(4) for _ in range(8)],np.eye(3),np.zeros(5),.1)


def test_refined_detection_and_wrong_id_rejection():
    img=np.full((400,400),255,np.uint8);img[100:300,100:300]=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,200)
    assert corners_refined(img,'DICT_4X4_50',0).shape==(4,2)
    with pytest.raises(ValueError,match='not found'):corners_refined(img,'DICT_4X4_50',1)
