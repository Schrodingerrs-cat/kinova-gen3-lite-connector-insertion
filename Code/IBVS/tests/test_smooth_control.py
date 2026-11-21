"""Orientation-task derivatives, bounded closed-loop response, and path geometry."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'ros2_iface'))
from ibvs.shape_control import center_size_angle_features,angle_difference,shape_ibvs_step
from ibvs.alignment import straight_joint_path,smooth_servo_path,path_times,trajectory_times
from joint2_priority_servo import smooth_profile
from ibvs.self_depth import SelfDepthTemplate,SelfDepthBank


def test_angle_feature_derivative_matches_finite_difference():
    points=np.array([[-.12,-.08],[.08,-.06],[.11,.09],[-.10,.12]])
    features,jacobian=center_size_angle_features(points)
    for i in range(8):
        shifted=points.copy();shifted.flat[i]+=1e-7
        actual=(center_size_angle_features(shifted)[0]-features)/1e-7
        np.testing.assert_allclose(actual,jacobian[:,i],rtol=1e-4,atol=1e-5)
    assert abs(angle_difference(np.radians(-179),np.radians(179))-np.radians(2))<1e-10


def test_orientation_aware_ibvs_centers_scales_and_rotates_with_bounded_steps():
    square=np.array([[-.025,-.025,0],[.025,-.025,0],[.025,.025,0],[-.025,.025,0]])
    points=square@Rotation.from_euler('z',12,degrees=True).as_matrix().T+[.03,-.02,.30]
    target=square[:,:2]/.25
    initial=abs(center_size_angle_features(points[:,:2]/points[:,2:])[0][3])
    for _ in range(180):
        xy=points[:,:2]/points[:,2:]
        dq=shape_ibvs_step(xy,target,points[:,2],np.eye(6),.35,'center_size_angle',
            np.ones(6),np.full(6,.05),.002,np.radians(.4))
        assert np.linalg.norm(dq[:3])<=.002+1e-10
        assert np.linalg.norm(dq[3:])<=np.radians(.4)+1e-10
        points=(points-dq[:3])@Rotation.from_rotvec(dq[3:]).as_matrix()
    final=center_size_angle_features(points[:,:2]/points[:,2:])[0]
    assert abs(final[3])<initial*.01
    np.testing.assert_allclose(points[:,:2]/points[:,2:],target,atol=1e-4)


def test_smooth_path_preserves_line_and_curved_paths_keep_their_geometry():
    start=np.zeros(6);end=np.radians([.3,-.8,.4,.1,-.3,.6])
    line=np.linspace(start,end,8)
    dense=smooth_servo_path(line)
    assert straight_joint_path(dense)
    np.testing.assert_allclose(dense[[0,-1]],line[[0,-1]])
    assert np.max(np.degrees(np.abs(np.diff(dense,axis=0))))<=.25+1e-10
    curved=line.copy();curved[3,0]+=.02
    assert not straight_joint_path(curved)
    np.testing.assert_array_equal(smooth_servo_path(curved),curved)
    reversal=line[[0,4,2,7]]
    assert not straight_joint_path(reversal)
    times=path_times(dense[[0,-1]],8.)
    assert times[-1]-.1>=1.875*np.degrees(np.max(np.abs(end-start)))/8.-1e-10


@pytest.mark.parametrize('hz',[0.,-1.,float('nan'),10.])
def test_invalid_step_frequency_is_rejected(hz):
    with pytest.raises(ValueError):smooth_profile({},hz)


def test_motion_duration_is_not_shortened_by_start_time_offset():
    path=np.array([[0.]*6,np.radians([.2,-.3,.1,.4,.1,-.2])])
    times=trajectory_times(path,8.,min_duration=.35,start_delay=.1)
    assert times[0]==pytest.approx(.1)
    assert times[-1]-times[0]>=.35-1e-12
    peak=1.875*np.max(np.degrees(np.abs(path[-1]-path[0])))/(times[-1]-times[0])
    assert peak<=8.+1e-10


def test_measured_appearance_bank_selects_matching_reference_and_keeps_foreground(tmp_path):
    depth=np.full((40,60),.3);mask=np.zeros_like(depth,dtype=bool);mask[20:35,5:25]=True;depth[mask]=.12
    paths=[]
    for name,color in [('dim',[60,45,20]),('bright',[200,185,160])]:
        image=np.full((*depth.shape,3),color,np.uint8)
        template=SelfDepthTemplate.calibrate(np.repeat(depth[None],5,axis=0),mask,np.eye(3),np.zeros(5),'optical',0.,color=image)
        path=tmp_path/(name+'.npz');template.save(path);paths.append(path)
    bank=SelfDepthBank(paths);depth[22,8]=.09
    excluded,details=bank.exclude(depth,np.eye(3),np.zeros(5),'optical',0.,color=image)
    assert details['source_template']=='bright.npz'
    assert not excluded[22,8] and excluded[25,15]
