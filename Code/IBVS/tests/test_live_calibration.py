"""Regression checks for self-return (gripper surface) depth filtering."""
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ibvs.self_depth import SelfDepthTemplate
from ibvs.depth import observed_voxels

K=np.array([[600.,0.,424.],[0.,600.,240.],[0.,0.,1.]])


def calibrated_self():
    depth=np.full((40,60),.3)
    mask=np.zeros(depth.shape,bool);mask[20:35,5:25]=True
    depth[mask]=.12
    template=SelfDepthTemplate.calibrate(np.repeat(depth[None],5,axis=0),mask,np.eye(3),np.zeros(5),'optical',0.)
    return template,depth


def test_self_filter_retains_new_obstacles_and_background(tmp_path):
    template,depth=calibrated_self()
    path=tmp_path/'self.npz';template.save(path);template=SelfDepthTemplate.load(path)
    depth[22,8]=.09  # closer than the measured finger surface
    depth[23,9]=.20  # a changed return must also remain in the map
    exclude,info=template.exclude(depth,np.eye(3),np.zeros(5),'optical',0.)
    assert info['support_fraction']>.9
    assert not exclude[22,8] and not exclude[23,9]
    assert exclude[25,15] and not exclude[0,0]
    filtered=np.where(exclude,np.nan,depth)
    assert filtered[22,8]==.09 and filtered[23,9]==.20 and filtered[0,0]==.3
    voxels=observed_voxels(filtered,np.eye(3),np.zeros(5),np.eye(4),.05,.5,stride=1)
    assert len(voxels)>0


@pytest.mark.parametrize('changed',['opening','intrinsics','frame','geometry'])
def test_self_filter_refuses_changed_camera_gripper_or_surface(changed):
    template,depth=calibrated_self()
    K2=np.eye(3);frame='optical';opening=0.
    if changed=='opening':opening=.1
    if changed=='intrinsics':K2[0,0]=2.
    if changed=='frame':frame='other_camera'
    if changed=='geometry':depth[template.mask]=.08
    with pytest.raises(ValueError):template.exclude(depth,K2,np.zeros(5),frame,opening)


def test_self_calibration_requires_consistently_measured_surfaces():
    template,depth=calibrated_self()
    frames=np.repeat(depth[None],5,axis=0);frames[0,template.mask]=0.
    with pytest.raises(ValueError):SelfDepthTemplate.calibrate(frames,template.mask,np.eye(3),np.zeros(5),'optical',0.)


def test_color_and_multiple_measured_depth_returns_retain_changed_objects():
    original,depth=calibrated_self()
    colors=np.full((*depth.shape,3),[80,60,30],np.uint8)
    frames=np.repeat(depth[None],6,axis=0)
    for i,value in enumerate([.11,.12,.13,.11,.12,.13]):frames[i,original.mask]=value
    template=SelfDepthTemplate.calibrate(frames,original.mask,np.eye(3),np.zeros(5),'optical',0.,color=colors)
    depth[original.mask]=.13
    depth[22,8]=.09
    changed=colors.copy();changed[23,9]=[0,0,255]
    exclude,details=template.exclude(depth,np.eye(3),np.zeros(5),'optical',0.,color=changed)
    assert exclude[25,15] and not exclude[22,8] and not exclude[23,9]
    assert details['support_fraction']>.9
    with pytest.raises(ValueError,match='color image'):
        template.exclude(depth,np.eye(3),np.zeros(5),'optical',0.)


def test_visible_finger_occludes_farther_depth_but_closer_or_changed_pixels_remain():
    original,depth=calibrated_self()
    colors=np.full((*depth.shape,3),[100,70,30],np.uint8)
    template=SelfDepthTemplate.calibrate(np.repeat(depth[None],5,axis=0),original.mask,
        np.eye(3),np.zeros(5),'optical',0.,color=colors)
    depth[22,8]=.30
    depth[23,9]=.10
    changed=colors.copy();changed[24,10]=[0,0,255]
    exclude,_=template.exclude(depth,np.eye(3),np.zeros(5),'optical',0.,color=changed)
    assert exclude[22,8] and not exclude[23,9] and not exclude[24,10]


def test_three_millimeter_boundary_is_inclusive_despite_float_roundoff():
    template,depth=calibrated_self()
    depth[template.mask]=.123
    exclude,_=template.exclude(depth,np.eye(3),np.zeros(5),'optical',0.)
    assert np.all(exclude[template.mask])


