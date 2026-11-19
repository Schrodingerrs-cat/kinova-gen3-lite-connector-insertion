"""Regression checks for live-image fits, labelled-hole tracking and self returns."""
from pathlib import Path
import sys
import cv2
import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ibvs.hole_rectangles import HoleRectangleTracker
from ibvs.hole_templates import template_data
from ibvs.self_depth import SelfDepthTemplate
from ibvs.depth import observed_voxels

K=np.array([[600.,0.,424.],[0.,600.,240.],[0.,0.,1.]])
POINTS=np.array([[[80,160],[200,160],[200,300],[80,300]],
                 [[340,160],[460,160],[460,300],[340,300]],
                 [[600,160],[720,160],[720,300],[600,300]]],float)


def image(points=POINTS,missing=()):
    result=np.full((480,848,3),[80,130,175],np.uint8)
    for index,point in enumerate(np.asarray(points).reshape(-1,2)):
        if index in missing:continue
        color=(130,130,130) if index in (0,8) else (10,10,10)
        cv2.circle(result,tuple(np.rint(point).astype(int)),3,color,-1)
    return result


@pytest.mark.parametrize('bad', [((np.nan,np.nan),(np.nan,np.nan),0.),((0.,0.),(np.inf,np.inf),0.)])
def test_nonfinite_opencv_ellipse_is_rejected(monkeypatch,bad):
    ref=yaml.safe_load((ROOT/'config/hole_rectangles.yaml').read_text())
    ref['feature_tracking'].pop('template_file',None)
    ref['feature_tracking']['seed_rectangles_image_px']=None
    original=cv2.fitEllipse
    calls=[0]
    def fitted(contour):
        calls[0]+=1
        return bad if calls[0]==1 else original(contour)
    monkeypatch.setattr(cv2,'fitEllipse',fitted)
    tracker=HoleRectangleTracker(dict(K=K,distortion=[0.]*5),ref['feature_tracking'],ref['corners_image_px'])
    tracker.find_holes(cv2.cvtColor(image(),cv2.COLOR_BGR2GRAY))
    assert calls[0]>1
    assert all(np.isfinite(center).all() and np.isfinite(radius) for _,center,radius in tracker.candidates)


@pytest.mark.parametrize('selected',[1,2,3])
def test_labelled_color_templates_track_pale_and_dark_holes_and_reject_loss(tmp_path,selected):
    path=tmp_path/'holes.npz'
    np.savez_compressed(path,**template_data(image(),POINTS))
    ref=yaml.safe_load((ROOT/'config/hole_rectangles.yaml').read_text())
    config=dict(ref['feature_tracking'],seed_rectangles_image_px=POINTS.tolist(),template_file=str(path),selected_rectangle=selected)
    tracker=HoleRectangleTracker(dict(K=K,distortion=[0.]*5),config,ref['corners_image_px'])
    for shift in range(5):
        pixels,valid=tracker.detect(image(POINTS+[shift,shift]))
        assert valid.all()
        np.testing.assert_allclose(pixels[0],POINTS[selected-1]+[shift,shift],atol=.8)
    pixels,valid=tracker.detect(image(POINTS+[4,4],missing=[4*(selected-1)]))
    assert not valid.any() and np.isnan(pixels).all()
    assert tracker.selected_rectangle==selected


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


def test_hole_edges_survive_perspective_and_reflective_brightness_change(tmp_path):
    reference=image();path=tmp_path/'holes.npz'
    np.savez_compressed(path,**template_data(reference,POINTS))
    reflected=reference.copy()
    for point in POINTS.reshape(-1,2):
        cv2.circle(reflected,tuple(point.astype(int)),3,(240,240,240),-1)
    transform=np.array([[1.12,-.05,-30.],[.04,1.10,-28.],[.00008,.00003,1.]])
    current=cv2.warpPerspective(reflected,transform,(848,480),borderValue=(80,130,175))
    expected=cv2.perspectiveTransform(POINTS.reshape(-1,1,2),transform).reshape(3,4,2)
    ref=yaml.safe_load((ROOT/'config/hole_rectangles.yaml').read_text())
    config=dict(ref['feature_tracking'],seed_rectangles_image_px=expected.tolist(),template_file=str(path),selected_rectangle=2)
    tracker=HoleRectangleTracker(dict(K=K,distortion=[0.]*5),config,ref['corners_image_px'])
    pixels,valid=tracker.detect(current)
    assert valid.all()
    np.testing.assert_allclose(pixels[0],expected[1],atol=1.)
