"""Synthetic checks of hole identities, selection, overlays, and depth support."""
import copy
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'ros2_iface'))
from ibvs.hole_rectangles import HoleRectangleTracker, hole_rim_depths, number_rectangles  # noqa: E402
from ibvs.overlay import FeatureOverlay  # noqa: E402
from ibvs.rectangles import RectangleTracker, make_rectangle_tracker  # noqa: E402
from alignment_test import load_inputs  # noqa: E402

CAMERA = dict(K=[[600., 0., 424.], [0., 600., 240.], [0., 0., 1.]], distortion=[0.]*5)
GROUPS = np.array([[[80, 160], [200, 160], [200, 300], [80, 300]],
                   [[340, 160], [460, 160], [460, 300], [340, 300]],
                   [[600, 160], [720, 160], [720, 300], [600, 300]]], float)


def reference():
    ref=yaml.safe_load((ROOT/'config/hole_rectangles.yaml').read_text())
    # Synthetic scenes define their own holes, independently of hardware calibration.
    ref['feature_tracking']['seed_rectangles_image_px']=None
    ref['feature_tracking'].pop('template_file',None)
    return ref


def scene(groups=GROUPS, missing=(), polarity='dark', extra=()):
    image = np.full((480, 848), 240 if polarity == 'dark' else 15, np.uint8)
    for index, point in enumerate(np.asarray(groups).reshape(-1, 2)):
        if index not in missing:
            cv2.circle(image, tuple(np.rint(point).astype(int)), 6, 10 if polarity == 'dark' else 245, -1)
    for point in extra:
        cv2.circle(image, tuple(point), 6, 10 if polarity == 'dark' else 245, -1)
    return image


@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('polarity', ['dark', 'bright'])
def test_selected_features_are_four_hole_centers(number, polarity):
    ref = reference()
    config = dict(ref['feature_tracking'], selected_rectangle=number, hole_polarity=polarity)
    tracker = make_rectangle_tracker(CAMERA, config, ref['corners_image_px'])
    pixels, valid = tracker.detect(scene(polarity=polarity))
    assert valid.tolist() == [True]
    np.testing.assert_allclose(pixels[0], GROUPS[number-1], atol=.8)
    assert len(tracker.candidates) == 12  # duplicate intensity thresholds are merged
    assert tracker.star.shape == (1, 4, 2)
    np.testing.assert_allclose(tracker.normalize(pixels[0]),
                               (GROUPS[number-1]-[424, 240])/600, atol=.002)


@pytest.mark.parametrize('number', [1, 2, 3])
def test_missing_selected_hole_never_selects_a_neighboring_rectangle(number):
    ref = reference()
    tracker = HoleRectangleTracker(CAMERA, dict(ref['feature_tracking'], selected_rectangle=number), ref['corners_image_px'])
    assert tracker.detect(scene())[1].all()
    pixels, valid = tracker.detect(scene(missing=[4*(number-1)]))
    assert not valid.any() and np.isnan(pixels).all()
    assert tracker.selected_rectangle == number
    # The same physical hole can be reacquired without renumbering the rectangles.
    recovered, valid = tracker.detect(scene())
    assert valid.all()
    np.testing.assert_allclose(recovered[0], GROUPS[number-1], atol=.8)


def test_identity_survives_motion_and_other_rectangles_leaving_view():
    ref = reference()
    tracker = HoleRectangleTracker(CAMERA, dict(ref['feature_tracking'], selected_rectangle=2), ref['corners_image_px'])
    for shift in range(0, 202, 4):
        groups = GROUPS-[shift, 0]
        pixels, valid = tracker.detect(scene(groups))
        assert valid.all(), (shift, tracker.status)
        np.testing.assert_allclose(pixels[0], groups[1], atol=.8)
    assert not np.isfinite(tracker.observed_rectangles[0]).all()


def test_identity_and_corner_correspondence_survive_gradual_rotation():
    ref = reference()
    tracker = HoleRectangleTracker(CAMERA, dict(ref['feature_tracking'], selected_rectangle=2), ref['corners_image_px'])
    for angle in range(20):
        transform = cv2.getRotationMatrix2D((424, 240), angle, 1.)
        groups = cv2.transform(GROUPS.reshape(-1, 1, 2), transform).reshape(3, 4, 2)
        pixels, valid = tracker.detect(scene(groups))
        assert valid.all(), (angle, tracker.status)
        np.testing.assert_allclose(pixels[0], groups[1], atol=.8)


def test_initial_clutter_is_rejected_and_click_seeds_disambiguate():
    ref = reference()
    config = ref['feature_tracking']
    tracker = HoleRectangleTracker(CAMERA, config, ref['corners_image_px'])
    noisy = scene(extra=[(40, 400)])
    assert not tracker.detect(noisy)[1].any()
    seeded = HoleRectangleTracker(CAMERA, dict(config, seed_rectangles_image_px=GROUPS.tolist()), ref['corners_image_px'])
    pixels, valid = seeded.detect(noisy)
    assert valid.all()
    np.testing.assert_allclose(pixels[0], GROUPS[0], atol=.8)
    assert not tracker.detect(scene(missing=[0]))[1].any()


def test_sheet_outline_does_not_hide_the_holes_from_detection():
    ref = reference()
    tracker = HoleRectangleTracker(CAMERA, ref['feature_tracking'], ref['corners_image_px'])
    image = scene()
    cv2.rectangle(image, (30, 110), (780, 350), 10, 2)
    pixels, valid = tracker.detect(image)
    assert valid.all()
    np.testing.assert_allclose(pixels[0], GROUPS[0], atol=.8)


def test_click_groups_are_numbered_left_to_right_regardless_of_click_order():
    clicked = GROUPS[[2, 0, 1]][:, [3, 1, 0, 2]]
    np.testing.assert_allclose(number_rectangles(clicked), GROUPS)
    duplicate = GROUPS.copy()
    duplicate[2, 0] = duplicate[1, 0]
    with pytest.raises(ValueError, match='distinct'):
        number_rectangles(duplicate)


@pytest.mark.parametrize('number', [0, 4, 1.5, True, '2'])
def test_invalid_rectangle_selection_is_rejected(number):
    ref = reference()
    ref['feature_tracking']['selected_rectangle'] = number
    with pytest.raises(ValueError, match='selected_rectangle'):
        FeatureOverlay(ref)


def test_overlay_resets_convergence_when_selection_changes_or_frame_goes_stale():
    ref = reference()
    ref['target_shape'] = 'rectangle'
    ref['corners_image_px'] = GROUPS[0].tolist()
    display = FeatureOverlay(ref)
    image = cv2.cvtColor(scene(), cv2.COLOR_GRAY2BGR)
    for stamp in range(5):
        drawn = display.render(image, CAMERA['K'], CAMERA['distortion'], stamp, draw_target=True)
    assert display.matched_frames == 5
    assert not np.array_equal(image, drawn)
    display.render(image, CAMERA['K'], CAMERA['distortion'], 4, draw_target=True)
    assert display.matched_frames == 5  # duplicate timestamps do not add evidence
    display.select_rectangle(3)
    assert display.matched_frames == 0
    display.render(image, CAMERA['K'], CAMERA['distortion'], 5, draw_target=True)
    np.testing.assert_allclose(display.corners, GROUPS[2], atol=.8)
    display.render(image, CAMERA['K'], CAMERA['distortion'], 6, age_s=1.)
    assert display.corners is None and display.matched_frames == 0


def test_hole_rim_plane_excludes_background_in_holes_and_rectangle_interior():
    pixels = GROUPS[1]
    depth = np.full((480, 848), .45)
    for point in pixels:
        center = tuple(point.astype(int))
        cv2.circle(depth, center, 20, .2, -1)  # measured sheet surface around each rim
        cv2.circle(depth, center, 6, .45, -1)  # background visible through the hole
    Z, support = hole_rim_depths(depth, pixels, np.full(4, 6.), CAMERA['K'], CAMERA['distortion'],
                                 .07, .5, .008, 30, .6)
    np.testing.assert_allclose(Z, .2, atol=1e-5)
    assert support['inlier_fraction'] > .99
    cv2.circle(depth, tuple(pixels[0].astype(int)), 20, float('nan'), -1)
    with pytest.raises(ValueError, match='rim.*insufficient'):
        hole_rim_depths(depth, pixels, np.full(4, 6.), CAMERA['K'], CAMERA['distortion'], .07, .5, .008, 30, .6)


def test_configuration_override_reaches_controller_and_original_mode_still_loads():
    settings, ref, _ = load_inputs(ROOT/'config/alignment.yaml', ROOT/'config/hole_rectangles.yaml', 3)
    assert ref['feature_tracking']['selected_rectangle'] == 3
    assert settings['tolerance_px'] == 5.
    ref['feature_tracking']['seed_rectangles_image_px']=None
    ref['feature_tracking'].pop('template_file',None)
    tracker = make_rectangle_tracker(CAMERA, ref['feature_tracking'], ref['corners_image_px'])
    np.testing.assert_allclose(tracker.detect(scene())[0][0], GROUPS[2], atol=.8)
    _, old, _ = load_inputs(ROOT/'config/alignment.yaml', ROOT/'config/center_rectangle.yaml')
    assert type(make_rectangle_tracker(CAMERA, old['feature_tracking'], old['corners_image_px'])) is RectangleTracker
    with pytest.raises(ValueError, match='hole_rectangles'):
        load_inputs(ROOT/'config/alignment.yaml', ROOT/'config/center_rectangle.yaml', 2)


def test_click_setup_saves_groups_and_selection_without_ros(tmp_path):
    from hole_overlay import HolePreview

    path = tmp_path/'holes.yaml'
    preview = HolePreview(copy.deepcopy(reference()), path, calibrate=True)
    image = cv2.cvtColor(scene(), cv2.COLOR_GRAY2BGR)
    preview.render(image, CAMERA, 1., 0., True)
    for point in GROUPS[[2, 0, 1]].reshape(12, 2):
        # Clicks slightly off-center must snap to measured blob centers.
        preview.mouse(cv2.EVENT_LBUTTONDOWN, int(point[0])+2, int(point[1])+1, 0, None)
    assert len(preview.clicks) == 12
    preview.save()
    saved = yaml.safe_load(path.read_text())
    np.testing.assert_allclose(saved['feature_tracking']['seed_rectangles_image_px'], GROUPS, atol=.8)
    assert not preview.calibrating
    preview.select(2)
    assert preview.reference['feature_tracking']['selected_rectangle'] == 2
    preview.render(image, CAMERA, 2., 0., True)
    np.testing.assert_allclose(preview.display.corners, GROUPS[1], atol=.8)
    preview.render(image, CAMERA, 3., 1., True)
    assert preview.display.corners is None
