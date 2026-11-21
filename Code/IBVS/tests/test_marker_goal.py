"""Marker tracking, taught goals and config loading. Synthetic images: logic checks, not hardware evidence."""
from pathlib import Path
import sys
import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
from alignment_test import load_inputs  # noqa: E402
from ibvs.markers import Tracker, mean_marker_depth, median_corners, save_goal  # noqa: E402
from ibvs.overlay import FeatureOverlay  # noqa: E402

K = [[600, 0, 424], [0, 600, 240], [0, 0, 1]]
CAMERA = dict(K=K, distortion=[0] * 5)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
GOAL = [[300, 150], [500, 140], [510, 330], [310, 340]]          # a trapezoid, not a perfect square


def tracker(marker_id=10, **extra):
    cfg = dict(aruco_dict="DICT_4X4_50", ids=[marker_id], target_pixels={marker_id: GOAL}, min_side_px=20, max_jump_px=30)
    return Tracker(CAMERA, {**cfg, **extra})


def canvas(marker_id=10, rotate=0, x=300, y=150, size=160):
    img = np.full((480, 848), 255, np.uint8)
    marker = np.rot90(cv2.aruco.generateImageMarker(DICT, marker_id, size), rotate)
    img[y:y + size, x:x + size] = marker
    return img


def test_single_marker_corners_are_found_with_identity():
    px, valid = tracker().detect(canvas())
    assert px.shape == (1, 4, 2) and valid.tolist() == [True]
    assert np.abs(px[0, 0] - [300, 150]).max() < 2 and np.abs(px[0, 2] - [460, 310]).max() < 2


def test_other_marker_ids_are_ignored_and_reported():
    t = tracker(10)
    px, valid = t.detect(canvas(11))
    assert not valid.any() and np.isnan(px).all() and t.candidates == [11]


def test_corner_order_follows_the_marker_not_the_image():
    t0, t1 = tracker(), tracker()
    a, _ = t0.detect(canvas(rotate=0))
    b, _ = t1.detect(canvas(rotate=1))
    assert np.abs(np.sort(a[0], axis=0) - np.sort(b[0], axis=0)).max() < 2     # same four points
    assert np.abs(a[0] - b[0]).max() > 100                                      # but a different index order


def test_jump_is_rejected():
    t = tracker()
    assert t.detect(canvas())[1][0]
    assert not t.detect(canvas(x=380))[1][0]                                    # 80 px jump > 30


def test_depth_from_marker_interior_and_failure():
    depth = np.zeros((480, 848), np.float32)
    depth[150:310, 300:460] = 0.18
    corners = [[300, 150], [460, 150], [460, 310], [300, 310]]
    Z, info = mean_marker_depth(depth, corners, .07, .5)
    assert np.allclose(Z, .18) and info["valid_samples"] > 1000
    with pytest.raises(ValueError, match="insufficient"):
        mean_marker_depth(np.zeros_like(depth), corners, .07, .5)


def test_median_goal_needs_still_marker():
    base = np.array(GOAL, float)
    frames = [base + np.random.default_rng(i).normal(0, .1, (4, 2)) for i in range(20)]
    assert np.abs(median_corners(frames) - base).max() < .2
    with pytest.raises(ValueError, match="moved or is noisy"):
        median_corners([base + i * 2 for i in range(20)])
    with pytest.raises(ValueError, match="at least 10"):
        median_corners(frames[:5])


def reference_files(tmp_path, audio=True, marker_id=11):
    goals = tmp_path / "goals.yaml"
    if audio:
        save_goal(goals, "audio", marker_id, GOAL, K, .1)
    return ROOT / "config" / "alignment.yaml", ROOT / "config" / "marker_goal.yaml", goals


def test_load_inputs_selects_socket_marker_and_taught_trapezoid(tmp_path):
    settings, reference, goals = reference_files(tmp_path)
    c, ref, _ = load_inputs(settings, reference, "audio", goals)
    assert ref["feature_tracking"]["ids"] == [11] and ref["corners_image_px"] == [list(map(float, p)) for p in GOAL]
    assert ref["socket"] == "audio" and c["tolerance_px"] == 5.0


def test_load_inputs_refuses_missing_goal_unknown_socket_and_wrong_marker(tmp_path):
    settings, reference, goals = reference_files(tmp_path)
    with pytest.raises(ValueError, match="teach_goal.py --socket usb_a"):
        load_inputs(settings, reference, "usb_a", goals)
    with pytest.raises(ValueError, match="must be one of"):
        load_inputs(settings, reference, "hdmi", goals)
    with pytest.raises(ValueError, match="must be one of"):
        load_inputs(settings, reference, None, goals)
    other = tmp_path / "wrong"
    other.mkdir()
    settings, reference, goals = reference_files(other, marker_id=99)
    with pytest.raises(ValueError, match="marker 99"):
        load_inputs(settings, reference, "audio", goals)


def test_save_goal_keeps_other_sockets_and_header(tmp_path):
    goals = tmp_path / "g.yaml"
    goals.write_text("# header kept\n")
    save_goal(goals, "audio", 11, GOAL, K, .1)
    save_goal(goals, "usb_a", 10, GOAL, K, .2)
    data = yaml.safe_load(goals.read_text())
    assert set(data) == {"audio", "usb_a"} and goals.read_text().startswith("# header kept")
    assert data["usb_a"]["marker_id"] == 10 and data["audio"]["camera_K"] == [list(map(float, r)) for r in K]


def test_overlay_tracks_the_selected_marker(tmp_path):
    settings, reference, goals = reference_files(tmp_path)
    _, ref, _ = load_inputs(settings, reference, "audio", goals)
    overlay = FeatureOverlay(ref)
    image = cv2.cvtColor(canvas(11), cv2.COLOR_GRAY2BGR)
    out = overlay.render(image, K, [0] * 5, stamp=1.0, age_s=0.0, draw_target=True)
    assert out.shape == image.shape and overlay.corners is not None and overlay.errors.max() > 5
    assert "usb" not in overlay.label.lower() and overlay.label == "ArUco ID 11"
