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








def test_save_goal_keeps_other_sockets_and_header(tmp_path):
    goals = tmp_path / "g.yaml"
    goals.write_text("# header kept\n")
    save_goal(goals, "audio", 11, GOAL, K, .1)
    save_goal(goals, "usb_a", 10, GOAL, K, .2)
    data = yaml.safe_load(goals.read_text())
    assert set(data) == {"audio", "usb_a"} and goals.read_text().startswith("# header kept")
    assert data["usb_a"]["marker_id"] == 10 and data["audio"]["camera_K"] == [list(map(float, r)) for r in K]




# ---- live marker choice: nothing taught ----
from ibvs.markers import AutoMarkerTracker  # noqa: E402
from ibvs.depth import drop_near_field  # noqa: E402

AUTO = dict(aruco_dict="DICT_4X4_50", min_side_px=20, max_jump_px=30, lock_frames=5)


def row(ids=(10, 11, 12), xs=(120, 340, 560), size=100, rotate=None):
    img = np.full((480, 848), 255, np.uint8)
    for i, x in zip(ids, xs):
        m = cv2.aruco.generateImageMarker(DICT, i, size)
        img[190:190 + size, x:x + size] = np.rot90(m, (rotate or {}).get(i, 0))
    return img


def test_picks_marker_nearest_the_image_centre_after_consecutive_frames():
    t = AutoMarkerTracker(CAMERA, AUTO)
    for _ in range(4):
        px, valid = t.detect(row())
        assert not valid.any() and t.locked is None                       # still choosing
    assert t.detect(row())[1].tolist() == [False] and t.locked == 11      # locked on the 5th frame, centre marker
    px, valid = t.detect(row())
    assert valid.tolist() == [True] and np.abs(px[0, 0] - [340, 190]).max() < 2
    assert t.candidates == [10, 11, 12]


def test_goal_is_centred_upright_and_keeps_the_size_at_choice():
    t = AutoMarkerTracker(CAMERA, AUTO)
    for _ in range(5):
        t.detect(row(size=100))
    goal = t.target_px[0]
    assert np.allclose(goal.mean(0), [424, 240], atol=1e-6)               # principal point
    assert np.allclose(np.linalg.norm(goal[1] - goal[0]), 100, atol=2) and abs(goal[1, 1] - goal[0, 1]) < 1e-9
    assert np.allclose(t.star[0], t.normalize(goal), atol=1e-9)


def test_forced_marker_is_chosen_even_when_not_nearest():
    t = AutoMarkerTracker(CAMERA, {**AUTO, "marker_id": 12})
    for _ in range(5):
        t.detect(row())
    assert t.locked == 12


def test_nothing_locks_on_tiny_or_rotated_markers_and_says_why():
    t = AutoMarkerTracker(CAMERA, {**AUTO, "min_side_px": 150})
    for _ in range(6):
        t.detect(row())
    assert t.locked is None and "too small" in t.status
    t = AutoMarkerTracker(CAMERA, AUTO)
    for _ in range(6):
        t.detect(row(rotate={11: 1, 10: 1, 12: 1}))
    assert t.locked is None and "rotated" in t.status


def test_locked_tracker_follows_its_marker_and_rejects_jumps():
    t = AutoMarkerTracker(CAMERA, AUTO)
    for _ in range(5):
        t.detect(row())
    assert t.detect(row())[1][0]
    assert not t.detect(row(xs=(120, 380, 560)))[1][0]                    # chosen marker jumped 40 px > 30
    assert not t.detect(row(ids=(10, 12, 13)))[1][0]                       # chosen marker missing


def test_near_field_is_removed_with_a_halo_and_far_depth_is_kept():
    depth = np.full((100, 100), .30)
    depth[80:, 40:60] = .10                                                # fingers
    out = drop_near_field(depth, .18, grow_px=5)
    assert np.isnan(out[85, 50]) and np.isnan(out[76, 50]) and not np.isnan(out[60, 50]) and out[10, 10] == .30



# ---- config loading and overlay: marker and goal come from the live view ----
def auto_inputs(socket=None, marker_id=None):
    return load_inputs(ROOT / "config" / "alignment.yaml", ROOT / "config" / "marker_goal.yaml", socket, marker_id)


def test_nothing_taught_is_needed_to_load_and_the_goal_is_only_a_placeholder():
    _, ref, _ = auto_inputs()
    assert ref["feature_tracking"]["marker_id"] is None and "camera_K" not in ref
    assert np.allclose(np.mean(ref["corners_image_px"], axis=0), [424, 240])


def test_socket_or_marker_id_only_force_which_marker_is_chosen():
    assert auto_inputs("audio")[1]["feature_tracking"]["marker_id"] == 11
    assert auto_inputs("usb_a")[1]["feature_tracking"]["marker_id"] == 12
    assert auto_inputs(None, 10)[1]["feature_tracking"]["marker_id"] == 10
    with pytest.raises(ValueError, match="must be one of"):
        auto_inputs("hdmi")


def test_overlay_takes_the_live_goal_after_the_marker_is_chosen():
    _, ref, _ = auto_inputs()
    overlay, tracker = FeatureOverlay(ref), AutoMarkerTracker(CAMERA, ref["feature_tracking"])
    for _ in range(5):
        tracker.detect(row())
    overlay.set_goal(tracker.locked, tracker.target_px[0])
    image = cv2.cvtColor(row(), cv2.COLOR_GRAY2BGR)
    px, valid = tracker.detect(row())
    out = overlay.render(image, K, [0] * 5, stamp=1.0, age_s=0.0, draw_target=True, observation=(px, valid))
    assert overlay.label == "ArUco ID 11" and out.shape == image.shape and overlay.errors.max() > 5
    assert np.allclose(overlay.target.mean(0), [424, 240])
