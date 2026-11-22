"""The acquire path (image handler -> tracker -> capture) with a fake ROS layer and a synthetic wall image."""
from pathlib import Path
from types import SimpleNamespace
import csv
import sys
import time
import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'ros2_iface'))
import alignment_test as at  # noqa: E402
from ibvs.markers import AutoMarkerTracker, TrackingLost  # noqa: E402
from ibvs.overlay import FeatureOverlay  # noqa: E402

K = [[435., 0, 426.5], [0, 434.4, 238.2], [0, 0, 1]]
CAMERA = dict(K=K, distortion=[0.] * 5)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def wall_image():
    img = np.full((480, 848), 255, np.uint8)
    for i, x in ((10, 340), (11, 436), (12, 532)):
        img[205:250, x:x + 45] = cv2.aruco.generateImageMarker(DICT, i, 45)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def experiment(tmp_path, taught=False, behind=0.):
    c, ref, _ = at.load_inputs(ROOT / 'config' / 'alignment.yaml', ROOT / 'config' / 'marker_goal.yaml', 'audio' if taught else None, None, False)
    img = wall_image()
    e = at.Experiment.__new__(at.Experiment)
    e.skip_checks, e.c, e.reference, e.camera, e.output = True, c, ref, CAMERA, tmp_path
    e.is_marker, e.phase, e.detection = True, 'ALIGN', None
    e.tracker, e.display = AutoMarkerTracker(CAMERA, ref['feature_tracking']), FeatureOverlay(ref)
    e.args = SimpleNamespace(show_image=False, live=True)
    e.last_image_stamp, e.after_motion_mono, e.color_arrival, e.last_marker_seen, e.tracking_epoch = None, time.monotonic() - 1, 0., 0., 0
    e.record_start_stamp, e.servo_steps, e.window, e.frames = None, 0, False, 0
    e.last_observation = e.initial_observation = None
    e.record_correction = e.record_sequence = False
    e.video = e.overlay_video = e.correction_video = None
    e.overlay_pub, e.image_type = SimpleNamespace(publish=lambda m: None), lambda: SimpleNamespace()
    e.csv_file = open(tmp_path / 'trial.csv', 'w', newline='')
    e.csv = csv.writer(e.csv_file)
    H, W = img.shape[:2]

    def msg(stamp, kind):
        sec, ns = int(stamp), int((stamp - int(stamp)) * 1e9)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=ns))
        if kind == 'color':
            return SimpleNamespace(header=header, encoding='rgb8', data=np.ascontiguousarray(img[:, :, ::-1]).tobytes(), width=W, height=H, step=W * 3)
        return SimpleNamespace(header=header, encoding='16UC1', data=np.full((H, W), 290, np.uint16).tobytes(), width=W, height=H, step=W * 2)
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(time.time() * 1e9)))
    e.r = SimpleNamespace(_img={}, get_clock=lambda: clock, q=np.zeros(6))

    def pump(seconds=.05):
        now = time.time() - behind                    # behind: the camera clock lags ROS time
        e.r._img['depth'] = msg(now, 'depth')
        e.on_color(msg(now, 'color'))
        time.sleep(.02)
    e.pump = pump
    return e


@pytest.mark.parametrize('taught', [False, True])
def test_capture_locks_a_marker_and_returns_the_error_for_the_goal(tmp_path, taught):
    e = experiment(tmp_path, taught)
    xy, Z, rms, maximum = e.capture()
    assert e.tracker.locked == 11 and np.isfinite(rms) and np.allclose(Z, .29, atol=.001)
    assert (tmp_path / 'initial_observation.npz').exists() and len(open(tmp_path / 'trial.csv').read().splitlines()) == 1


def test_a_camera_clock_far_from_ros_time_does_not_stop_capture(tmp_path):
    e = experiment(tmp_path, behind=3600.)                                  # headers an hour behind
    assert np.isfinite(e.capture()[2])


def test_frames_from_before_the_end_of_the_last_move_are_rejected_with_the_reason(tmp_path):
    e = experiment(tmp_path)
    e.after_motion_mono = time.monotonic() + 100                            # a move that "ends" in the future: every frame is too early
    with pytest.raises(TrackingLost, match=r"frames handled: \d+ \| rejected: \{'frame arrived before the end of the last move': \d+\}"):
        e.capture()


def test_failure_reports_an_error_inside_the_image_handler(tmp_path):
    e = experiment(tmp_path)
    e.tracker.detect = lambda gray: 1 / 0
    with pytest.raises(TrackingLost, match="image handler error: ZeroDivisionError"):
        e.capture()


def test_camera_is_listened_to_under_both_namings():
    assert at.camera_prefixes(dict(camera_topic_prefix="/eih_cam")) == ["/eih_cam", "/camera/camera"]
    assert at.camera_prefixes(dict(camera_topic_prefix="/eih_cam/")) == ["/eih_cam", "/camera/camera"]
    assert at.camera_prefixes(dict(camera_topic_prefix="/camera/camera")) == ["/camera/camera"]


def test_no_images_at_all_is_called_out_with_the_topic_names(tmp_path):
    e = experiment(tmp_path)
    e.camera_prefixes = ["/eih_cam", "/camera/camera"]
    e.pump = lambda seconds=.05: time.sleep(.02)                     # the handler never runs: no images arrive
    e.r._img.clear()
    e.frames_processed = 0
    e.r._img.update(color=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))),
                    depth=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))))
    with pytest.raises(TrackingLost, match=r"NO IMAGES reached the handler: nothing is publishing /eih_cam/color/image_raw or /camera/camera/color/image_raw"):
        e.capture()
