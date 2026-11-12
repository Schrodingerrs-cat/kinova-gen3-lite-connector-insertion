import pathlib
import sys

import cv2
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from housing_grasp.model_mode import detect_tag_pose, load_panel_map, place_candidates, plug_pose, plug_pose_from_sticker, tag_plane_height  # noqa: E402

K = np.array([[434.7, 0, 424.0], [0, 434.7, 240.0], [0, 0, 1.0]])
T_BC = np.eye(4)
T_BC[:3, :3], T_BC[:3, 3] = np.diag([1.0, -1.0, -1.0]), [0.30, 0.0, 0.20]      # camera 0.2 m above the panel, looking straight down
TAG = dict(dictionary="DICT_4X4_50", id=0, size_m=0.025)


def _render(center_xy, yaw_deg, z=0.0, tag_id=0):
    """White image with the tag at a known pose on the panel plane (corners projected with the camera, tag image warped onto them)."""
    s, y = TAG["size_m"], np.radians(yaw_deg)
    Rz = np.array([[np.cos(y), -np.sin(y)], [np.sin(y), np.cos(y)]])
    corners = [np.r_[center_xy + Rz @ np.array(c) * s / 2, z] for c in ((-1, 1), (1, 1), (1, -1), (-1, -1))]   # TL, TR, BR, BL
    dst = []
    for p in corners:
        c = np.linalg.inv(T_BC) @ np.r_[p, 1]
        dst.append([K[0, 0] * c[0] / c[2] + K[0, 2], K[1, 1] * c[1] / c[2] + K[1, 2]])
    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), tag_id, 600, borderBits=1)
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [600, 0], [600, 600], [0, 600]]), np.float32(dst))
    return cv2.warpPerspective(tag, H, (848, 480), borderValue=255, flags=cv2.INTER_AREA)


def test_tag_pose_is_recovered_within_a_degree_and_a_millimetre():
    img = cv2.cvtColor(_render(np.array([0.31, 0.02]), 30.0), cv2.COLOR_GRAY2BGR)
    t = detect_tag_pose(img, K, T_BC, TAG)
    yaw = np.degrees(np.arctan2(t["T"][1, 0], t["T"][0, 0]))
    assert abs(((yaw - 30.0 + 180) % 360) - 180) < 1.0
    assert np.linalg.norm(t["T"][:2, 3] - [0.31, 0.02]) < 0.001 and t["T"][2, 2] > 0.99 and t["side_px"] > 40   # x, y, yaw from the tag; height comes from the depth (below)
    print("tag height error (mm):", round(t["T"][2, 3] * 1e3, 2))


def test_wrong_tag_id_or_no_tag_raises():
    img = cv2.cvtColor(_render(np.array([0.30, 0.0]), 0.0, tag_id=1), cv2.COLOR_GRAY2BGR)
    with pytest.raises(RuntimeError, match="not seen"):
        detect_tag_pose(img, K, T_BC, TAG)


def test_candidates_are_placed_by_the_plug_pose():
    plug = dict(xy=np.array([0.30, 0.0]), yaw=np.radians(20.0), z=0.0)
    cand = dict(yaw_deg=90.0, s_mm=2.0, mid_mm=0.0, u0_mm=3.0, gap_mm=7.5, cost=0.1)
    q = place_candidates([cand], plug, 0.012, 0.085)[0]
    a_w = np.array([np.cos(np.radians(20)), np.sin(np.radians(20))]) * -2e-3 * -1   # strip axis a = (-1, 0) in the plug frame, rotated by the plug yaw, times s = 2 mm
    assert np.allclose(q["jaw"][:2], [-np.sin(np.radians(20)), np.cos(np.radians(20))], atol=1e-9)   # jaw across the plug's thin direction (plug y), rotated by the yaw
    assert np.allclose(q["p"][:2], np.array([0.30, 0.0]) + np.array([-np.cos(np.radians(20)), -np.sin(np.radians(20))]) * 2e-3, atol=1e-9) and abs(q["p"][2] - 0.003) < 1e-12
    assert abs(q["open_width_m"] - (0.0075 + 0.024)) < 1e-12
    assert place_candidates([cand], plug, 0.012, 0.02) == []                       # opening larger than the gripper: dropped
    del a_w


def test_plug_pose_from_the_tag_offset():
    T = np.eye(4)
    T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]                                 # tag rotated by 90 deg
    T[:3, 3] = [0.2, 0.1, 0.0]
    p = plug_pose(T, dict(xy_m=[0.04, 0.0], yaw_deg=10.0, z_m=0.0))
    assert np.allclose(p["xy"], [0.2, 0.14], atol=1e-9) and abs(np.degrees(p["yaw"]) - 100.0) < 1e-6   # offset 4 cm along the tag x axis, which now points along +y


def test_plug_pose_from_the_sticker_plane():
    az, tilt = np.radians(10.0), np.radians(74.0)                                  # wide-face normal at 10 deg, plane 74 deg from horizontal
    n = np.array([np.cos(az) * np.sin(tilt), np.sin(az) * np.sin(tilt), np.cos(tilt)])
    u = np.cross(n, [0, 0, 1.0]); u /= np.linalg.norm(u); v = np.cross(n, u)
    rng = np.random.default_rng(2)
    centre = np.array([0.26, -0.02, 0.01])
    G = centre + rng.uniform(-0.006, 0.006, (200, 1)) * u + rng.uniform(-0.006, 0.006, (200, 1)) * v
    st = plug_pose_from_sticker(G, centre + n * 0.2)                               # camera on the outward side
    assert np.allclose(st["xy"], centre[:2] - n[:2] * 0.00375, atol=2e-4)
    assert abs(((np.degrees(st["yaw"]) - 100.0 + 90) % 180) - 90) < 2.0           # plug x axis = normal azimuth - 90 deg


def test_unmeasured_panel_map_refuses_to_run():
    with pytest.raises(RuntimeError, match="not measured"):
        load_panel_map(ROOT / "config" / "panel_map.yaml")


def test_panel_height_comes_from_the_depth_over_the_tag():
    img = _render(np.array([0.31, 0.02]), 30.0)
    t = detect_tag_pose(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, T_BC, TAG)
    depth = np.full(img.shape, 0.2, np.float32)                                    # flat panel at z = 0 seen from 0.2 m
    depth += np.random.default_rng(0).normal(0, 0.0005, depth.shape).astype(np.float32)   # 0.5 mm depth noise
    assert abs(tag_plane_height(depth, K, T_BC, t["corners_px"])) < 0.0005
    assert tag_plane_height(np.zeros_like(depth), K, T_BC, t["corners_px"]) is None
