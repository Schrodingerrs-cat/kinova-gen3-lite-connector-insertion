"""Synthetic checks of the green-marker stages (geometry only, not hardware evidence)."""
import pathlib, sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp.green import dbscan, green_mask, marker_planes, ransac_plane  # noqa: E402

G = load_config(str(pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"), allow_unscanned=True)["green"]


def test_mask_takes_head_green_not_wood_or_white():
    img = np.zeros((40, 40, 3), np.uint8)
    img[:] = (60, 110, 190)                    # wood, BGR
    img[5:15, 5:15] = (200, 200, 20)           # teal/green (H~90, S high)
    img[25:35, 25:35] = (250, 250, 245)        # white
    m = green_mask(img, G)
    assert m[5:15, 5:15].mean() > 0.9 and not m[25:35, 25:35].any() and not m[20:24, 0:4].any()


def test_ransac_finds_tilted_table_and_ignores_object():
    rng = np.random.default_rng(1)
    xy = rng.uniform(-0.15, 0.15, (20000, 2))
    z = 0.05 * xy[:, 0] + rng.normal(0, 0.0005, 20000)
    obj = np.c_[rng.uniform(-0.01, 0.01, (2000, 2)), rng.uniform(0.01, 0.03, 2000)]
    n, d, _ = ransac_plane(np.vstack([np.c_[xy, z], obj]), 0.003)
    assert n[2] > 0.99 and abs(np.degrees(np.arccos(n[2])) - np.degrees(np.arctan(0.05))) < 0.3


def test_two_marker_plates_split_with_outward_normals():
    rng = np.random.default_rng(2)
    plate = lambda y: np.c_[rng.uniform(-0.01, 0.01, 1500), rng.normal(y, 0.0002, 1500), rng.uniform(0.0, 0.007, 1500)]
    (P1, c1, n1), (P2, c2, n2) = marker_planes(np.vstack([plate(-0.00885), plate(0.00885)]), G)
    lo, hi = sorted([(c1, n1), (c2, n2)], key=lambda t: t[0][1])
    assert abs(hi[0][1] - lo[0][1] - 0.0177) < 0.001
    assert lo[1][1] < -0.95 and hi[1][1] > 0.95    # normals point away from each other


def test_dbscan_separates_blobs():
    rng = np.random.default_rng(3)
    P = np.vstack([rng.normal(0, 0.001, (200, 3)), rng.normal(0.05, 0.001, (200, 3))])
    assert len(set(dbscan(P, 0.006, 15)) - {-1}) == 2


def test_green_frame_of_a_flat_patch():
    from housing_grasp.green import green_frame, largest_blob
    rng = np.random.default_rng(4)
    C = np.c_[rng.uniform(-0.015, 0.015, 3000), rng.uniform(-0.006, 0.006, 3000), rng.normal(0.01, 0.0001, 3000)]
    f = green_frame(C)
    assert abs(f["normal"][2]) > 0.99 and abs(abs(f["long_axis"][0]) - 1) < 0.02 and abs(f["length_m"] - 0.03) < 0.002 and abs(f["width_m"] - 0.012) < 0.002
    m = np.zeros((50, 50), bool); m[10:20, 30:40] = True; m[40:42, 5:7] = True
    assert largest_blob(m, 20) == (30, 10, 40, 20) and largest_blob(m, 500) is None
