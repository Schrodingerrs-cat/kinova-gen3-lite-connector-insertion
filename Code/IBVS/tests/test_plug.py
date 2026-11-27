"""Plug position between the blue fingertips."""
import pathlib
import sys

import cv2
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ibvs.plug import finger_pair_x  # noqa: E402


def frame(left, right, y=440):
    img = np.full((480, 848, 3), 255, np.uint8)
    for x in (left, right):
        cv2.rectangle(img, (x - 15, y - 20), (x + 15, y + 20), (160, 60, 20), -1)       # BGR blue
    return img


def test_middle_of_two_fingertips():
    assert finger_pair_x(frame(440, 500)) == pytest.approx(470, abs=1)
    assert finger_pair_x(frame(425, 485)) == pytest.approx(455, abs=1)


def test_blue_object_elsewhere_is_ignored():
    img = frame(440, 500)
    cv2.rectangle(img, (800, 320), (830, 360), (160, 60, 20), -1)                       # blue thing in the background
    assert finger_pair_x(img) == pytest.approx(470, abs=1)


def test_one_finger_only_gives_none():
    assert finger_pair_x(frame(440, 440)) is None


def test_taught_usb_frame():
    taught = sorted((ROOT.parents[1] / "Media" / "IBVS" / "poses").glob("taught_usb_insert_*/color.png"))
    if not taught:
        pytest.skip("taught frame not in the checkout")
    assert finger_pair_x(cv2.imread(str(taught[0]))) == pytest.approx(471.9, abs=3)
