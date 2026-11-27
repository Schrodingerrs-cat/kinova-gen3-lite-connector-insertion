"""Where the gripper holds the plug, as the wrist camera sees it: the middle between the two blue fingertips at the bottom of the frame."""
import cv2
import numpy as np


def finger_pair_x(bgr, band=(425, 455), min_area=600, x_range=(200, 700)):
    """x pixel (848 px wide frame) of the middle of the two blue fingertips, or None if both are not found."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, (95, 90, 40), (130, 255, 255)), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _, stats, centres = cv2.connectedComponentsWithStats(mask)
    xs = [centres[i][0] for i in range(1, n)
          if stats[i, 4] > min_area and band[0] <= centres[i][1] <= band[1] and x_range[0] <= centres[i][0] <= x_range[1]]
    return None if len(xs) < 2 else float((min(xs) + max(xs)) / 2)
