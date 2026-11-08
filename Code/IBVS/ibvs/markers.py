"""ArUco corner features -> normalized image points, with validity per marker."""
import cv2
import numpy as np


class Tracker:
    def __init__(self, cfg_cam, cfg_mk):
        self.K = np.array(cfg_cam["K"], float).reshape(3, 3)
        self.dist = np.array(cfg_cam["distortion"], float)
        self.m = cfg_mk
        self.ids = list(cfg_mk["ids"])
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg_mk["aruco_dict"])),
                                           cv2.aruco.DetectorParameters())
        self.target_px = np.array([cfg_mk["target_pixels"][i] for i in self.ids], float)   # (m,4,2)
        self.star = self.normalize(self.target_px.reshape(-1, 2)).reshape(-1, 4, 2)
        self.prev = {}

    def normalize(self, px):
        return cv2.undistortPoints(np.asarray(px, np.float64).reshape(-1, 1, 2), self.K, self.dist).reshape(-1, 2)

    def detect(self, gray):
        """Returns (px (m,4,2) with NaN for invalid markers, valid (m,) bool)."""
        corners, found, _ = self.det.detectMarkers(gray)
        seen = {} if found is None else {int(i): c.reshape(4, 2) for i, c in zip(found.ravel(), corners)}
        px = np.full((len(self.ids), 4, 2), np.nan)
        for k, i in enumerate(self.ids):
            c = seen.get(i)
            if c is None:
                continue
            side = np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1).min()
            jump = np.linalg.norm(c - self.prev[i], axis=1).max() if i in self.prev else 0.0
            if side >= self.m["min_side_px"] and jump <= self.m["max_jump_px"]:
                px[k] = c
            self.prev[i] = c
        return px, ~np.isnan(px[:, 0, 0])

    def features(self, px, valid):
        """Current and desired normalized points of the valid markers, (n,2) each, plus the pixel error rms."""
        s = self.normalize(px[valid].reshape(-1, 2))
        s_star = self.star[valid].reshape(-1, 2)
        f = np.array([self.K[0, 0], self.K[1, 1]])
        return s, s_star, float(np.sqrt((((s - s_star) * f) ** 2).sum(1).mean()))
