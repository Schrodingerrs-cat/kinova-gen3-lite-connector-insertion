"""ArUco corner features -> normalized image points, with validity per marker."""
from datetime import datetime, timezone

import cv2
import numpy as np
import yaml


class TrackingLost(RuntimeError):
    """No fresh measured corners/depth; motion must hold until reacquisition."""


class Tracker:
    def __init__(self, cfg_cam, cfg_mk):
        self.K = np.array(cfg_cam["K"], float).reshape(3, 3)
        self.dist = np.array(cfg_cam["distortion"], float)
        self.m = cfg_mk
        self.ids = list(cfg_mk["ids"])
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg_mk["aruco_dict"]))
        self.params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "ArucoDetector")
                       else cv2.aruco.DetectorParameters_create())
        self.params.detectInvertedMarker = bool(cfg_mk.get("detect_inverted_marker", False))
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(self.dictionary, self.params) if hasattr(cv2.aruco, "ArucoDetector") else None
        self.target_px = np.array([cfg_mk["target_pixels"][i] for i in self.ids], float)   # (m,4,2)
        self.star = self.normalize(self.target_px.reshape(-1, 2)).reshape(-1, 4, 2)
        self.prev = {}
        self.candidates = []

    def normalize(self, px):
        return cv2.undistortPoints(np.asarray(px, np.float64).reshape(-1, 1, 2), self.K, self.dist).reshape(-1, 2)

    def detect(self, gray):
        """Returns (px (m,4,2) with NaN for invalid markers, valid (m,) bool)."""
        if self.det is None:
            corners, found, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        else:
            corners, found, _ = self.det.detectMarkers(gray)
        seen = {} if found is None else {int(i): c.reshape(4, 2) for i, c in zip(found.ravel(), corners)}
        self.candidates = list(seen)
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


def mean_marker_depth(depth_m, pixels, lo, hi, min_points=30):
    """One depth for all four corners: mean measured depth inside the marker (flat paper on the wall)."""
    pixels = np.asarray(pixels, float)
    inner = pixels.mean(axis=0) + .8 * (pixels - pixels.mean(axis=0))
    mask = np.zeros(depth_m.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.rint(inner).astype(np.int32), 1)
    samples = depth_m[mask.astype(bool)]
    valid = samples[np.isfinite(samples) & (samples >= lo) & (samples <= hi)]
    if len(valid) < min_points or len(valid) < .25 * len(samples):
        raise ValueError("marker interior has insufficient measured depth")
    mean = float(valid.mean())
    return np.full(4, mean), dict(method="uniform mean measured interior depth; no plane fit", samples=len(samples),
                                  valid_samples=len(valid), mean_depth_m=mean)


def median_corners(frames, max_std_px=.5):
    """Goal corners from several detections of one still marker; refuses a moving or noisy view."""
    frames = np.asarray(frames, float)
    if frames.ndim != 3 or frames.shape[1:] != (4, 2) or len(frames) < 10 or not np.isfinite(frames).all():
        raise ValueError("need at least 10 finite four-corner detections")
    if frames.std(axis=0).max() > max_std_px:
        raise ValueError("marker moved or is noisy: corner std %.2f px (limit %.2f)" % (frames.std(axis=0).max(), max_std_px))
    return np.median(frames, axis=0)


def save_goal(path, socket, marker_id, corners, K, std_px):
    """Add or replace one socket's taught goal in the goals file, keeping the other sockets."""
    goals = (yaml.safe_load(open(path)) if path.exists() else None) or {}
    goals[socket] = dict(marker_id=int(marker_id), corners_image_px=np.asarray(corners, float).tolist(),
                         camera_K=np.asarray(K, float).tolist(), corner_std_px=float(std_px),
                         taught_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"))
    header = "".join(line for line in open(path) if line.startswith("#")) if path.exists() else ""
    path.write_text(header + yaml.safe_dump(goals, sort_keys=True))
