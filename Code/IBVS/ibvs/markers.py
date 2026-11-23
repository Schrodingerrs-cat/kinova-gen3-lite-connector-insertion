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


class AutoMarkerTracker:
    """Picks a visible marker at run time and builds its goal from the live view. Nothing is taught or remembered.

    Goal = the chosen marker centred on the optical axis, upright, at the size it has when chosen. The marker closest to the
    image centre is chosen (or `marker_id` if given), once it is seen `lock_frames` times in a row. Same interface as Tracker."""

    def __init__(self, cfg_cam, cfg):
        self.K = np.array(cfg_cam["K"], float).reshape(3, 3)
        self.dist = np.array(cfg_cam["distortion"], float)
        self.m, self.forced, self.order = cfg, cfg.get("marker_id"), cfg.get("marker_order", "nearest")
        self.visible, self.sequence = {}, []
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["aruco_dict"]))
        modern = hasattr(cv2.aruco, "ArucoDetector")
        params = cv2.aruco.DetectorParameters() if modern else cv2.aruco.DetectorParameters_create()
        params.detectInvertedMarker = bool(cfg.get("detect_inverted_marker", False))
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params = params
        self.det = cv2.aruco.ArucoDetector(self.dictionary, params) if modern else None
        self.ids, self.prev, self.candidates, self.status = [], {}, [], "looking for a marker"
        self.locked, self._seen, self._count = None, None, 0
        self.target_px = np.zeros((1, 4, 2))
        self.star = np.zeros((1, 4, 2))

    def normalize(self, pixels):
        return cv2.undistortPoints(np.asarray(pixels, np.float64).reshape(-1, 1, 2), self.K, self.dist).reshape(-1, 2)

    @staticmethod
    def sides(c):
        return np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)

    def choose(self, seen):
        usable = {i: c for i, c in seen.items() if self.sides(c).min() >= self.m["min_side_px"]}
        if self.forced is not None:
            return self.forced if self.forced in usable else None
        if self.order == "left":
            return min(usable, key=lambda i: usable[i][:, 0].mean()) if usable else None
        centre = np.array([self.K[0, 2], self.K[1, 2]])
        return min(usable, key=lambda i: np.linalg.norm(usable[i].mean(0) - centre)) if usable else None

    def lock(self, marker_id, corners):
        taught = self.m.get("taught_goal_px")
        if taught is not None:                                          # the goal that was taught for this socket
            goal = np.array(taught, float)
        else:
            tilt = float(np.degrees(np.arctan2(*(corners[1] - corners[0])[::-1])))
            if abs(tilt) > 20.:
                self.status = "marker %d is rotated %.0f deg in the image: the camera is not facing the wall upright" % (marker_id, tilt)
                return
            h = self.sides(corners).mean() / 2
            dx, dy = self.m.get("goal_offset_px", [0, 0])                 # image x right, y DOWN
            cx, cy = self.K[0, 2] + dx, self.K[1, 2] + dy
            goal = np.array([[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]])
        self.target_px, self.star = goal.reshape(1, 4, 2), self.normalize(goal).reshape(1, 4, 2)
        self.ids, self.locked, self.prev = [marker_id], marker_id, {marker_id: corners}
        self.sequence = sorted(self.visible, key=self.visible.get)               # every usable marker in view, left to right
        self.status = "tracking marker %d" % marker_id

    def detect(self, gray):
        """(px (1,4,2) NaN when invalid, valid (1,)). Unlocked: choose; locked: track the chosen id."""
        corners, ids, _ = (self.det.detectMarkers(gray) if self.det is not None
                           else cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params))
        seen = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}
        self.candidates = sorted(seen)
        self.visible = {i: float(c[:, 0].mean()) for i, c in seen.items() if self.sides(c).min() >= self.m["min_side_px"]}
        px, valid = np.full((1, 4, 2), np.nan), np.array([False])
        if self.locked is None:
            pick = self.choose(seen)
            self._count = self._count + 1 if pick is not None and pick == self._seen else (1 if pick is not None else 0)
            self._seen = pick
            if pick is None:
                self.status = "no usable marker in view" if not seen else "markers %s seen but too small" % sorted(seen)
            elif self._count >= self.m.get("lock_frames", 5):
                self.lock(pick, seen[pick])
            return px, valid
        c = seen.get(self.locked)
        if c is not None:
            jump = np.linalg.norm(c - self.prev[self.locked], axis=1).max()
            if self.sides(c).min() >= self.m["min_side_px"] and jump <= self.m["max_jump_px"]:
                px[0], valid[0] = c, True
            self.prev[self.locked] = c
        return px, valid
