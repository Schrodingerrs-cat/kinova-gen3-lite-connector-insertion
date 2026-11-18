"""Four image corners from an observed rectangular contour; no marker decoding."""
import cv2
import numpy as np


class TrackingLost(RuntimeError):
    """No fresh measured corners/depth; motion must hold until reacquisition."""


class RectangleTracker:
    def __init__(self, camera, config, target):
        self.K = np.asarray(camera["K"], float).reshape(3, 3)
        self.dist = np.asarray(camera["distortion"], float)
        self.c = config
        thresholds = config.get('intensity_thresholds')
        if thresholds is None and 'intensity_threshold' in config:
            thresholds = [config['intensity_threshold']]
        self.thresholds = None if thresholds is None else np.asarray(thresholds, float)
        if self.thresholds is not None and (self.thresholds.ndim != 1 or not len(self.thresholds)
                or not np.isfinite(self.thresholds).all() or np.any(self.thresholds <= 0) or np.any(self.thresholds >= 255)):
            raise ValueError('intensity thresholds must be between 0 and 255')
        self.target_px = np.asarray(target, float).reshape(1, 4, 2)
        self.star = self.normalize(self.target_px[0]).reshape(1, 4, 2)
        self.prev = {}
        self.candidates = []

    def normalize(self, pixels):
        return cv2.undistortPoints(np.asarray(pixels, np.float64).reshape(-1, 1, 2),
                                   self.K, self.dist).reshape(-1, 2)

    def detect(self, gray):
        masks = ([cv2.Canny(gray, self.c["canny_low"], self.c["canny_high"])] if self.thresholds is None else
                 [cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)[1] for threshold in self.thresholds])
        contours = []
        for mask in masks:
            contours.extend(cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0])
        height, width = gray.shape
        self.candidates = []
        for contour in contours:
            polygon = cv2.approxPolyDP(contour, self.c["polygon_epsilon"] * cv2.arcLength(contour, True), True)
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue
            points = polygon.reshape(4, 2).astype(float)
            area = abs(cv2.contourArea(polygon))
            if area > self.c["max_area_fraction"] * width * height:
                continue
            if np.any(points < 3) or np.any(points >= np.array([width-3, height-3])):
                continue
            center = points.mean(axis=0)
            points = points[np.argsort(np.arctan2(points[:, 1]-center[1], points[:, 0]-center[0]))]
            points = np.roll(points, -np.argmin(points.sum(axis=1)), axis=0)
            sides = np.roll(points, -1, axis=0) - points
            lengths = np.linalg.norm(sides, axis=1)
            if lengths.min() < self.c["min_side_px"] or lengths.max()/lengths.min() > self.c["max_side_ratio"]:
                continue
            cosine = np.abs(np.sum(sides * np.roll(sides, 1, axis=0), axis=1) /
                            (lengths * np.roll(lengths, 1)))
            if cosine.max() > self.c["max_angle_cosine"]:
                continue
            self.candidates.append((area, points))
        pixels, valid = np.full((1, 4, 2), np.nan), np.array([False])
        if not self.candidates:
            return pixels, valid
        if 0 not in self.prev:
            ordered = [p for _,p in sorted(self.candidates,key=lambda item:item[0],reverse=True)]
        else:
            matches = [np.roll(p,shift,axis=0) for _,p in self.candidates for shift in range(4)]
            ordered = sorted(matches,key=lambda p:np.linalg.norm(p-self.prev[0],axis=1).max())
        window = int(self.c.get('subpixel_window_px',0))
        for points in ordered:
            if 0 in self.prev and np.linalg.norm(points-self.prev[0],axis=1).max() > self.c['max_jump_px']:
                continue
            if window:
                refined = cv2.cornerSubPix(gray,points.astype(np.float32).reshape(4,1,2),(window,window),(-1,-1),
                                           (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.01)).reshape(4,2)
                if (not np.isfinite(refined).all() or np.abs(refined-points).max() > window
                        or np.any(refined < 3) or np.any(refined >= np.array([width-3,height-3]))):
                    continue
                points = refined.astype(float)
                if 0 in self.prev and np.linalg.norm(points-self.prev[0],axis=1).max() > self.c['max_jump_px']:
                    continue
            self.prev[0] = points.copy()
            pixels[0],valid[0] = points,True
            return pixels,valid
        return pixels,valid


def mean_rectangle_depths(depth_m, pixels, lo, hi, min_points=30):
    """Uniform IBVS depth estimate from measured pixels inside the rectangle."""
    pixels = np.asarray(pixels,float)
    inner = pixels.mean(axis=0)+.8*(pixels-pixels.mean(axis=0))
    mask = np.zeros(depth_m.shape,np.uint8)
    cv2.fillConvexPoly(mask,np.rint(inner).astype(np.int32),1)
    samples = depth_m[mask.astype(bool)]
    valid = samples[np.isfinite(samples)&(samples>=lo)&(samples<=hi)]
    if len(valid)<min_points or len(valid)<.25*len(samples):
        raise ValueError("rectangle interior has insufficient measured depth")
    mean = float(valid.mean())
    return np.full(4,mean), dict(method="uniform mean measured interior depth; no plane fit",
                                 samples=len(samples),valid_samples=len(valid),mean_depth_m=mean)
