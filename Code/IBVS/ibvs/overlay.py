"""Show the selected marker and its corner correspondences on a camera image."""
import cv2
import numpy as np

from .markers import Tracker
from .rectangles import make_rectangle_tracker
from .shape_control import shape_metrics


class FeatureOverlay:
    def __init__(self, config):
        self.marker = dict(config["feature_tracking"])
        self.feature_type = self.marker.get('type', 'aruco')
        self.holes = self.feature_type == 'hole_rectangles'
        self.rectangle = self.feature_type in ('rectangle', 'hole_rectangles')
        if self.holes:
            from .hole_rectangles import validate_hole_settings
            validate_hole_settings(self.marker)
        if not self.rectangle and len(self.marker["ids"]) != 1:
            raise ValueError("the center rectangle expects one marker")
        self.marker_id = 0 if self.rectangle else int(self.marker["ids"][0])
        self.label = "RECTANGLE / SQUARE" if self.rectangle else f"ArUco ID {self.marker_id}"
        if self.holes:
            self.label = f"HOLE RECTANGLE {self.marker.get('selected_rectangle', 1)} / 3"
        self.order = np.asarray(self.marker.pop("canonical_target_order"), dtype=int)
        if self.order.shape != (4,) or sorted(self.order.tolist()) != list(range(4)):
            raise ValueError("canonical_target_order must be a permutation of 0..3")
        self.target = np.asarray(config["corners_image_px"], dtype=float)[self.order]
        self.marker["target_pixels"] = {self.marker_id: self.target.tolist()}
        self.tolerance = float(config["tolerance_px"])
        self.required_frames = int(config["converge_frames"])
        self.stale_s = float(self.marker.pop("camera_stale_s"))
        if (not np.isfinite(self.target).all() or self.target.shape != (4, 2)
                or not np.isfinite(self.tolerance) or self.tolerance <= 0
                or self.required_frames < 1 or not np.isfinite(self.stale_s) or self.stale_s <= 0):
            raise ValueError("invalid feature-overlay configuration")
        self.target_shape = config.get('target_shape', 'rectangle')
        if self.target_shape == 'square':
            sides = np.roll(self.target, -1, axis=0)-self.target
            lengths = np.linalg.norm(sides, axis=1)
            if (lengths.min() <= 0 or not np.allclose(lengths, lengths[0])
                    or not np.allclose(np.sum(sides*np.roll(sides, -1, axis=0), axis=1), 0.)):
                raise ValueError('square target requires four equal sides and right angles')
        self.tracker = self.camera_key = self.last_stamp = None
        self.corners = self.errors = None
        self.matched_frames = 0

    def select_rectangle(self, number):
        from .hole_rectangles import rectangle_number
        if not self.holes:
            raise ValueError('rectangle selection requires hole_rectangles tracking')
        number = rectangle_number(number)
        self.marker['selected_rectangle'] = number
        self.label = f'HOLE RECTANGLE {number} / 3'
        self.tracker = self.camera_key = None
        self.clear()

    def clear(self):
        self.corners = self.errors = None
        self.matched_frames = 0
        self.last_stamp = None
        if self.tracker is not None:
            self.tracker.prev.clear()

    def render(self, image, K, distortion, stamp, age_s=0.0, frame_valid=True,
               observation=None, draw_target=False, phase="", feature_tracker=None):
        """Detect on raw pixels; count each fresh camera timestamp only once."""
        result = image.copy()
        color = (80, 90, 255)
        if not frame_valid:
            self.clear()
            state = "CAMERA FRAME / SIZE MISMATCH"
        elif not np.isfinite(age_s) or age_s > self.stale_s:
            self.clear()
            state = "CAMERA STALE"
        elif K is None:
            self.clear()
            state = "WAITING FOR CAMERA INFO"
        else:
            K = np.asarray(K, dtype=float).reshape(3, 3)
            distortion = np.asarray(distortion, dtype=float)
            camera_key = (tuple(K.ravel()), tuple(distortion.ravel()))
            if self.camera_key != camera_key:
                camera = dict(K=K.tolist(), distortion=distortion.tolist())
                self.tracker = (make_rectangle_tracker(camera, self.marker, self.target) if self.rectangle
                                else Tracker(camera, self.marker))
                self.camera_key = camera_key
                self.clear()
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.clear()
            if stamp != self.last_stamp:
                self.last_stamp = stamp
                pixels, valid = (self.tracker.detect(image if self.holes else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
                                 if observation is None else observation)
                self.corners = pixels[0] if valid[0] else None
                if self.corners is None:
                    self.errors = None
                    self.matched_frames = 0
                else:
                    self.errors = np.linalg.norm(self.corners - self.target, axis=1)
                    self.matched_frames = (min(self.matched_frames + 1, self.required_frames)
                                           if self.errors.max() <= self.tolerance else 0)
            state = "NOT AVAILABLE (missing or rejected)" if self.corners is None else "TRACKING"
            if self.corners is not None:
                color = (255, 230, 0)
                if self.matched_frames:
                    state = f"IN TOLERANCE {self.matched_frames}/{self.required_frames}"
                    color = (80, 235, 80)

        context = feature_tracker if feature_tracker is not None else self.tracker
        if self.holes and context is not None and K is not None and frame_valid and np.isfinite(age_s) and age_s <= self.stale_s:
            context.draw_context(result)
            if self.corners is None:
                state = context.status
        if self.corners is not None:
            pixels = np.rint(self.corners).astype(np.int32)
            center = pixels.mean(axis=0)
            cv2.polylines(result, [pixels.reshape(4, 1, 2)], True, (255, 230, 0), 2, cv2.LINE_AA)
            for index, (actual, target) in enumerate(zip(pixels, self.target)):
                actual = tuple(int(v) for v in actual)
                cv2.circle(result, actual, 4, (255, 230, 0), -1, cv2.LINE_AA)
                feature = f'H{index+1}' if self.holes else f'C{index}'
                label = f"{feature} -> P{self.order[index]+1}"
                width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0]
                x = actual[0] - width - 7 if actual[0] < center[0] else actual[0] + 7
                y = actual[1] - 8 if actual[1] < center[1] else actual[1] + 18
                anchor = (max(2, min(x, image.shape[1]-width-2)), max(12, min(y, image.shape[0]-2)))
                cv2.putText(result, label, anchor,
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 230, 0), 1, cv2.LINE_AA)
                if self.errors[index] > self.tolerance:
                    target = tuple(np.rint(target).astype(int))
                    cv2.arrowedLine(result, actual, target, (255, 80, 230), 1, cv2.LINE_AA, tipLength=.08)

        tint = result[36:88].copy()
        tint[:] = (24, 28, 34)
        cv2.addWeighted(tint, .90, result[36:88], .10, 0, result[36:88])
        cv2.putText(result, f"{phase + ' | ' if phase else ''}FEATURE: {self.label} | {state}", (12, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
        detail = "No current feature corners are being used."
        if self.errors is not None:
            rms = float(np.sqrt(np.mean(self.errors**2)))
            detail = f"Corner error: RMS {rms:.1f} px | max {self.errors.max():.1f} px | tolerance {self.tolerance:g} px"
        cv2.putText(result, detail, (12, 80), cv2.FONT_HERSHEY_SIMPLEX, .5, (235, 235, 235), 1, cv2.LINE_AA)
        if self.rectangle and self.corners is not None:
            metrics = shape_metrics(self.corners, self.target)
            cv2.putText(result, f"Center {metrics['center_error_px']:.1f} px | size {100*metrics['size_ratio']:.1f}% | angle {metrics['orientation_error_deg']:+.1f} deg",
                        (12, 106), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 230, 0), 1, cv2.LINE_AA)
        if draw_target:
            target = self.target[np.argsort(self.order)]
            pixels = np.rint(target).astype(np.int32)
            cv2.polylines(result, [pixels.reshape(4,1,2)], True, (0,255,180), 2, cv2.LINE_AA)
            for index, (u,v) in enumerate(pixels):
                cv2.circle(result, (int(u),int(v)), round(self.tolerance), (0,255,180), 1)
                cv2.putText(result, f"P{index+1}", (int(u)+8,int(v)-7),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,180), 1, cv2.LINE_AA)
            cv2.putText(result, f"TARGET {self.target_shape.upper()} | standard image-based visual servo", (12,28),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0,255,180), 1, cv2.LINE_AA)
        return result
