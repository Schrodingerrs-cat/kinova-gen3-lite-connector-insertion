"""Four measured hole centers per rectangle, numbered left to right at acquisition."""
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from .rectangles import RectangleTracker


def rectangle_number(value):
    if isinstance(value, bool) or value not in (1, 2, 3):
        raise ValueError('selected_rectangle must be 1, 2, or 3')
    return int(value)


def order_corners(points):
    """Clockwise image order: top-left, top-right, bottom-right, bottom-left."""
    points = np.asarray(points, float)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError('four finite hole centers are required per rectangle')
    center = points.mean(axis=0)
    points = points[np.argsort(np.arctan2(points[:, 1]-center[1], points[:, 0]-center[0]))]
    points = np.roll(points, -np.argmin(points.sum(axis=1)), axis=0)
    polygon = points.astype(np.float32)
    if (not cv2.isContourConvex(polygon) or abs(cv2.contourArea(polygon)) < 100
            or np.linalg.norm(points-np.roll(points, 1, axis=0), axis=1).min() < 5):
        raise ValueError('each rectangle needs four distinct corners around a convex area')
    return points


def number_rectangles(groups):
    """Number once by image x centroid; preserve physical point order afterwards."""
    groups = np.asarray(groups, float)
    if groups.shape != (3, 4, 2):
        raise ValueError('exactly three groups of four hole centers are required')
    groups = np.asarray([order_corners(group) for group in groups])
    distances = np.linalg.norm(groups.reshape(12, 2)[:, None]-groups.reshape(12, 2), axis=2)
    np.fill_diagonal(distances, np.inf)
    if distances.min() < 5:
        raise ValueError('the 12 holes must be distinct')
    groups = groups[np.argsort(groups[:, :, 0].mean(axis=1))]
    if np.min(np.diff(groups[:, :, 0].mean(axis=1))) < 5:
        raise ValueError('rectangle centers must have a distinct left-to-right order')
    return groups


def validate_hole_settings(config):
    rectangle_number(config.get('selected_rectangle', 1))
    if config.get('hole_polarity', 'dark') not in ('dark', 'bright'):
        raise ValueError('hole_polarity must be dark or bright')
    for name, default in [('min_hole_diameter_px', 3.), ('max_hole_diameter_px', 50.),
                          ('max_hole_axis_ratio', 2.5), ('min_hole_circularity', .45),
                          ('max_jump_px', 20.), ('association_margin_px', 2.)]:
        value = config.get(name, default)
        if not np.isfinite(value) or value <= 0:
            raise ValueError(name+' must be finite and positive')
    if config.get('min_hole_diameter_px', 3.) >= config.get('max_hole_diameter_px', 50.):
        raise ValueError('invalid hole diameter interval')
    if config.get('max_hole_axis_ratio', 2.5) < 1 or config.get('min_hole_circularity', .45) > 1:
        raise ValueError('invalid hole shape limits')
    seeds = config.get('seed_rectangles_image_px')
    if seeds is not None:
        ordered = number_rectangles(seeds)
        if not np.allclose(ordered, seeds):
            raise ValueError('seed rectangles must be left-to-right, with TL/TR/BR/BL corners')



class HoleRectangleTracker(RectangleTracker):
    def __init__(self, camera, config, target):
        validate_hole_settings(config)
        super().__init__(camera, config, target)
        self.selected_rectangle = rectangle_number(config.get('selected_rectangle', 1))
        seeds = config.get('seed_rectangles_image_px')
        self.rectangles = None if seeds is None else number_rectangles(seeds)
        self.observed_rectangles = np.full((3, 4, 2), np.nan)
        self.hole_radii = np.full((3, 4), np.nan)
        self.status = 'WAITING FOR 12 HOLES'
        self.templates = None
        if config.get('template_file'):
            from pathlib import Path
            from .hole_templates import HoleTemplates
            path = Path(config['template_file'])
            if not path.is_absolute():
                path = Path(__file__).resolve().parents[1]/'config'/path
            self.templates = HoleTemplates(path)
            if self.rectangles is None:
                raise ValueError('hole templates require labelled seed rectangles')

    def select_rectangle(self, number):
        self.selected_rectangle = rectangle_number(number)
        self.prev.clear()

    def find_holes(self, gray):
        """Ellipse centers from thresholded blobs; merge duplicate threshold detections."""
        if self.templates is not None:
            self.candidates = self.templates.detect(gray,self.rectangles)
            return self.candidates
        if gray.ndim==3:
            gray=cv2.cvtColor(gray,cv2.COLOR_BGR2GRAY)
        minimum = self.c.get('min_hole_diameter_px', 3.)
        maximum = self.c.get('max_hole_diameter_px', 50.)
        mode = cv2.THRESH_BINARY_INV if self.c.get('hole_polarity', 'dark') == 'dark' else cv2.THRESH_BINARY
        detections = []
        thresholds = self.thresholds if self.thresholds is not None else [64, 96, 128, 160, 192]
        for threshold in thresholds:
            mask = cv2.threshold(gray, threshold, 255, mode)[1]
            for contour in cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[0]:
                area, perimeter = abs(cv2.contourArea(contour)), cv2.arcLength(contour, True)
                if len(contour) < 5 or perimeter <= 0 or area <= 0:
                    continue
                if 4*np.pi*area/perimeter**2 < self.c.get('min_hole_circularity', .45):
                    continue
                try:
                    center, axes, _ = cv2.fitEllipse(contour)
                except cv2.error:
                    continue
                center, axes = np.asarray(center, float), np.asarray(axes, float)
                if not np.isfinite(center).all() or not np.isfinite(axes).all() or np.any(axes <= 0):
                    continue
                if (axes.min() < minimum or axes.max() > maximum
                        or axes.max()/axes.min() > self.c.get('max_hole_axis_ratio', 2.5)):
                    continue
                radius = float(axes.max()/2.)
                if np.any(center-radius < 1) or np.any(center+radius >= np.array(gray.shape[::-1])-1):
                    continue
                u, v = np.rint(center).astype(int)
                inner = max(1, int(radius/3))
                if np.mean(mask[v-inner:v+inner+1, u-inner:u+inner+1] > 0) < .5:
                    continue
                detections.append((area, center, radius))
        unique = []
        for detection in sorted(detections, key=lambda item: item[0], reverse=True):
            if all(np.linalg.norm(detection[1]-other[1]) > max(3., .5*min(detection[2], other[2]))
                   for other in unique):
                unique.append(detection)
        self.candidates = unique
        return unique

    def detect(self, gray):
        pixels, valid = np.full((1, 4, 2), np.nan), np.array([False])
        holes = self.find_holes(gray)
        self.observed_rectangles[:] = np.nan
        self.hole_radii[:] = np.nan
        if self.rectangles is None:
            if len(holes) != 12:
                self.status = f'NEED 12 HOLES: found {len(holes)}; use click setup for clutter'
                return pixels, valid
            centers = np.asarray([hole[1] for hole in holes])
            try:
                # Automatic grouping assumes three side-by-side rectangles.
                self.rectangles = number_rectangles(centers[np.argsort(centers[:, 0])].reshape(3, 4, 2))
            except ValueError:
                self.status = 'AMBIGUOUS GROUPING: use click setup'
                return pixels, valid
        if not holes:
            self.status = 'NO HOLES DETECTED'
            return pixels, valid
        previous = self.rectangles.reshape(12, 2).copy()
        centers = np.asarray([hole[1] for hole in holes])
        distances = np.linalg.norm(previous[:, None]-centers, axis=2)
        separation = np.linalg.norm(previous[:, None]-previous, axis=2)
        np.fill_diagonal(separation, np.inf)
        gate = min(self.c.get('max_jump_px', 20.), .4*separation.min())
        # Dummy columns let missing holes stay missing instead of stealing another match.
        cost = np.column_stack([distances, np.full((12, 12), gate)])
        rows, columns = linear_sum_assignment(cost)
        matched = []
        measured = self.observed_rectangles.reshape(12, 2)
        for row, column in zip(rows, columns):
            if column >= len(holes) or distances[row, column] >= gate:
                continue
            alternatives = np.delete(distances[row], column)
            if alternatives.size and alternatives.min()-distances[row, column] < self.c.get('association_margin_px', 2.):
                continue
            measured[row] = centers[column]
            self.hole_radii.flat[row] = holes[column][2]
            matched.append(row)
        # Move predictions with the sheet, even when some other rectangles leave view.
        # Only measured centers are ever returned to IBVS or drawn as current features.
        if len(matched) >= 4:
            H, inliers = cv2.findHomography(previous[matched], measured[matched], cv2.RANSAC, 3.)
            if H is not None and inliers.sum() >= 4:
                projected = cv2.perspectiveTransform(previous.reshape(-1, 1, 2), H).reshape(12, 2)
                if np.isfinite(projected).all():
                    self.rectangles = projected.reshape(3, 4, 2)
        for row in matched:
            self.rectangles.reshape(12, 2)[row] = measured[row]
        if self.templates is not None and len(matched)>=4:
            H,inliers=cv2.findHomography(self.templates.seed_points[matched],measured[matched],cv2.RANSAC,3.)
            if H is not None and inliers.sum()>=4:
                for row in matched:
                    point=self.templates.seed_points[row]
                    probe=np.array([point,point+[1,0],point+[0,1]],float)
                    mapped=cv2.perspectiveTransform(probe.reshape(-1,1,2),H).reshape(3,2)
                    scale=np.linalg.svd((mapped[1:]-mapped[0]).T,compute_uv=False).max()
                    self.hole_radii.flat[row]=self.templates.radii[row]*scale
        index = self.selected_rectangle-1
        selected = self.observed_rectangles[index]
        if not np.isfinite(selected).all():
            self.status = f'RECTANGLE {index+1}: MISSING OR AMBIGUOUS HOLE'
            return pixels, valid
        try:
            order_corners(selected)
        except ValueError:
            self.status = 'REJECTED HOLE GEOMETRY'
            return pixels, valid
        self.prev[0] = selected.copy()
        pixels[0], valid[0] = selected, True
        self.status = 'TRACKING'
        return pixels, valid

    def draw_context(self, image):
        for index, corners in enumerate(self.observed_rectangles):
            finite = np.isfinite(corners).all(axis=1)
            if not finite.any():
                continue
            selected = index+1 == self.selected_rectangle
            color = (255, 230, 0) if selected else (150, 150, 150)
            points = np.rint(corners[finite]).astype(int)
            if finite.all():
                cv2.polylines(image, [points.reshape(4, 1, 2)], True, color, 2 if selected else 1, cv2.LINE_AA)
            for point, hole in zip(points, np.flatnonzero(finite)+1):
                anchor = tuple(point)
                cv2.circle(image, anchor, 5, color, 1, cv2.LINE_AA)
                if not selected or not finite.all():
                    cv2.putText(image, f'R{index+1}.H{hole}', (anchor[0]+7, anchor[1]+16),
                                cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)


def hole_rim_depths(depth, pixels, radii, K, distortion, lo, hi, inlier_m, min_points, min_fraction):
    """Fit measured depth around hole rims, excluding the open hole centers."""
    from .alignment import marker_plane_depths

    mask = np.zeros(depth.shape, np.uint8)
    for point, radius in zip(pixels, radii):
        ring = np.zeros_like(mask)
        center = tuple(np.rint(point).astype(int))
        cv2.circle(ring, center, max(3, int(np.ceil(radius*1.9))), 1, -1)
        cv2.circle(ring, center, max(2, int(np.ceil(radius*1.15))), 0, -1)
        measured = depth[ring.astype(bool)]
        good = np.isfinite(measured) & (measured >= lo) & (measured <= hi)
        if good.sum() < max(5, min_points//4) or good.mean() < min_fraction:
            raise ValueError('a hole rim has insufficient measured depth')
        mask |= ring
    depths, support = marker_plane_depths(depth, pixels, K, distortion, lo, hi,
                                          inlier_m, min_points, min_fraction, mask=mask)
    support['method'] = 'measured plane around four hole rims; hole interiors excluded'
    support['mean_depth_m'] = float(depths.mean())
    return depths, support
