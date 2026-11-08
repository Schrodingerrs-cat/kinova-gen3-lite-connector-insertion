"""Stage 2: support plane + the solid object standing on it (independent of the ROI size)."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


def fit_plane(points, cam_origin, threshold=0.0015, iters=300, seed=0):
    """RANSAC plane + least-squares refit. Returns (point_on_plane, unit normal facing the camera)."""
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        p = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((points - p[0]) @ n) < threshold
        if inl.sum() > best_n:
            best, best_n = inl, inl.sum()
    if best is None:
        raise ValueError("plane fit failed")
    q = points[best]
    c = q.mean(0)
    n = np.linalg.svd(q - c, full_matrices=False)[2][-1]
    return c, (n if (cam_origin - c) @ n >= 0 else -n)


def roi_cluster(points, in_roi, radius):
    """Boolean mask of the connected component (radius graph) holding the most ROI points."""
    pairs = cKDTree(points).query_pairs(radius, output_type="ndarray")
    n = len(points)
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    _, lab = connected_components(g, directed=False)
    return lab == np.bincount(lab[in_roi], minlength=lab.max() + 1).argmax()


def segment_object(points, in_roi, cam_origin, p):
    """Returns (plane (point, normal), object_mask over points, height of every point above the plane).

    The plane is first fitted on a ring around the ROI, then refitted on table points hugging the
    segmented object, so a large or small ROI box gives the same plane and the same object.
    """
    xy_roi = points[in_roi, :2].mean(0)
    ring = ~in_roi & (np.linalg.norm(points[:, :2] - xy_roi, axis=1) < p["plane_ring_m"])
    plane = fit_plane(points[ring] if ring.sum() > 200 else points, cam_origin)  # fallback: dominant plane
    obj = None
    for _ in range(2):
        c0, n = plane
        h = (points - c0) @ n
        above = np.flatnonzero((h > p["min_height_m"]) & (h < p["max_height_m"]))  # fingers/arm are far above
        if not in_roi[above].any():
            raise ValueError("no object points above the support plane inside the ROI")
        flat = points[above] - np.outer(h[above], n)
        obj = above[roi_cluster(flat, in_roi[above], p["cluster_radius_m"])]
        # refit on table points just outside the object (not the cable, not far-field)
        dist = cKDTree(points[obj] - np.outer(h[obj], n)).query(points - np.outer(h, n))[0]
        near = (dist > p["table_gap_m"]) & (dist < p["plane_ring_m"]) & (np.abs(h) < 0.012)
        if near.sum() > 200:
            plane = fit_plane(points[near], cam_origin)
    c0, n = plane
    h = (points - c0) @ n
    mask = np.zeros(len(points), bool)
    mask[obj] = True
    return plane, mask & (h > p["min_height_m"]) & (h < p["max_height_m"]), h
