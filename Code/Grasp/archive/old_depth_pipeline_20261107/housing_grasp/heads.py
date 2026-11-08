"""Find cable heads in a fused workspace cloud, analytically (no ROI, no markers), and plan grasps on them.

The fused cloud's table is not a perfect plane (a few mm of warp over the workspace), so a global plane
cannot separate objects. Instead: top-view height map (per-cell median z), local ground = low percentile of
that map over a window larger than a head, height = map - ground. Heads are the compact blobs that stand
well above the local ground; the cable body stays low and tape/noise are too small. Each blob is then
measured and planned by the same pipeline as a user ROI (plan_grasp), and ranked.
"""
import numpy as np
from scipy import ndimage



def height_map(cloud, cell):
    """Per-cell median z on a regular top-view grid. Returns (grid with NaN = empty, origin xy)."""
    lo = cloud[:, :2].min(0)
    ij = np.floor((cloud[:, :2] - lo) / cell).astype(int)
    shape = ij.max(0) + 1
    key = ij[:, 0] * shape[1] + ij[:, 1]
    order = np.lexsort((cloud[:, 2], key))
    key_s, z_s = key[order], cloud[order, 2]
    first = np.r_[0, np.flatnonzero(np.diff(key_s)) + 1]
    count = np.diff(np.r_[first, len(key_s)])
    grid = np.full(shape[0] * shape[1], np.nan)
    grid[key_s[first]] = z_s[first + count // 2]
    return grid.reshape(shape), lo


def head_blobs(cloud, hd):
    """Label map of compact blobs standing above the local ground. Returns (labels, height map, origin)."""
    cell = hd["cell_m"]
    grid, lo = height_map(cloud, cell)
    ground = ndimage.percentile_filter(np.where(np.isnan(grid), np.nanmedian(grid), grid), hd["ground_percentile"],
                                       size=int(hd["ground_window_m"] / cell))
    height = grid - ground
    mask = ndimage.binary_opening(np.nan_to_num(height) > hd["min_head_height_m"], iterations=1)
    return ndimage.label(mask)[0], height, lo


def find_heads(cloud, cfg):
    """Head candidates in the workspace, best first: dict(center xyz, area_m2, height_m, score).

    Only LOCATES heads. The fused cloud is too blurred (view-to-view registration error) to size them or
    to plan a grasp; that is done afterwards from a close live view (plan_grasp).
    """
    hd = cfg["heads"]
    labels, height, lo = head_blobs(cloud, hd)
    cell = hd["cell_m"]
    x0, x1, y0, y1 = hd["region"]
    out = []
    for k in range(1, labels.max() + 1):
        blob = labels == k
        area = blob.sum() * cell ** 2
        ci, cj = np.argwhere(blob).mean(0)
        cx, cy = lo[0] + (ci + 0.5) * cell, lo[1] + (cj + 0.5) * cell
        top = float(np.nanpercentile(height[blob], 90))
        if not (x0 <= cx <= x1 and y0 <= cy <= y1) or not (hd["min_area_m2"] <= area <= hd["max_area_m2"]) or top > hd["max_head_height_m"]:
            continue
        near = (np.abs(cloud[:, 0] - cx) < cell) & (np.abs(cloud[:, 1] - cy) < cell)
        top_z = float(np.percentile(cloud[near, 2], 80)) if near.any() else 0.0
        out.append(dict(center=np.array([cx, cy, top_z]), area_m2=float(area), height_m=top, score=top))
    return sorted(out, key=lambda c: -c["score"])
