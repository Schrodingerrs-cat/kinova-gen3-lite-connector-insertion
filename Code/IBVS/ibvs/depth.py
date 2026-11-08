import numpy as np


def corner_depths(depth_m, px, lo, hi, patch=2):
    """Median depth in a (2*patch+1)^2 window per corner. px (n,2). Invalid (0, NaN, out of [lo,hi]) -> NaN."""
    h, w = depth_m.shape
    out = np.full(len(px), np.nan)
    for i, (u, v) in enumerate(px):
        u, v = int(round(u)), int(round(v))
        win = depth_m[max(0, v - patch):min(h, v + patch + 1), max(0, u - patch):min(w, u + patch + 1)]
        ok = win[np.isfinite(win) & (win >= lo) & (win <= hi)]
        if ok.size > win.size // 2:
            out[i] = np.median(ok)
    return out


def fill_depths(Z):
    """Invalid corners take the median of the valid ones; None if no corner is valid (abort)."""
    ok = np.isfinite(Z)
    if not ok.any():
        return None
    return np.where(ok, Z, np.median(Z[ok]))
