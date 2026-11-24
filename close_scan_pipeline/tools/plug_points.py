"""Plug-only points per view, selected by the connector marker colour and the measured prior from vision.py (no CAD)."""
import pathlib
import sys

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vision  # noqa: E402

prior = vision.prior


def rz(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1.0]])


def points(views, pr, spec, radius=0.020, max_pts=1500, top_tol=0.006, open_mm=None, mode="teal_or_dark"):
    out = []
    for v in views:
        f = np.load(v)
        d, c, K, T = f["depth"].astype(np.float32), f["color"], f["K"], f["T_BC"]
        med = cv2.medianBlur(d, 5)
        ok = (d > 0.08) & (d < 0.8) & (np.abs(d - med) < 0.004)
        hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        mark = vision.marker_mask(hsv, spec)
        if mode == "sticker":
            cand = (cv2.morphologyEx(mark.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0) & ok
        else:
            cand = (mark | (hsv[..., 2] < 90)) & ok
        vv, uu = np.nonzero(cand)
        z = d[vv, uu]
        P = np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
        pc = pr["plug_centre"]
        keep = (np.hypot(P[:, 0] - pc[0], P[:, 1] - pc[1]) <= radius) & (P[:, 2] >= pr["board_top"] + 0.003) & (P[:, 2] <= pr["plug_top"] + top_tol)
        mask = np.zeros(d.shape, bool)
        mask[vv[keep], uu[keep]] = True
        pts = P[keep]
        if open_mm and keep.sum() > 50:
            rng = float(np.median(z[keep]))
            k = max(3, int(round(open_mm * K[0, 0] / rng / 1000)))
            op = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
            n, lab, st, _ = cv2.connectedComponentsWithStats(op)
            if n > 1:
                body = lab == (1 + int(np.argmax(st[1:, 4])))
                sel = body[vv[keep], uu[keep]]
                mask, pts = body, P[keep][sel]
        if len(pts) > max_pts:
            pts = pts[np.random.default_rng(0).choice(len(pts), max_pts, replace=False)]
        out.append(dict(name=v.stem, pts=pts, mask=mask, color=c, K=K, T=T))
    return out


def fit_box(P, lo_pct=5, hi_pct=95):
    """Rotated rectangle (min area of the percentile extents) in plan view: dict(centre, yaw of the long side mod 180, long, short, z0, z1, th, ext)."""
    xy, c = P[:, :2], np.median(P[:, :2], axis=0)
    best = None
    for th in range(0, 180):
        q = (xy - c) @ rz(-th)[:2, :2].T
        lo, hi = np.percentile(q, lo_pct, axis=0), np.percentile(q, hi_pct, axis=0)
        if best is None or np.prod(hi - lo) < best[0]:
            best = (np.prod(hi - lo), th, lo, hi)
    _, th, lo, hi = best
    ext = hi - lo
    return dict(centre=c + rz(th)[:2, :2] @ ((lo + hi) / 2), yaw=float((th if ext[0] >= ext[1] else th + 90) % 180), long=float(max(ext)), short=float(min(ext)),
                z0=float(np.percentile(P[:, 2], 5)), z1=float(np.percentile(P[:, 2], 95)), th=th, ext=ext)


def _vertical_normals(views, data):
    """Per view the surface normals (azimuth deg, unit vectors) of the plug pixels that lie on near-vertical faces, from the depth of the run. Yields (view name, azimuth array, normal array)."""
    for v, d in zip(views, data):
        f = np.load(v)
        dep, K, T = f["depth"].astype(np.float32), f["K"], f["T_BC"]
        med = cv2.medianBlur(dep, 5)
        ok = (dep > 0.08) & (dep < 0.8) & (np.abs(dep - med) < 0.004)
        v2, u2 = np.nonzero(ok)
        z = dep[v2, u2]
        Pm = np.full(dep.shape + (3,), np.nan, np.float32)
        Pm[v2, u2] = np.c_[(u2 - K[0, 2]) * z / K[0, 0], (v2 - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
        vv, uu = np.nonzero(d["mask"])
        s = 2
        sel = (vv >= s) & (vv < dep.shape[0] - s) & (uu >= s) & (uu < dep.shape[1] - s)
        vv, uu = vv[sel], uu[sel]
        A, B = Pm[vv, uu + s] - Pm[vv, uu - s], Pm[vv + s, uu] - Pm[vv - s, uu]
        N = np.cross(A, B)
        n = np.linalg.norm(N, axis=1)
        g = np.isfinite(n) & (n > 0) & (np.abs(A).max(1) < 0.006) & (np.abs(B).max(1) < 0.006)
        N, P = N[g] / n[g, None], Pm[vv[g], uu[g]]
        N[(N * (T[:3, 3] - P)).sum(1) < 0] *= -1
        vert = np.abs(N[:, 2]) < 0.35
        yield d["name"], np.degrees(np.arctan2(N[vert][:, 1], N[vert][:, 0])), N[vert]


def dominant_face_dir(views, data, half=15):
    """Direction (deg, mod 180) in which most near-vertical plug surface faces, and how much it dominates the perpendicular direction. The plug's wide faces carry more pixels than its narrow ones,
    so the peak is the wide-face normal (the jaws close along it) and ratio = pixels near the peak / pixels near the perpendicular. Returns (direction, ratio, pixels at peak, pixels at perpendicular)."""
    ang = np.concatenate([a for _, a, _ in _vertical_normals(views, data)])
    if len(ang) < 50:
        return None, 0.0, 0, 0
    best = max(range(0, 180, 2), key=lambda t: np.sum(np.abs((ang - t + 90) % 180 - 90) < half))
    peak = int(np.sum(np.abs((ang - best + 90) % 180 - 90) < half))
    perp = int(np.sum(np.abs((ang - best) % 180 - 90) < half))
    z = np.exp(2j * np.radians(ang[np.abs((ang - best + 90) % 180 - 90) < half]))
    return float(np.degrees(np.angle(z.mean())) / 2 % 180), peak / max(perp, 1), peak, perp


def wide_face_yaw(views, data, strip_dir, tol=30):
    """Yaw of the plug's wide faces: the normal direction (deg, mod 180) of near-vertical faces whose normal lies within tol deg of strip_dir (the dominant face direction from dominant_face_dir).
    Returns a list of dict(view, side, pixels, yaw); side is '+y' or '-y' by the sign of the mean normal along strip_dir (the two opposite faces)."""
    out = []
    d0 = np.array([np.cos(np.radians(strip_dir)), np.sin(np.radians(strip_dir)), 0])
    for name, ang, N in _vertical_normals(views, data):
        wide = np.abs((ang - strip_dir + 90) % 180 - 90) < tol
        if wide.sum() == 0:
            out.append(dict(view=name, side="none", pixels=0, yaw=None))
            continue
        a = ang[wide]
        out.append(dict(view=name, side="+y" if np.mean(N[wide] @ d0) > 0 else "-y", pixels=int(wide.sum()), yaw=float(np.degrees(np.angle(np.exp(2j * np.radians(a)).mean())) / 2 % 180)))
    return out
