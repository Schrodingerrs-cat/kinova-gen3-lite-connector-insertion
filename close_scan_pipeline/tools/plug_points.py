"""Shared by plug_box.py: the board-derived prior and the plug-only points per view (same selection as fit_plug_local.py)."""
import pathlib
import sys

import cv2
import numpy as np
import trimesh
import yaml

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board_yaw as by  # noqa: E402

D = pathlib.Path.home() / "kinova/work/setup_cad"


def rz(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1.0]])


def prior(views, part="NAUO49"):
    B = [by.board(v) for v in views]
    yaw_b = float(np.degrees(np.angle(np.exp(2j * np.radians(np.array([b["yaw"] for b in B]))).mean())) / 2 % 180)
    cen_b = np.mean([b["centre"] for b in B], axis=0)
    zb = float(np.mean([b["z"] for b in B]))
    parts = yaml.safe_load(open(D / "parts_index.yaml"))
    strip = [p for p in parts if p["id"] == "NAUO43"][0]
    pp = [p for p in parts if p["id"] == part][0]
    strip_top = trimesh.load(D / "parts" / strip["file"]).bounds[1][2]
    c0 = np.array([0.25, -0.05])
    R0 = rz(yaw_b - 90)
    t_xy = cen_b - (R0[:2, :2] @ (np.array(strip["centre_m"][:2]) - c0) + c0)
    t = np.array([t_xy[0], t_xy[1], zb - strip_top])
    mesh = trimesh.load(D / "parts" / pp["file"])
    mp = trimesh.Trimesh((mesh.vertices - np.r_[c0, 0]) @ R0.T + np.r_[c0, 0] + t, mesh.faces)
    return dict(board_yaw=yaw_b, board_centre=cen_b, board_top=zb, plug_centre=np.r_[mp.bounds.mean(axis=0)[:2], 0.0], plug_top=float(mp.bounds[1][2]))


def points(views, pr, radius=0.020, max_pts=1500, top_tol=0.006, open_mm=None, mode="teal_or_dark"):
    out = []
    for v in views:
        f = np.load(v)
        d, c, K, T = f["depth"].astype(np.float32), f["color"], f["K"], f["T_BC"]
        med = cv2.medianBlur(d, 5)
        ok = (d > 0.08) & (d < 0.8) & (np.abs(d - med) < 0.004)
        hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        teal = (hsv[..., 0] >= 80) & (hsv[..., 0] <= 105) & (hsv[..., 1] >= 70) & (hsv[..., 2] >= 35) & (hsv[..., 2] <= 140)
        if mode == "sticker":
            bright = ((hsv[..., 0] >= 75) & (hsv[..., 0] <= 105) & (hsv[..., 1] >= 100) & (hsv[..., 2] >= 80)).astype(np.uint8)
            cand = (cv2.morphologyEx(bright, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0) & ok
        else:
            cand = (teal | (hsv[..., 2] < 90)) & ok
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


def wide_face_yaw(views, data, strip_dir, tol=30):
    """Yaw of the plug's wide (+-y) faces from the surface normals of the plug pixels. Returns a list of dict(view, side, pixels, yaw): the normal direction (deg, mod 180) of near-vertical faces
    whose normal lies within tol deg of the board strip direction; side is '+y' or '-y' by the mean normal direction (left or right face)."""
    out = []
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
        ang = np.degrees(np.arctan2(N[:, 1], N[:, 0]))
        wide = (np.abs(N[:, 2]) < 0.35) & (np.abs((ang - strip_dir + 90) % 180 - 90) < tol)
        if wide.sum() == 0:
            out.append(dict(view=d["name"], side="none", pixels=0, yaw=None))
            continue
        a = ang[wide]
        out.append(dict(view=d["name"], side="+y" if np.mean(N[wide][:, 1]) > 0 else "-y", pixels=int(wide.sum()), yaw=float(np.degrees(np.angle(np.exp(2j * np.radians(a)).mean())) / 2 % 180)))
    return out
