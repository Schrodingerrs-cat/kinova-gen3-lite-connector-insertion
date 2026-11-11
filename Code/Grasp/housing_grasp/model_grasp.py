"""Grasp search on a CAD model of the plug standing upright in the panel (offline, no robot).
Model frame (mm): x, y across the plug, u = height above the panel surface. Straight-down approach, pads on the vertical walls.
Candidate = (jaw yaw, pad centre s along the strip, tip height u0). Scores like the head-mode search: antipodal normals, deeper (low u0), more contact points, wider face."""
import numpy as np


def plug_frame(mesh, panel_z_mm):
    """STEP/GLB mesh (metres, plug axis = z, metal tip at +z) -> surface samples in the upright frame: x, y = -y (a proper rotation), u = panel_z - z."""
    import trimesh
    pts, idx = trimesh.sample.sample_surface(mesh, 300000, seed=0)
    n = mesh.face_normals[idx]
    P, N = pts * 1e3, n.copy()
    P = np.c_[P[:, 0], -P[:, 1], panel_z_mm - P[:, 2]]
    N = np.c_[N[:, 0], -N[:, 1], -N[:, 2]]
    return P, N


def search(P, N, pad_w=6.0, pad_len=25.0, u0s=range(3, 13), yaw_step=5, s_step=1.0, band=0.5, w=None):
    """Returns candidates best first: dict(yaw_deg, s_mm, u0_mm, gap_mm, contact, face_mm, anti, cost)."""
    w = w or dict(anti=1.0, depth=0.5, contact=0.5, face=0.5, center=0.5)
    out = []
    for yaw in np.arange(0, 180, yaw_step):
        r = np.radians(yaw)
        b, a = np.array([np.cos(r), np.sin(r)]), np.array([-np.sin(r), np.cos(r)])
        tb, ta = P[:, :2] @ b, P[:, :2] @ a
        for s in np.arange(-8, 8.01, s_step):
            for u0 in u0s:
                win = (np.abs(ta - s) <= pad_w / 2) & (P[:, 2] >= u0) & (P[:, 2] <= u0 + pad_len)
                if win.sum() < 200:
                    continue
                lo, hi = tb[win].min(), tb[win].max()
                left, right = win & (tb < lo + band), win & (tb > hi - band)
                if left.sum() < 20 or right.sum() < 20:
                    continue
                anti = 0.5 * (np.clip(-(N[left, :2] @ b), 0, 1).mean() + np.clip(N[right, :2] @ b, 0, 1).mean())
                fl = []   # wall length along the strip axis, from all points in the same band at the same heights
                for sel in ((tb < lo + band), (tb > hi - band)):
                    m = sel & (P[:, 2] >= u0) & (P[:, 2] <= u0 + pad_len)
                    fl.append(np.ptp(ta[m]) if m.sum() >= 3 else 0.0)
                out.append(dict(yaw_deg=float(yaw), s_mm=float(s), u0_mm=float(u0), gap_mm=float(hi - lo), contact=int(min(left.sum(), right.sum())), face_mm=float(min(fl)), anti=float(anti)))
    if not out:
        return out
    umin, umax, cmax, fmax = min(q["u0_mm"] for q in out), max(q["u0_mm"] for q in out), max(q["contact"] for q in out), max(q["face_mm"] for q in out)
    for q in out:
        q["cost"] = float(w["anti"] * (1 - q["anti"]) + w["depth"] * (q["u0_mm"] - umin) / max(umax - umin, 1e-9) + w["contact"] * (1 - q["contact"] / cmax)
                          + w["face"] * (1 - q["face_mm"] / fmax) + w["center"] * abs(q["s_mm"]) / 10.0)
    return sorted(out, key=lambda q: q["cost"])
