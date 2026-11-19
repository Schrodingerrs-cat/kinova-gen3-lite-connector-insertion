"""CAD mode: fit the connector's STEP model to depth points of 3 close views. Offline, no ROS. All points mm, base frame.
Better than a free 6-DOF fit on top-down views: the panel plane (measured from the white panel around the plug) fixes the plug axis and height,
the trimmed cost drops the cable, and the fit is validated by leaving one VIEW out (fit on 2, test on the 3rd)."""
import cv2
import numpy as np
import trimesh
from scipy.ndimage import maximum_filter, minimum_filter
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

from .color_marker import head_mask, marker_mask, nearest_blob
from . import connectors
from .connectors import ROOT, marker_cfg

load_config = connectors.load   # the config API of this module (tools and tests use cad_fit.load_config)


def model_cloud(spec, n=40000, u_range=None):
    """Mesh (m) -> model frame (mm), cropped to the part that stands above the panel (fit_u_mm). Returns points, normals."""
    mesh_path = ROOT / spec["mesh"]
    if not mesh_path.exists():
        import cascadio
        cascadio.step_to_glb(str(ROOT / spec["step"]), str(mesh_path))
    sc = trimesh.load(mesh_path)
    mesh = sc.to_geometry() if hasattr(sc, "to_geometry") else sc
    P, f = trimesh.sample.sample_surface(mesh, n, seed=0)
    A, b = np.array(spec["mesh_to_model"]["A"], float), np.array(spec["mesh_to_model"]["b_mm"], float)
    M, N = P @ A.T * 1000 + b, mesh.face_normals[f] @ A.T
    u0, u1 = u_range or spec["fit_u_mm"]
    keep = (M[:, 2] >= u0) & (M[:, 2] <= u1)
    return M[keep], N[keep]


def backproject(depth, mask, K, T_BC, jump_m=0.006):
    valid = np.isfinite(depth) & (depth > 0)
    hi, lo = maximum_filter(np.where(valid, depth, 0.0), 3), minimum_filter(np.where(valid, depth, np.inf), 3)
    valid &= (hi - lo) <= jump_m
    v, u = np.nonzero(valid & mask)
    z = depth[v, u].astype(float)
    return (np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z] @ T_BC[:3, :3].T + T_BC[:3, 3]) * 1000


def connector_points(view, spec, g, hint_m, jump_m=0.006):
    """Sticker blob nearest the hint -> its centre -> dark + sticker pixels connected to it within the connector radius -> 3D points (mm)."""
    color, depth, K, T = view["color"], view["depth"], view["K"], view["T_BC"]
    g = marker_cfg(g, spec)
    sticker = backproject(depth, nearest_blob(marker_mask(color, g), K, T, hint_m), K, T, jump_m)
    if len(sticker) < 15:
        raise RuntimeError("sticker not found near the target (%d depth points)" % len(sticker))
    c = np.median(sticker, 0) / 1000
    g2 = dict(g, dark_v=spec["dark_v"], footprint_radius_m=spec["roi_radius_mm"] / 1000)
    pts = backproject(depth, head_mask(color, K, T, c, g2)[0], K, T, jump_m)
    pts = pts[np.linalg.norm(pts - np.median(pts, 0), axis=1) < 40]
    return pts, sticker


def panel_plane(views, center_mm, pc, z_below):
    """White panel pixels in a ring around the connector -> plane (n up, d) in mm: n.p + d = 0. RANSAC over near-horizontal planes that cross the
    connector axis 0..pc['below_mm'] below z_below (the lowest connector point): the white plate behind the panel and tape cannot win."""
    P = []
    for v in views:
        hsv = cv2.cvtColor(v["color"], cv2.COLOR_BGR2HSV)
        Q = backproject(v["depth"], (hsv[..., 1] < 70) & (hsv[..., 2] > 130), v["K"], v["T_BC"], 0.004)
        r = np.linalg.norm(Q[:, :2] - center_mm[:2], axis=1)
        P.append(Q[(r > pc["ring_mm"][0]) & (r < pc["ring_mm"][1])])
    P = np.vstack(P)
    if len(P) < pc["min_points"]:
        raise RuntimeError("panel plane: only %d white points around the connector (need %d)" % (len(P), pc["min_points"]))
    rng, best = np.random.default_rng(0), None
    for _ in range(1500):
        q = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(q[1] - q[0], q[2] - q[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n = n / np.linalg.norm(n) * (1 if n[2] > 0 else -1)
        if np.degrees(np.arccos(n[2])) > pc["max_tilt_deg"]:
            continue
        d = -n @ q[0]
        zc = -(d + n[:2] @ center_mm[:2]) / n[2]
        if not z_below - pc["below_mm"] <= zc <= z_below + 3.0:
            continue
        inl = np.abs(P @ n + d) < pc["tol_mm"]
        if best is None or inl.sum() > best[0].sum():
            best = (inl, n, d)
    if best is None or best[0].sum() < pc["min_inliers"]:
        raise RuntimeError("no panel plane found near the connector (white points %d)" % len(P))
    inl, n = best[0], best[1]
    c = P[inl].mean(0)
    n2 = np.linalg.svd(P[inl] - c, full_matrices=False)[2][2]
    n2 = n2 if n2[2] > 0 else -n2
    if np.degrees(np.arccos(n2[2])) <= pc["max_tilt_deg"]:    # the refit may not drift past the tilt limit (it mixes in the plate otherwise)
        n = n2
    d = float(-np.median(P[inl] @ n))
    return dict(n=n, d=d, tilt_deg=float(np.degrees(np.arccos(n[2]))), inliers=int(inl.sum()), points=int(len(P)))


def _rn(n):
    """Rotation taking z to the unit vector n."""
    v = np.cross([0.0, 0.0, 1.0], n)
    s, c = np.linalg.norm(v), n[2]
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def _pose(x, plane, tilt):
    """x = [a1, a2, h, psi (, rx, ry)] -> (R, t_mm): axis = panel normal (+ small tilt), t on the plane + h along n."""
    n = plane["n"]
    e1 = np.cross(n, [1.0, 0.0, 0.0]); e1 /= np.linalg.norm(e1)
    R = _rn(n) @ (Rot.from_euler("xy", x[4:6]).as_matrix() if tilt else np.eye(3)) @ Rot.from_euler("z", x[3]).as_matrix()
    return R, -plane["d"] * n + x[0] * e1 + x[1] * np.cross(n, e1) + x[2] * n


def close_view_pose(target, az_deg, dist, tilt_deg, R_home_cam):
    """Camera pose T_BC (m) looking at `target` from distance `dist`, tilted `tilt_deg` from straight down, at azimuth az_deg; smallest turn from the home camera orientation."""
    a, th = np.radians(az_deg), np.radians(tilt_deg)
    cam = np.asarray(target, float) + dist * np.array([np.sin(th) * np.cos(a), np.sin(th) * np.sin(a), np.cos(th)])
    e0, ez = R_home_cam[:, 2], (np.asarray(target, float) - cam) / dist
    ax = np.cross(e0, ez)
    R = Rot.from_rotvec(ax / np.linalg.norm(ax) * np.arccos(np.clip(e0 @ ez, -1, 1))).as_matrix() @ R_home_cam if np.linalg.norm(ax) > 1e-9 else R_home_cam
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cam
    return T


def residual(pts, tree, R, t):
    return tree.query((pts - t) @ R)[0]


def _solve(obs, tree, plane, x0, h0, trim, tilt, free_psi, tilt_max):
    """Trimmed robust fit from x0: keep the best `trim` of the points, re-pick, solve (3 rounds)."""
    free = [0, 1, 2] + ([3] if free_psi else []) + ([4, 5] if tilt else [])
    x = x0.copy()
    lo = np.full(len(x), -np.inf); hi = np.full(len(x), np.inf)
    lo[4:6], hi[4:6] = -np.radians(tilt_max), np.radians(tilt_max)

    def full(z):
        y = x.copy(); y[free] = z; return y
    for _ in range(3):
        R, t = _pose(x, plane, tilt)
        d = residual(obs, tree, R, t)
        keep = d <= np.quantile(d, trim)
        f = lambda z: np.r_[residual(obs[keep], tree, *_pose(full(z), plane, tilt)), 0.5 * (full(z)[2] - h0)]
        s = least_squares(f, x[free], loss="soft_l1", f_scale=1.5, bounds=(lo[free], hi[free]), max_nfev=60, diff_step=1e-3)
        x = full(s.x)
    R, t = _pose(x, plane, tilt)
    d = residual(obs, tree, R, t)
    return dict(x=x, R=R, t=t, score=float(np.mean(np.sort(d)[: int(trim * len(d))] ** 2)))


def fit_pose(obs, tree, spec, plane, fc, tilt=False, start=None):
    """Best pose over yaw starts (one start for an axially symmetric connector). obs = Nx3 mm."""
    h0 = -spec["seat_u_mm"]
    axial = spec["symmetry"] == "axial"
    if start is not None:
        return _solve(obs, tree, plane, start["x"], h0, fc["trim"], tilt, not axial, fc["tilt_max_deg"])
    R0, t0 = _pose(np.array([0.0, 0.0, h0, 0.0]), plane, False)
    c = np.median(obs, 0) - t0
    e1 = np.cross(plane["n"], [1.0, 0.0, 0.0]); e1 /= np.linalg.norm(e1)
    a = [c @ e1, c @ np.cross(plane["n"], e1)]
    runs = []
    for psi in ([0.0] if axial else np.radians(np.arange(0, 360, fc["yaw_step_deg"]))):
        runs.append(_solve(obs, tree, plane, np.array([a[0], a[1], h0, psi, 0.0, 0.0]), h0, fc["trim"], tilt, not axial, fc["tilt_max_deg"]))
    runs.sort(key=lambda r: r["score"])
    return runs[0] | dict(near=[r for r in runs if r["score"] <= 1.2 * runs[0]["score"] + 1e-6])


def _stats(d, within):
    return dict(points=int(len(d)), median_mm=float(np.median(d)), p80_mm=float(np.percentile(d, 80)), p95_mm=float(np.percentile(d, 95)), within_pct=float(100 * np.mean(d < within)))


def fit_views(obs_views, M, spec, plane, cfg):
    """obs_views: list of Nx3 (mm). Fits all views, then leave-one-view-out. Returns report (pose of the best model, per-view held-out stats, gate)."""
    fc, rng = cfg["fit"], np.random.default_rng(0)
    obs = [o if len(o) <= fc["train_per_view"] else o[rng.choice(len(o), fc["train_per_view"], replace=False)] for o in obs_views]
    tree = cKDTree(M)
    ref = np.array([0.0, 0.0, 0.5 * sum(spec["fit_u_mm"])])   # model point used to compare poses (middle of the visible part)
    out, seated = {}, None
    for tilt in (False, True):
        allfit = fit_pose(np.vstack(obs), tree, spec, plane, fc, tilt, start=seated)      # the free-tilt fit starts from the seated one
        seated = allfit if not tilt else seated
        held = []
        for i in range(len(obs)):
            rest = np.vstack([o for j, o in enumerate(obs) if j != i])
            f = fit_pose(rest, tree, spec, plane, fc, tilt, start=allfit)
            held.append(dict(fit=f, stats=_stats(residual(obs_views[i], tree, f["R"], f["t"]), fc["gate"]["within_mm"])))
        axes = np.array([allfit["R"][:, 2]] + [h["fit"]["R"][:, 2] for h in held])
        pts = np.array([allfit["R"] @ ref + allfit["t"]] + [h["fit"]["R"] @ ref + h["fit"]["t"] for h in held])
        out[tilt] = dict(fit=allfit, held=[h["stats"] for h in held],
                         pos_spread_mm=float(np.linalg.norm(pts[:, None] - pts[None], axis=2).max()),
                         axis_spread_deg=float(np.degrees(np.arccos(np.clip(axes @ axes.T, -1, 1))).max()),
                         tilt_deg=float(np.degrees(np.arccos(np.clip(allfit["R"][:, 2] @ plane["n"], -1, 1)))),
                         heldout_p80_mm=float(np.median([h["stats"]["p80_mm"] for h in held])))
    a, b = out[False], out[True]
    use_tilt = b["tilt_deg"] >= 3.0 and b["heldout_p80_mm"] <= (1 - fc["tilt_gain"]) * a["heldout_p80_mm"]
    pick = b if use_tilt else a
    R, t = pick["fit"]["R"], pick["fit"]["t"]
    u_axis = -(plane["n"] @ t + plane["d"]) / (plane["n"] @ R[:, 2])     # model u where the axis crosses the measured panel plane
    g, fails = fc["gate"], []
    for i, s in enumerate(pick["held"]):
        if s["p80_mm"] > g["p80_mm"] or s["within_pct"] < g["min_within_pct"]:
            fails.append("held-out view %d: p80 %.1f mm, %.0f%% within %.0f mm" % (i, s["p80_mm"], s["within_pct"], g["within_mm"]))
    if abs(float(u_axis) - spec["seat_u_mm"]) > g["seat_tol_mm"]:
        fails.append("plug sits %.1f mm from where the panel plane says (model seat u %.1f, implied %.1f): bad panel plane or wrong connector" % (abs(u_axis - spec["seat_u_mm"]), spec["seat_u_mm"], u_axis))
    if pick["pos_spread_mm"] > g["pos_spread_mm"]:
        fails.append("pose position differs %.1f mm between leave-one-out fits (limit %.1f)" % (pick["pos_spread_mm"], g["pos_spread_mm"]))
    if pick["axis_spread_deg"] > g["axis_spread_deg"]:
        fails.append("plug axis differs %.1f deg between leave-one-out fits (limit %.1f)" % (pick["axis_spread_deg"], g["axis_spread_deg"]))
    T = np.eye(4); T[:3, :3], T[:3, 3] = R, t
    return dict(T_base_model_mm=T, used_tilt_fit=bool(use_tilt), tilt_deg=pick["tilt_deg"], heldout=pick["held"], pos_spread_mm=pick["pos_spread_mm"],
                axis_spread_deg=pick["axis_spread_deg"], gate_pass=not fails, gate_failures=fails, implied_seat_u_mm=float(u_axis), seat_u_mm=spec["seat_u_mm"],
                seated=dict(heldout_p80_mm=a["heldout_p80_mm"], tilt_deg=a["tilt_deg"]), free_tilt=dict(heldout_p80_mm=b["heldout_p80_mm"], tilt_deg=b["tilt_deg"]),
                yaw_near=[float(np.degrees(r["x"][3])) % 360 for r in pick["fit"].get("near", [])])


def fit_summary(spec, rep, plane):
    """One line of the fit's quality for the service message."""
    return "%s fit: held-out within 2 mm %s %%, p80 %.1f mm, tilt %.1f deg (%s), implied seat %.1f mm (model %.1f), panel tilt %.1f deg" % (
        spec["name"], [round(h["within_pct"]) for h in rep["heldout"]], np.median([h["p80_mm"] for h in rep["heldout"]]), rep["tilt_deg"],
        "free-tilt fit" if rep["used_tilt_fit"] else "seated on the panel", rep["implied_seat_u_mm"], spec["seat_u_mm"], plane["tilt_deg"])
