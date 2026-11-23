"""CAD mode: place the connector's model-frame grasp candidates with the fitted pose, then drop the ones the live scene rules out. Offline, no ROS."""
import json

import numpy as np
from scipy.spatial import cKDTree


def load_candidates(spec, root):
    return json.load(open(root / spec["candidates"]))


def scene_points(views, centre_mm, T, M, radius_mm=60.0, voxel_mm=1.5, aligns=None):
    """All depth points within radius of the connector from every view, minus the ones the posed model explains (2 mm): panel, cable, neighbours.
    aligns: per view, a function moving its xy into the frame the fit was made in (crib alignment), or None."""
    from .cad_fit import backproject
    pts = []
    for i, v in enumerate(views):
        P = backproject(v["depth"], np.ones(v["depth"].shape, bool), v["K"], v["T_BC"], 0.004)
        if aligns is not None and aligns[i] is not None:
            P[:, :2] = aligns[i](P[:, :2])
        pts.append(P[np.linalg.norm(P[:, :2] - centre_mm[:2], axis=1) < radius_mm])
    P = np.vstack(pts)
    P = P[np.unique(np.floor(P / voxel_mm).astype(int), axis=0, return_index=True)[1]]
    posed = M @ T[:3, :3].T + T[:3, 3]
    return P[cKDTree(posed).query(P)[0] > 2.0]


def _base_yaw_to_model(R, want):
    """Model jaw yaw psi whose image R @ (cos psi, sin psi, 0) points along the base yaw `want` (rad)."""
    psi = np.radians(np.arange(0, 360, 1.0))
    v = R[:2, :2] @ np.array([np.cos(psi), np.sin(psi)]) + np.outer(R[:2, 2], 0)
    d = np.abs((np.arctan2(v[1], v[0]) - want + np.pi) % (2 * np.pi) - np.pi)
    return psi[np.argmin(d)]


def place(T, cands, spec, pc, plane, scene, jaw_ref, finger_w, finger_t, margin, max_open):
    """T = model -> base (mm). jaw_ref = base yaw (rad) of the jaw axis at home. Returns (candidates best first, stats).
    Candidate: dict(p (m, fingertip centre), jaw (horizontal-ish unit), approach (unit, down the plug axis), width_m, open_width_m, offset_m, cost, yaw_deg, u0_mm)."""
    R, t = T[:3, :3], T[:3, 3]
    st = dict(considered=0, no_fit=0, panel=0, scene=0)
    if spec["symmetry"] == "axial":                      # same grasp at any yaw about the axis: make the yaws the arm wants (like the head mode)
        offs = pc["jaw_yaw_offsets_deg"]
        cands = [dict(c, yaw_deg=float(np.degrees(_base_yaw_to_model(R, jaw_ref + np.radians(o))))) for c in cands for o in offs]
    out, z_g = [], -R[:, 2]
    want = spec.get("jaw_model_yaw_deg")   # jaws must close along this model direction (pads on the wide faces); other candidates are dropped
    for c in cands:
        st["considered"] += 1
        if want is not None and abs((c["yaw_deg"] - want + 90) % 180 - 90) > pc["jaw_yaw_tol_deg"]:
            st["face"] = st.get("face", 0) + 1
            continue
        r = np.radians(c["yaw_deg"])
        b, a = np.array([np.cos(r), np.sin(r), 0.0]), np.array([-np.sin(r), np.cos(r), 0.0])
        ow = c["gap_mm"] * 1e-3 + 2 * margin
        if ow > max_open:
            st["no_fit"] += 1
            continue
        p = R @ (b * c["mid_mm"] + a * c["s_mm"] + [0, 0, c["u0_mm"]]) + t
        if plane["n"] @ p + plane["d"] < pc["clearance_mm"]:
            st["panel"] += 1
            continue
        y = R @ b
        x = np.cross(y, z_g)
        q = (scene - p) @ np.column_stack([x, y, z_g])   # grasp frame: z = approach (down), y = jaw; fingers extend back (-z) from the tip
        side = np.abs(q[:, 0]) <= finger_w * 500
        hit = side & (np.abs(q[:, 1]) >= ow * 500) & (np.abs(q[:, 1]) <= ow * 500 + finger_t * 1000) & (q[:, 2] >= -pc["box_h_m"] * 1000) & (q[:, 2] <= pc["clearance_mm"])
        if hit.sum() > pc["max_hits"]:
            st["scene"] += 1
            continue
        out.append(dict(p=p * 1e-3, jaw=y, approach=z_g, width_m=c["gap_mm"] * 1e-3, open_width_m=ow, offset_m=abs(c["s_mm"]) * 1e-3, cost=c["cost"],
                        yaw_deg=c["yaw_deg"], u0_mm=c["u0_mm"], hits=int(hit.sum()), tip_above_panel_mm=float(plane["n"] @ p + plane["d"])))
    st["valid"] = len(out)
    return sorted(out, key=lambda d: d["cost"])[: pc["max_candidates"]], st
