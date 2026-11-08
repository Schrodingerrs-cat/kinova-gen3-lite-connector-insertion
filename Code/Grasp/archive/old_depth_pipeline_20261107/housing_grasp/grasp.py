"""Dimension-free antipodal grasps on the housing's flat side faces.

The jaws close across the measured housing width, approach along -z_H (top-down), and
sit where the housing is flattest and most parallel-sided, away from the tip edge and
from the cable end (strain relief). Grasp frame G (= gripper TCP at the fingertip line):
z_G = approach (into object), y_G = closing axis, x_G = y_G x z_G.
"""
import numpy as np

from .cloud import clean_cloud
from .housing import housing_pose


def _pose(R, p):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    return T


def _finger_boxes(T_BG, open_w, gp, standoff):
    """Two finger swept volumes (G-frame boxes) as (center, R, half_extents) in the frame of T_BG."""
    fw, ft = gp["finger_width_m"], gp["finger_thickness_m"]
    below, length = gp["finger_tip_below_tcp_m"], gp["finger_length_m"]
    z_lo, z_hi = -below, length + standoff  # column swept while descending (up = -z_G)
    half = np.array([fw / 2, ft / 2, (z_hi - z_lo) / 2])
    boxes = []
    for sgn in (+1, -1):
        c_local = np.array([0, sgn * (open_w / 2 + ft / 2), -(z_hi + z_lo) / 2])
        boxes.append((T_BG[:3, :3] @ c_local + T_BG[:3, 3], T_BG[:3, :3], half))
    return boxes


def _points_in_box(points, box):
    c, R, half = box
    return (np.abs((points - c) @ R) <= half).all(1)


def candidates(pose, cfg):
    """Geometric grasp candidates (before collision checks), best flatness first."""
    hs, gp, g = cfg["housing"], cfg["gripper"], cfg["grasp"]
    T_BH, (L, W, H), prof = pose["T_BH"], pose["dims"], pose["profile"]
    fw = gp["finger_width_m"]
    keep = hs["cable_end_keepout_frac"] * L
    # if the tip cannot be told from the cable, keep out of both ends
    if pose["tip_known"]:
        lo, hi = hs["tip_keepout_m"] + fw / 2, L - keep - fw / 2
    else:
        amb = hs["ambiguous_keepout_frac"] * L
        lo, hi = amb + fw / 2, L - amb - fw / 2
    if hi < lo:
        return []
    mid = (lo + hi) / 2
    z_G = -T_BH[:3, 2]
    out = []
    for d in np.linspace(lo, hi, g["n_offsets"]):
        m = np.abs(prof["d"] - d) <= fw / 2
        if m.sum() < 3:
            continue
        # parallel faces -> constant lateral extent; flat top -> constant height
        flat_cost = np.std(prof["ext"][m]) / W + np.std(prof["top"][m]) / H
        y_loc = np.median(prof["yc"][m])
        x_loc = d - L / 2
        z_loc = -H / 2 + g["contact_height_frac"] * H
        p = T_BH[:3, :3] @ np.array([x_loc, y_loc, z_loc]) + T_BH[:3, 3]
        for flip in (+1, -1):
            y_G = flip * T_BH[:3, 1]
            out.append(dict(T_BG=_pose(np.stack([np.cross(y_G, z_G), y_G, z_G], 1), p),
                            zone_offset_m=d, flip=flip, flat=flat_cost, cost=flat_cost + 0.1 * abs(d - mid) / L))
    return out


def plan_grasp(points_B, in_roi, cam_origin, cfg):
    """Full pipeline. Returns dict with ok, reasons, measured dims, ranked grasps, housing pose."""
    hs, gp, g = cfg["housing"], cfg["gripper"], cfg["grasp"]
    points_B, in_roi = clean_cloud(points_B, in_roi, cfg["cloud"])
    pose = housing_pose(points_B, in_roi, cam_origin, hs)
    res = dict(ok=False, reasons=list(pose["reasons"]), T_BH=pose["T_BH"], grasps=[], rejected=[],
               plane=pose.get("plane"), cluster=pose.get("cluster"), core=pose.get("core_points"), profile=pose.get("profile"), cloud=points_B, dims=pose.get("dims"), tip_known=pose.get("tip_known"))
    if not pose["ok"]:
        return res
    L, W, H = pose["dims"]
    open_w = W + 2 * g["open_margin_m"]
    if open_w > gp["max_open_m"]:
        res["reasons"].append(f"housing {W*1e3:.0f} mm wide needs {open_w*1e3:.0f} mm opening > gripper {gp['max_open_m']*1e3:.0f} mm")
        return res

    # obstacles = everything except points that belong to the housing box itself
    T_HB = np.linalg.inv(pose["T_BH"])
    local = points_B @ T_HB[:3, :3].T + T_HB[:3, 3]
    in_housing = (np.abs(local) <= np.array([L, W, H]) / 2 + g["housing_slack_m"]).all(1)
    obstacles = points_B[~in_housing]
    plane_c, n = pose["plane"]

    cands = candidates(pose, cfg)
    if not cands:
        res["reasons"].append("housing too short for the finger width after tip / cable-end keepouts")
        return res
    for c in sorted(cands, key=lambda c: c["cost"]):
        T = c["T_BG"]
        fingertip_h = (T[:3, 3] - plane_c) @ n - gp["finger_tip_below_tcp_m"]
        why = None
        if fingertip_h < g["plane_clearance_m"]:
            why = f"fingertip {fingertip_h*1e3:.1f} mm above support, need {g['plane_clearance_m']*1e3:.1f}"
        else:
            hits = sum(_points_in_box(obstacles, b).sum()
                       for b in _finger_boxes(T, open_w, gp, g["standoff_m"]))
            if hits > g["max_collision_points"]:
                why = f"{hits} obstacle points in finger volume"
        if why:
            res["rejected"].append((c["zone_offset_m"], c["flip"], why))
            continue
        z_G = T[:3, 2]
        c.update(T_pre=_pose(T[:3, :3], T[:3, 3] - z_G * g["standoff_m"]),
                 T_lift=_pose(T[:3, :3], T[:3, 3] - z_G * g["lift_m"]),
                 open_width_m=open_w, expected_closed_width_m=W,
                 T_G_housing=np.linalg.inv(T) @ pose["T_BH"])  # in-hand pose to log per pickup
        res["grasps"].append(c)
    if not res["grasps"]:
        res["reasons"].append("all grasp candidates rejected")
    res["ok"] = bool(res["grasps"])
    if "optimizer" in cfg:
        from .optimize import score_grasps  # task-aware re-rank (docs/Strategy.md)
        score_grasps(res, cfg, cfg["optimizer"])
    return res


def grasp_held(gripper_position, gp):
    """Fingers stopped before the fully closed position -> something is between them.

    This cannot tell a thick housing from thin prongs; the width table gives a second cue
    (see the node log) and a wrist-camera check is still to be added.
    """
    return gripper_position < gp["empty_frac"] * gp["close_cmd"]
