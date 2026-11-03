"""Task-constrained antipodal grasps on the housing's flat side faces.

Gripper closes across the housing width, approaches along -z_H (top-down), and
sits inside an approved zone measured from the tip (clear of strain relief / pins).
Grasp frame G (= gripper TCP, between fingertip pads): z_G = approach (into object),
y_G = closing axis, x_G = y_G x z_G.
"""
import numpy as np

from .perception import housing_pose


def _pose(R, p):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    return T


def _finger_boxes(T_BG, open_w, gp, standoff):
    """Two finger swept volumes (G-frame boxes) in the same frame as T_BG.

    Returns list of (center, R, half_extents).
    """
    fw, ft = gp["finger_width_m"], gp["finger_thickness_m"]
    below, length = gp["finger_tip_below_tcp_m"], gp["finger_length_m"]
    z_lo, z_hi = -below, length + standoff  # column swept while descending
    half = np.array([fw / 2, ft / 2, (z_hi - z_lo) / 2])
    boxes = []
    for sgn in (+1, -1):
        c_local = np.array([0, sgn * (open_w / 2 + ft / 2), (z_hi + z_lo) / 2])
        # z up in G = back along approach, so flip sign of z offsets
        c_local[2] = -c_local[2]
        boxes.append((T_BG[:3, :3] @ c_local + T_BG[:3, 3], T_BG[:3, :3], half))
    return boxes


def _points_in_box(points, box):
    c, R, half = box
    return (np.abs((points - c) @ R) <= half).all(1)


def candidates(T_BH, cfg):
    """All geometric grasp candidates (before collision checks), in base frame."""
    hs, gp = cfg["housing"], cfg["gripper"]
    z_h = T_BH[:3, 2]
    zone_lo, zone_hi = hs["grasp_zone_from_tip_m"]
    xs = np.linspace(zone_lo, zone_hi, cfg["grasp"]["n_offsets"])
    mid = (zone_lo + zone_hi) / 2
    z_G = -z_h
    out = []
    for d in xs:
        # housing x runs tip->cable from -L/2, so the zone offset d from the tip is:
        x_loc = d - hs["length_m"] / 2
        # contact height: fraction of housing height, measured from the bottom face
        z_loc = -hs["height_m"] / 2 + cfg["grasp"]["contact_height_frac"] * hs["height_m"]
        p = T_BH[:3, :3] @ np.array([x_loc, 0, z_loc]) + T_BH[:3, 3]
        for flip in (+1, -1):
            y_G = flip * T_BH[:3, 1]
            x_G = np.cross(y_G, z_G)
            out.append(dict(T_BG=_pose(np.stack([x_G, y_G, z_G], 1), p),
                            zone_offset_m=d, flip=flip, off_center=abs(d - mid)))
    return out


def plan_grasp(points_B, in_roi, cam_origin, cfg):
    """Full pipeline. Returns dict with status, reasons, ranked grasps, housing pose."""
    hs, gp, g = cfg["housing"], cfg["gripper"], cfg["grasp"]
    pose = housing_pose(points_B, in_roi, cam_origin, hs)
    res = dict(ok=False, reasons=list(pose["reasons"]), T_BH=pose["T_BH"], grasps=[], rejected=[])
    if not pose["ok"]:
        return res
    open_w = hs["width_m"] + 2 * g["open_margin_m"]
    if open_w > gp["max_open_m"]:
        res["reasons"].append(f"needed opening {open_w*1e3:.0f} mm > gripper {gp['max_open_m']*1e3:.0f} mm")
        return res

    # scene obstacles = everything except points that belong to the housing box itself
    T_HB = np.linalg.inv(pose["T_BH"])
    local = points_B @ T_HB[:3, :3].T + T_HB[:3, 3]
    slack = g["housing_slack_m"]
    in_housing = (np.abs(local) <= np.array([hs["length_m"], hs["width_m"], hs["height_m"]]) / 2 + slack).all(1)
    obstacles = points_B[~in_housing]
    n = pose["plane"][1]
    plane_c = pose["plane"][0]

    for c in candidates(pose["T_BH"], cfg):
        T = c["T_BG"]
        why = None
        fingertip_h = (T[:3, 3] - plane_c) @ n - gp["finger_tip_below_tcp_m"]
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
                 open_width_m=open_w, expected_closed_width_m=hs["width_m"],
                 T_G_housing=np.linalg.inv(T) @ pose["T_BH"])  # in-hand pose to log per pickup
        res["grasps"].append(c)
    res["grasps"].sort(key=lambda c: c["off_center"])
    if not res["grasps"]:
        res["reasons"].append("all grasp candidates rejected")
    res["ok"] = bool(res["grasps"])
    return res


def verify_grasp(closed_width_m, hs, tol_m):
    """Gripper-position cue after closing: width must match the housing, not empty or cable-pinched."""
    err = closed_width_m - hs["width_m"]
    if abs(err) <= tol_m:
        return True, "width matches housing"
    return False, ("gripper closed on nothing" if err < -tol_m else "wider than housing (cable pinched or off-face)")
