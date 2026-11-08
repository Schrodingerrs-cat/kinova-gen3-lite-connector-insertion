"""Task-oriented, uncertainty-aware ranking of the antipodal housing grasps (docs/Strategy.md).

    Q(g)        = (a Q_contact + b Q_insertion + c Q_clearance + d Q_reach) / (a+b+c+d)      each term in [0, 1]
    Q_robust(g) = E_xi[Q(g; xi)] - kappa * std_xi[Q(g; xi)],   xi = housing pose error (dx, dy, yaw)

A sample xi where the jaws would land on the housing (no lateral gap left) or the finger pad would
reach the insertion face counts as a failure with Q = 0. A grasp whose failure rate exceeds
`max_fail_rate` is rejected, not just penalised (hard constraint, Strategy.md section 4).

Insertion enters twice: (1) the pads must leave the insertion face (the tip) free, and (2) the gripper
pose that puts the held connector at the goal, T_WG* = T_WC* T_GC^-1, must be reachable (`reach_fn`,
e.g. an IK check). Without `reach_fn` the reach term is dropped and the weights renormalised.
"""
import numpy as np

from .grasp import _finger_boxes, _points_in_box, _pose


def insertion_gripper_pose(cand, T_WC_goal):
    """Gripper pose that puts the held housing at T_WC_goal: T_WG* = T_WC* T_GC^-1 (T_GC = cand['T_G_housing'])."""
    return T_WC_goal @ np.linalg.inv(cand["T_G_housing"])


def _protrusion(d, L, fw, tip_known, dx=0.0):
    """Distance from the insertion face (tip) to the nearest pad edge. Unknown tip -> worst of both ends."""
    d = d + dx
    return d - fw / 2 if tip_known else min(d, L - d) - fw / 2


def _side_gap(open_w, W, fw, dy, yaw):
    """Free gap per side around the housing when it is off-centre by dy and rotated by yaw under the pad."""
    return (open_w - (W * np.cos(yaw) + fw * abs(np.sin(yaw)))) / 2 - abs(dy)


def _clearance_nominal(c, obstacles, open_w, gp, g, margins):
    """Fraction of inflation margins (m) at which the finger volumes still see no obstacle points."""
    clear = 0
    for m in margins:
        gi = dict(gp, finger_width_m=gp["finger_width_m"] + 2 * m, finger_thickness_m=gp["finger_thickness_m"] + 2 * m)
        hits = sum(_points_in_box(obstacles, b).sum() for b in _finger_boxes(c["T_BG"], open_w, gi, g["standoff_m"]))
        clear += hits <= g["max_collision_points"]
    return clear / len(margins)


def _q(w, t):
    num = sum(w[k] * t[k] for k in t)
    return num / sum(w[k] for k in t)


def score_grasps(res, cfg, opt, T_WC_goal=None, reach_fn=None, rng=None):
    """Score and re-rank res['grasps'] in place. Returns res.

    opt: cfg['optimizer'] (weights, flat_scale, protrusion_target_frac, min_protrusion_m, sigma_*, ...).
    reach_fn(T_gripper_at_insertion) -> reach score in [0, 1] (0 = unreachable / in collision).
    """
    if not res["ok"]:
        return res
    rng = rng or np.random.default_rng(opt.get("seed", 0))
    gp, g = cfg["gripper"], cfg["grasp"]
    L, W, _ = res["dims"]
    fw, tip_known = gp["finger_width_m"], res["tip_known"]
    w = dict(opt["weights"])
    if reach_fn is None:
        w.pop("reach")
    T_HB = np.linalg.inv(res["T_BH"])
    local = res["cloud"] @ T_HB[:3, :3].T + T_HB[:3, 3]
    obstacles = res["cloud"][~(np.abs(local) <= np.array([L, W, res["dims"][2]]) / 2 + g["housing_slack_m"]).all(1)]

    kept = []
    for c in res["grasps"]:
        open_w = c["open_width_m"]
        d = c["zone_offset_m"]
        prot = _protrusion(d, L, fw, tip_known)
        t = dict(contact=float(np.exp(-c["flat"] / opt["flat_scale"])),
                 insertion=float(np.clip(prot / (opt["protrusion_target_frac"] * L), 0, 1)),
                 clearance=_clearance_nominal(c, obstacles, open_w, gp, g, opt["clearance_margins_m"]))
        if reach_fn is not None:
            t["reach"] = float(np.clip(reach_fn(insertion_gripper_pose(c, T_WC_goal)) if T_WC_goal is not None else 1.0, 0, 1))
            if t["reach"] <= 0:
                res["rejected"].append((d, c["flip"], "insertion pose unreachable (T_WG* has no valid IK)"))
                continue
        if prot < opt["min_protrusion_m"]:
            res["rejected"].append((d, c["flip"], f"pads leave only {prot*1e3:.1f} mm of the insertion face free"))
            continue
        c.update(terms=t, Q=float(_q(w, t)))
        kept.append(c)
    kept.sort(key=lambda c: -c["Q"])

    # Monte-Carlo robustness only on the shortlist (Strategy.md: 20-50 samples per shortlisted grasp)
    sig = np.array([opt["sigma_axis_m"], opt["sigma_lateral_m"], np.radians(opt["sigma_yaw_deg"])])
    xi = rng.normal(0, 1, (opt["n_samples"], 3)) * sig
    final = []
    for c in kept[:opt["n_shortlist"]]:
        q = np.empty(len(xi)); fail = 0
        for i, (dx, dy, yaw) in enumerate(xi):
            gap = _side_gap(c["open_width_m"], W, fw, dy, yaw)
            prot = _protrusion(c["zone_offset_m"], L, fw, tip_known, dx)
            if gap < 0 or prot < opt["min_protrusion_m"] / 2:   # jaw lands on the housing / pad reaches the tip
                q[i] = 0.0; fail += 1; continue
            t = dict(c["terms"])
            t["insertion"] = float(np.clip(prot / (opt["protrusion_target_frac"] * L), 0, 1))
            t["clearance"] = min(t["clearance"], float(np.clip(gap / g["open_margin_m"], 0, 1)))
            q[i] = _q(w, t)
        c.update(Q_robust=float(q.mean() - opt["kappa"] * q.std()), fail_rate=fail / len(xi))
        if c["fail_rate"] > opt["max_fail_rate"]:
            res["rejected"].append((c["zone_offset_m"], c["flip"], f"fails in {c['fail_rate']:.0%} of pose-error samples"))
        else:
            final.append(c)
    final.sort(key=lambda c: -c["Q_robust"])
    res["grasps"] = final
    res["ok"] = bool(final)
    if not final:
        res["reasons"].append("no grasp survives scoring (insertion face free / reachable / robust to pose error)")
    return res


# ---- baselines of docs/Strategy.md section 6 (same housing measurement, different selection) -------------

def centroid_grasp(res, cfg):
    """Baseline 1: grasp at the measured housing centre, no keepouts, no scoring."""
    if res["dims"] is None:
        return None
    g = cfg["grasp"]
    T_BH, (L, W, H) = res["T_BH"], res["dims"]
    z_G = -T_BH[:3, 2]
    y_G = T_BH[:3, 1]
    p = T_BH[:3, :3] @ np.array([0, 0, -H / 2 + g["contact_height_frac"] * H]) + T_BH[:3, 3]
    T = _pose(np.stack([np.cross(y_G, z_G), y_G, z_G], 1), p)
    return dict(T_BG=T, zone_offset_m=L / 2, flip=1, open_width_m=W + 2 * g["open_margin_m"],
                T_G_housing=np.linalg.inv(T) @ T_BH)


def select(res, cfg, method):
    """'centroid' | 'antipodal' (best flatness, no task terms) | 'optimizer' (needs score_grasps run on res)."""
    if method == "centroid":
        return centroid_grasp(res, cfg)
    if method == "antipodal":
        return min(res["grasps"], key=lambda c: c["cost"], default=None)
    if method == "optimizer":
        return res["grasps"][0] if res["grasps"] else None
    raise ValueError(method)
