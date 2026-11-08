"""Gripper self-scan: finger size and jaw gap vs gripper position, measured by the wrist camera.

The fingers are rigid in the tool frame, so with the arm over empty table the only things within a few
cm of the tool are the fingers (the table is > 0.2 m from the camera). No calipers needed.
"""
import numpy as np

BOX = dict(x=0.09, y=0.04, z=(-0.02, 0.06))  # tool frame, m


def finger_points(depth_m, K, T_tool_cam):
    """Tool-frame points of whatever is inside the finger box in one depth image."""
    v, u = np.mgrid[0:depth_m.shape[0], 0:depth_m.shape[1]]
    ok = depth_m > 0
    z = depth_m[ok]
    pc = np.stack([(u[ok] - K[0, 2]) * z / K[0, 0], (v[ok] - K[1, 2]) * z / K[1, 1], z], 1)
    pt = pc @ T_tool_cam[:3, :3].T + T_tool_cam[:3, 3]
    keep = (np.abs(pt[:, 0]) < BOX["x"]) & (np.abs(pt[:, 1]) < BOX["y"]) & (pt[:, 2] > BOX["z"][0]) & (pt[:, 2] < BOX["z"][1])
    return pt[keep]


def measure_fingers(points, tip_band_m=0.010, min_points=60):
    """Fingers sit at +-x (jaw axis), tips at max z. Returns gap, tip z, pad thickness, pad width, visible length."""
    if len(points) < min_points:
        raise ValueError("fingers not visible (%d points in the finger box)" % len(points))
    tip_z = np.percentile(points[:, 2], 99)
    band = points[points[:, 2] > tip_z - tip_band_m]
    sides = []
    for sel in (band[:, 0] < 0, band[:, 0] >= 0):
        p = band[sel]
        if len(p) < min_points // 2:
            raise ValueError("one finger is not visible")
        ax = np.abs(p[:, 0])
        sides.append(dict(inner=np.percentile(ax, 2), outer=np.percentile(ax, 98), width=np.ptp(np.percentile(p[:, 1], [2, 98]))))
    return dict(gap_m=sides[0]["inner"] + sides[1]["inner"], tip_z_m=tip_z,
                thickness_m=float(np.mean([s["outer"] - s["inner"] for s in sides])),
                width_m=float(np.mean([s["width"] for s in sides])),
                visible_length_m=float(tip_z - np.percentile(points[:, 2], 1)))


def summarize(scans):
    """scans: list of (gripper_position, measurement dict) -> values for config/gripper_scan.yaml."""
    scans = sorted(scans, key=lambda s: s[0])
    return dict(
        finger_width_m=round(float(np.median([m["width_m"] for _, m in scans])), 4),
        finger_thickness_m=round(float(np.median([m["thickness_m"] for _, m in scans])), 4),
        finger_length_m=round(float(np.median([m["visible_length_m"] for _, m in scans])), 4),
        tip_z_in_tool_m=round(float(np.median([m["tip_z_m"] for _, m in scans])), 4),
        position_to_width_m=[[round(float(p), 3), round(float(m["gap_m"]), 4)] for p, m in scans])


def grasp_held(gripper_position, gp):
    """Fingers stopped before the fully closed position -> something is between them.

    This cannot tell a thick housing from thin prongs; the width table gives a second cue
    (see the node log) and a wrist-camera check is still to be added.
    """
    return gripper_position < gp["empty_frac"] * gp["close_cmd"]
