"""Marker finding and run-time measurements from the saved views. No CAD: everything here comes from the colour image and depth of the run itself."""
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import board_yaw as by  # noqa: E402

WS = (0.18, 0.58, -0.32, 0.32)   # base-frame x0, x1, y0, y1 where the arm can reach (grasp.yaml workspace.region)


def marker_mask(hsv, spec):
    H, S, V = [hsv[..., i].astype(int) for i in range(3)]
    m = np.zeros(H.shape, bool)
    for lo, hi in spec["hue"]:
        m |= (H >= lo) & (H <= hi)
    m &= (S >= spec["sat"][0]) & (S <= spec["sat"][1]) & (V >= spec["val"][0]) & (V <= spec["val"][1])
    return m


def to_base(d, K, T, vv, uu):
    z = d[vv, uu]
    return np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]


def find_marker(view, spec, near=None, radius=0.04):
    """Largest blob of the marker colour inside the reachable workspace, anywhere in the view. With near=(x, y) the blob closest to that point (within radius).
    Returns dict(P centre xyz in the base frame, px, top = 90th percentile height of its pixels, board_z = median height of the white board pixels in a ring around it). Raises if there is none."""
    f = np.load(view)
    d, c, K, T = f["depth"].astype(np.float32), f["color"], f["K"], f["T_BC"]
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    m = marker_mask(hsv, spec).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        if st[i, 4] < spec["min_px"] or cen[i][1] > 420:    # bottom rows: fingertips of the gripper
            continue
        vv, uu = np.nonzero((lab == i) & (d > 0.08) & (d < 0.8))
        if len(vv) < 0.5 * spec["min_px"]:
            continue
        blob = (lab == i).astype(np.uint8)
        ring = cv2.dilate(blob, np.ones((31, 31), np.uint8)) - cv2.dilate(blob, np.ones((9, 9), np.uint8))
        white = (hsv[..., 1] < 130) & (hsv[..., 2] > 195)
        if white[ring > 0].mean() < 0.2:       # a plug marker sits on the white board
            continue
        P = to_base(d, K, T, vv, uu)
        c3 = np.median(P, axis=0)
        rv, ru = np.nonzero((ring > 0) & white & (d > 0.08) & (d < 0.8))
        board_z = float(np.median(to_base(d, K, T, rv, ru)[:, 2])) if len(rv) > 50 else float("nan")
        if not (WS[0] <= c3[0] <= WS[1] and WS[2] <= c3[1] <= WS[3]):
            continue
        out.append(dict(P=c3, px=int(st[i, 4]), top=float(np.percentile(P[:, 2], 90)), n=len(vv), board_z=board_z))
    if near is not None:
        out = [o for o in out if np.hypot(o["P"][0] - near[0], o["P"][1] - near[1]) <= radius]
        out.sort(key=lambda o: np.hypot(o["P"][0] - near[0], o["P"][1] - near[1]))
    else:
        out.sort(key=lambda o: -o["px"])
    if not out:
        raise RuntimeError("no blob of the marker colour in the view")
    return out[0]


def prior(views, spec):
    """Per-run replacement for the old CAD prior: marker position, board top, board yaw and plug top, all measured from these views. Same keys as plug_points.prior used to return."""
    marks = [find_marker(v, spec) for v in views]
    mc = np.median([m["P"][:2] for m in marks], axis=0)
    B = [by.board(v, mc) for v in views]
    yaw = float(np.degrees(np.angle(np.exp(2j * np.radians(np.array([b["yaw"] for b in B]))).mean())) / 2 % 180)
    return dict(board_yaw=yaw, board_centre=np.mean([b["centre"] for b in B], axis=0), board_top=float(np.nanmedian([m["board_z"] for m in marks])), table_z=float(np.mean([b["table"] for b in B])),
                plug_centre=np.r_[mc, 0.0], plug_top=float(np.median([m["top"] for m in marks])), marker_px=[m["px"] for m in marks])


def run_cloud(views, step=3):
    """All valid depth points of the run's views in the base frame (every `step`-th pixel). This is the collision picture for plugs and cables: nothing from CAD."""
    out = []
    for v in views:
        f = np.load(v)
        d, K, T = f["depth"].astype(np.float32), f["K"], f["T_BC"]
        ok = (d > 0.08) & (d < 0.8) & (np.abs(d - cv2.medianBlur(d, 5)) < 0.004)
        ok[::, :] &= (np.arange(d.shape[0])[:, None] % step == 0) & (np.arange(d.shape[1])[None, :] % step == 0)
        vv, uu = np.nonzero(ok)
        out.append(to_base(d, K, T, vv, uu))
    return np.vstack(out)


def finger_hits(cloud, centre_xy, jaw_dir, open_width, z_tip, reach=0.06, pad=0.012, half_len=0.010):
    """Points of the run cloud inside the two fingers swept straight down onto the grasp: each finger is a box `pad` thick just outside the open jaws, `half_len` either side across the jaws,
    from the fingertip height up `reach`. The target plug sits between the jaws, so it is not in the boxes. Returns the points inside (empty = clear)."""
    u = np.array(jaw_dir[:2], float) / np.linalg.norm(jaw_dir[:2])
    w = np.array([-u[1], u[0]])
    rel = cloud[:, :2] - np.asarray(centre_xy)
    along, across = rel @ u, rel @ w
    inside = (np.abs(along) > open_width / 2) & (np.abs(along) < open_width / 2 + pad) & (np.abs(across) < half_len) & (cloud[:, 2] > z_tip + 0.002) & (cloud[:, 2] < z_tip + reach)
    return cloud[inside]
