"""Synthetic strip scene: ray cast a table, the white strip and plugs (boxes with teal top, dark sides) into depth + colour."""
import numpy as np

from housing_grasp import cad_fit

K = np.array([[420.0, 0, 424], [0, 420.0, 240], [0, 0, 1]])
W, H = 848, 480
TABLE_Z, STRIP_TOP = -0.040, -0.018
HOME_R = np.array([[1.0, 0, 0], [0, -1, 0], [0, 0, -1]])


def box(centre_xy, yaw_deg, size_xy, z0, z1, top_bgr=(190, 170, 30), side_bgr=(30, 30, 30)):
    return dict(c=np.array(centre_xy, float), yaw=np.radians(yaw_deg), s=np.array(size_xy, float), z0=z0, z1=z1, top=top_bgr, side=side_bgr)


def strip(centre=(0.30, 0.0), yaw=20.0):
    return box(centre, yaw, (0.150, 0.051), TABLE_Z, STRIP_TOP, top_bgr=(235, 232, 230), side_bgr=(220, 220, 220))


def plug(centre, yaw, size=(0.015, 0.0075), height=0.045):
    return box(centre, yaw, size, STRIP_TOP, STRIP_TOP + height)


def render(T_BC, boxes):
    v, u = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1).reshape(-1, 3)
    d = d_cam @ T_BC[:3, :3].T
    o = T_BC[:3, 3]
    t_best = np.full(len(d), np.inf)
    col = np.zeros((len(d), 3), np.uint8)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (TABLE_Z - o[2]) / d[:, 2]
        hit = t > 0
        t_best[hit], col[hit] = t[hit], (70, 110, 150)
        for b in boxes:
            c, s_ = np.cos(b["yaw"]), np.sin(b["yaw"])
            R = np.array([[c, -s_], [s_, c]])
            lo = np.array([-b["s"][0] / 2, -b["s"][1] / 2, b["z0"]])
            hi = np.array([b["s"][0] / 2, b["s"][1] / 2, b["z1"]])
            oo = np.r_[R.T @ (o[:2] - b["c"]), o[2]]
            dd = np.c_[d[:, :2] @ R, d[:, 2]]
            t1, t2 = (lo - oo) / dd, (hi - oo) / dd
            tn, tf = np.minimum(t1, t2).max(1), np.maximum(t1, t2).min(1)
            ok = (tn <= tf) & (tn > 0) & (tn < t_best)
            p = oo + dd[ok] * tn[ok, None]
            top = np.abs(p[:, 2] - b["z1"]) < 1e-6
            t_best[ok] = tn[ok]
            col[ok] = np.where(top[:, None], b["top"], b["side"])
    depth = np.where(np.isfinite(t_best), t_best, 0).astype(np.float32).reshape(H, W)
    return dict(depth=depth, color=col.reshape(H, W, 3), K=K, T_BC=T_BC)


def cam(target, az, dist, tilt):
    return cad_fit.close_view_pose(np.array(target, float), az, dist, tilt, HOME_R)


def home_view(boxes, centre=(0.30, 0.0), height=0.45):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = HOME_R, [centre[0], centre[1], height]
    return render(T, boxes)
