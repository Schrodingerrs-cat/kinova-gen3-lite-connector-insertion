"""Where did the planned grasp put the fingers? Overlay on a close camera view + top-down diagram, for a run folder or a plan.json.
Written automatically next to every cad / strip plan (plots/grasp_overlay.png); by hand: python3 tools/grasp_overlay.py RUN_DIR_OR_plan.json [...]
Left: the camera image, the crib outline and its long axis (white-strip detection), the observed plug points and the two finger pads at the planned grasp.
Right: base-frame top view. Red = planned pads and gap, green dashed = the pads a wide-face grasp would have (jaws along the crib's long axis)."""
import json
import pathlib

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parents[1]
from . import strip_config, strip_fit as F
from .config import load_config
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

WORK = HERE.parents[1] / "Media" / "Grasp" / "work"


def box(p, jaw_xy, width, fw, ft, z):
    """The two finger pads (4 corners each, mm, xyz) closed on a gap of `width` mm, centred at p (mm)."""
    a = np.array([-jaw_xy[1], jaw_xy[0]])
    out = []
    for s in (-1, 1):
        c = p[:2] + s * jaw_xy * (width / 2 + ft / 2)
        out.append(np.array([np.r_[c + i * a * fw / 2 + j * jaw_xy * ft / 2, z] for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]))
    return out


def project(P_mm, v):
    c = (P_mm / 1000 - v["T_BC"][:3, 3]) @ v["T_BC"][:3, :3]
    K = v["K"]
    return np.c_[K[0, 0] * c[:, 0] / c[:, 2] + K[0, 2], K[1, 1] * c[:, 1] / c[:, 2] + K[1, 2]]


def overlay(plan_json):
    plan = json.load(open(plan_json))
    if "candidates" not in plan or not plan["candidates"]:
        return None
    gr = load_config(HERE / "config" / "grasp.yaml", allow_unscanned=True)["gripper"]
    fw, ft = gr["finger_width_m"] * 1000, gr["finger_thickness_m"] * 1000
    c = plan["candidates"][0]
    p, jaw = np.array(c["p"]) * 1000, np.array(c["jaw"])[:2]
    jaw = jaw / np.linalg.norm(jaw)
    width = c["width_m"] * 1000
    vdir = next((d for d in (WORK / "cad" / plan.get("views_dir", ""), WORK / "strip" / plan.get("views_dir", "")) if d.is_dir()), None)
    if vdir is None:
        return None
    frames = [dict(np.load(f)) for f in sorted(vdir.glob("view_*.npz"))]
    sc = strip_config.load("usb_a")[0]["strip"]
    region, seed = np.array([180, 580, -320, 320.]), p[:2]
    poses = [F.strip_pose(v, sc, region, seed) for v in frames]
    summ = F.strip_summary(poses)
    cyaw = summ["yaw"] if summ else None
    off = None if cyaw is None else abs((np.degrees(np.arctan2(jaw[1], jaw[0])) - cyaw + 90) % 180 - 90)
    k = int(np.argmax([q["inside"] for q in poses]))   # the view that sees the most crib
    v = frames[k]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 6.2), gridspec_kw=dict(width_ratios=[1.25, 1]))
    img = cv2.cvtColor(v["color"], cv2.COLOR_BGR2RGB)
    a1.imshow(img)
    if summ:
        q = poses[k]
        ring = F_corners(q["centre"], np.array(sc["size_mm"]), q["yaw"])
        pts = project(np.c_[ring, np.full(len(ring), q["top_z"])], v)
        a1.plot(*pts.T, "-", c="yellow", lw=2, label="crib outline (detected), yaw %.0f deg" % q["yaw"])
        c0 = project(np.array([[*q["centre"], q["top_z"]]]), v)[0]
        e = project(np.array([[*(q["centre"] + 40 * F.direction(q["yaw"])), q["top_z"]]]), v)[0]
        a1.annotate("", xy=e, xytext=c0, arrowprops=dict(arrowstyle="->", color="yellow", lw=2))
    for pad in box(p, jaw, width, fw, ft, p[2]):
        a1.plot(*project(np.vstack([pad, pad[:1]]), v).T, "-", c="red", lw=2)
    a1.plot(*project(np.array([[*(p[:2] - jaw * width / 2), p[2]], [*(p[:2] + jaw * width / 2), p[2]]]), v).T, "r-", lw=3, label="planned gap %.1f mm" % width)
    a1.set_title("camera view %d with the planned finger pads (red)" % k, fontsize=10)
    a1.legend(fontsize=8, loc="lower left")
    a1.axis("off")
    if summ:
        cr = F_corners(summ["centre"], np.array(sc["size_mm"]), summ["yaw"])
        a2.plot(*np.vstack([cr, cr[:1]]).T, "k-", lw=1.5, label="crib")
        a2.annotate("", xy=summ["centre"] + 50 * F.direction(summ["yaw"]), xytext=summ["centre"], arrowprops=dict(arrowstyle="->", lw=1.5))
        ideal = F.direction(summ["yaw"])
        for pad in box(p, ideal, 4.5, fw, ft, 0):
            a2.plot(*np.vstack([pad, pad[:1]])[:, :2].T, "--", c="green", lw=1.5)
        a2.plot([], [], "--", c="green", label="pads on the wide faces (jaws along the crib's long axis)")
    for pad in box(p, jaw, width, fw, ft, 0):
        a2.plot(*np.vstack([pad, pad[:1]])[:, :2].T, "-", c="red", lw=2)
    a2.plot([], [], "-", c="red", label="planned pads")
    a2.set_aspect("equal")
    a2.set_xlim(p[0] - 70, p[0] + 70)
    a2.set_ylim(p[1] - 70, p[1] + 70)
    a2.set_xlabel("base x (mm)")
    a2.set_ylabel("base y (mm)")
    a2.legend(fontsize=7, loc="upper right")
    a2.set_title("top view: jaw is %s from the crib's long axis" % ("n/a" if off is None else "%.0f deg" % off), fontsize=10)
    fig.suptitle("%s: %s, plan %s" % (plan.get("method", "plan"), plan.get("connector", ""), pathlib.Path(plan_json).parent.name), fontsize=11)
    fig.tight_layout()
    out = pathlib.Path(plan_json).parent / "plots"
    out.mkdir(exist_ok=True)
    f = out / "grasp_overlay.png"
    fig.savefig(f, dpi=110)
    plt.close(fig)
    return f, off


def F_corners(centre, size, yaw):
    s = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]]) * size / 2
    return centre + F.from_uv(s, yaw)
