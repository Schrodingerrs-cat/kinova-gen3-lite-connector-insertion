"""Plots + metrics for one saved plan frame, regenerated from frame.npz with the CURRENT pipeline.

CLI (no ROS needed):
  python3 -m housing_grasp.report FRAME.npz OUT_DIR [--roi u0 v0 u1 v1]
  python3 -m housing_grasp.report FRAME.npz OUT_DIR --sweep CU CV      # ROI-size invariance of the measurement
The grasp node also calls report() after every /grasp/plan.
"""
import argparse
import json
import pathlib

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

from .cloud import cloud_from_depth  # noqa: E402
from .config import load_config  # noqa: E402
from .grasp import plan_grasp  # noqa: E402

CONFIG = pathlib.Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"
plt.rcParams.update({"font.size": 11, "axes.titlesize": 12})


def run(frame, roi, cfg):
    f = np.load(frame)
    roi = tuple(roi) if roi else tuple(f["roi"])
    raw, m_raw = cloud_from_depth(f["depth"].astype(float), f["K"], f["T_BC"], roi, edge_jump_m=1e9)
    pts, m = cloud_from_depth(f["depth"].astype(float), f["K"], f["T_BC"], roi, edge_jump_m=cfg["cloud"]["edge_jump_m"])
    res = plan_grasp(pts, m, f["T_BC"][:3, 3], cfg)
    return f, roi, raw, res


def window(res):
    c = res["T_BH"][:3, 3] if res["T_BH"] is not None else res["cloud"][:, :2].mean(0)
    return c[0] * 1e3, c[1] * 1e3


def top_axes(ax, cx, cy, half=70):
    ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal"); ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)")


def plot_raw_vs_clean(raw, res, cfg, out):
    plane_c, n = res["plane"]
    cx, cy = window(res)
    fig, axs = plt.subplots(1, 2, figsize=(12, 5.6), sharex=True, sharey=True)
    for ax, p, title in ((axs[0], raw, "Raw depth (%d points)" % len(raw)),
                         (axs[1], res["cloud"], "Cleaned (%d points)" % len(res["cloud"]))):
        h = (p - plane_c) @ n * 1e3
        s = ax.scatter(p[:, 0] * 1e3, p[:, 1] * 1e3, c=np.clip(h, -5, 30), s=1.5, cmap="viridis", vmin=-5, vmax=30)
        ax.set_title(title); top_axes(ax, cx, cy)
    fig.colorbar(s, ax=axs, label="height above fitted table plane (mm)", shrink=0.85)
    fig.suptitle("Point cloud cleaning, top view (cleaned = edge filter + 1 mm voxel grid + outlier removal)", y=1.0)
    fig.savefig(out / "01_raw_vs_clean_topview.png", dpi=110, bbox_inches="tight"); plt.close(fig)


def box_corners(T, L, W):
    c = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [L / 2, W / 2]
    return (T[:3, :3][:2, :2] @ c.T).T + T[:2, 3]


def plot_segmentation(res, cfg, out):
    cx, cy = window(res)
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    p = res["cloud"]
    ax.scatter(p[:, 0] * 1e3, p[:, 1] * 1e3, s=1.5, c="0.78", label="cleaned scene")
    if res["cluster"] is not None:
        c = res["cluster"]; ax.scatter(c[:, 0] * 1e3, c[:, 1] * 1e3, s=3, c="tab:blue", label="segmented object (>2 mm above table)")
    if res["core"] is not None:
        c = res["core"]; ax.scatter(c[:, 0] * 1e3, c[:, 1] * 1e3, s=4, c="tab:orange", label="solid core (housing)")
    if res["T_BH"] is not None:
        L, W, H = res["dims"]
        ax.add_patch(Polygon(box_corners(res["T_BH"], L, W) * 1e3, fill=False, ec="k", lw=2, label="fitted housing box"))
        x = res["T_BH"][:2, 0]
        ax.annotate("", xy=(res["T_BH"][:2, 3] + x * L * 0.75) * 1e3, xytext=(res["T_BH"][:2, 3] - x * L * 0.5) * 1e3,
                    arrowprops=dict(arrowstyle="->", color="k"))
    if res["grasps"]:
        g = res["grasps"][0]; c = g["T_BG"][:2, 3]; y = g["T_BG"][:2, 1]; w = g["open_width_m"] / 2
        ax.plot(*np.array([c - y * w, c + y * w]).T * 1e3, "r-", lw=2.5, label="best grasp: open jaw line")
        ax.plot(*(c * 1e3), "r*", ms=11, label="grasp TCP")
    top_axes(ax, cx, cy, 60)
    ax.set_title("Segmentation and best grasp, top view (arrow: tip -> cable)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=8, markerscale=2)
    fig.savefig(out / "02_segmentation_and_grasp_topview.png", dpi=110, bbox_inches="tight"); plt.close(fig)


def housing_frame_points(res, p):
    T = res["T_BH"]
    return (p - T[:3, 3]) @ T[:3, :3]


def plot_cross_section(res, cfg, out):
    if res["T_BH"] is None or not res["grasps"]:
        return
    L, W, H = res["dims"]; g = res["grasps"][0]; gp = cfg["gripper"]
    c = housing_frame_points(res, res["cluster"])
    sel = np.abs(c[:, 0] - (g["zone_offset_m"] - L / 2)) < 0.004   # slice of the housing at the grasp position
    tcp = housing_frame_points(res, g["T_BG"][:3, 3][None])[0]
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.scatter(c[sel, 1] * 1e3, (c[sel, 2] + H / 2) * 1e3, s=8, c="tab:blue", label="object points, 8 mm slice at grasp")
    ax.axhline(0, color="saddlebrown", lw=2, label="fitted table plane")
    ow, ft, fl = g["open_width_m"], gp["finger_thickness_m"], gp["finger_length_m"]
    for sgn in (1, -1):
        y0 = sgn * ow / 2 if sgn > 0 else -ow / 2 - ft
        ax.add_patch(Rectangle((y0 * 1e3, (tcp[2] + H / 2) * 1e3), ft * 1e3, fl * 1e3, fc="tab:red", alpha=0.35,
                               label="fingers when open (PLACEHOLDER size)" if sgn > 0 else None))
    ax.plot(0, (tcp[2] + H / 2) * 1e3, "r*", ms=14, label="grasp TCP (fingertip line)")
    ax.set_xlim(-ow / 2 * 1e3 - 12, ow / 2 * 1e3 + 12); ax.set_ylim(-2, 45)
    ax.set_aspect("equal"); ax.set_xlabel("across housing width, y (mm)"); ax.set_ylabel("height above table (mm)")
    ax.set_title("Cross-section at the grasp: fingertips vs housing"); ax.legend(fontsize=8, loc="upper right")
    fig.savefig(out / "03_cross_section_at_grasp.png", dpi=110, bbox_inches="tight"); plt.close(fig)


def plot_profile(res, cfg, out):
    if not res["grasps"] or res["T_BH"] is None:
        return
    prof = res["profile"]
    fw = cfg["gripper"]["finger_width_m"]; d0 = res["grasps"][0]["zone_offset_m"]
    fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axs[0].plot(prof["d"] * 1e3, prof["ext"] * 1e3, "o-", ms=3); axs[0].set_ylabel("housing width (mm)")
    axs[1].plot(prof["d"] * 1e3, prof["top"] * 1e3, "o-", ms=3, c="tab:green"); axs[1].set_ylabel("top height (mm)")
    for ax in axs:
        ax.axvspan((d0 - fw / 2) * 1e3, (d0 + fw / 2) * 1e3, color="r", alpha=0.2, label="finger pad window (best grasp)")
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=8); axs[1].set_xlabel("distance from tip along the housing (mm)")
    axs[0].set_title("Housing profile along its long axis (flat, constant regions are preferred for grasping)")
    fig.savefig(out / "04_housing_profile.png", dpi=110, bbox_inches="tight"); plt.close(fig)


def report(frame, out, roi=None, cfg=None):
    """Write out/plots/*.png and out/metrics_pipeline.json for a saved frame. Returns the metrics dict."""
    cfg = cfg or load_config(CONFIG)
    f, roi, raw, res = run(frame, roi, cfg)
    out = pathlib.Path(out)
    (out / "plots").mkdir(parents=True, exist_ok=True)
    plot_raw_vs_clean(raw, res, cfg, out / "plots")
    plot_segmentation(res, cfg, out / "plots")
    plot_cross_section(res, cfg, out / "plots")
    plot_profile(res, cfg, out / "plots")
    r1 = lambda v: [round(float(x) * 1e3, 1) for x in v]
    g = res["grasps"][0] if res["grasps"] else None
    metrics = dict(frame=str(frame), roi_px=[int(v) for v in roi], ok=res["ok"], reasons=res["reasons"],
                   tip_known=res.get("tip_known"), raw_points=len(raw), clean_points=len(res["cloud"]),
                   housing_LWH_mm=None if res["dims"] is None else r1(res["dims"]),
                   housing_center_base_mm=None if res["T_BH"] is None else r1(res["T_BH"][:3, 3]),
                   n_grasps=len(res["grasps"]),
                   best_grasp_tcp_base_mm=None if g is None else r1(g["T_BG"][:3, 3]),
                   best_grasp_open_width_mm=None if g is None else round(g["open_width_m"] * 1e3, 1))
    (out / "metrics_pipeline.json").write_text(json.dumps(metrics, indent=1))
    return metrics


def sweep(frame, cu, cv, out, halves=(30, 50, 70, 100, 130, 160, 200)):
    """Does the measured housing depend on the ROI box size? Two plots + metrics_roi_invariance.json."""
    cfg = load_config(CONFIG)
    f = np.load(frame)
    rows = []
    for h in halves:
        pts, m = cloud_from_depth(f["depth"].astype(float), f["K"], f["T_BC"], (cu - h, cv - h, cu + h, cv + h),
                                  edge_jump_m=cfg["cloud"]["edge_jump_m"])
        r = plan_grasp(pts, m, f["T_BC"][:3, 3], cfg)
        rows.append(dict(roi_half_px=h, ok=r["ok"],
                         dims_mm=None if r["dims"] is None else [round(float(v) * 1e3, 1) for v in r["dims"]],
                         center_mm=None if r["T_BH"] is None else [round(float(v) * 1e3, 1) for v in r["T_BH"][:3, 3]]))
    out = pathlib.Path(out)
    (out / "plots").mkdir(parents=True, exist_ok=True)
    (out / "metrics_roi_invariance.json").write_text(json.dumps(rows, indent=1))
    ok = [r for r in rows if r["ok"]]
    x = [2 * r["roi_half_px"] for r in ok]
    d, c = np.array([r["dims_mm"] for r in ok]), np.array([r["center_mm"] for r in ok])
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for i, name in enumerate(("length", "width", "height")):
        ax.plot(x, d[:, i], "o-", label=name)
    ax.set(xlabel="ROI box side (px)", ylabel="measured housing size (mm)", ylim=(0, 50), title="Measured housing size vs ROI box size")
    ax.grid(alpha=.3); ax.legend()
    fig.savefig(out / "plots/roi_size_vs_dimensions.png", dpi=110, bbox_inches="tight"); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(x, np.linalg.norm(c[:, :2] - c[0, :2], axis=1), "o-")
    ax.set(xlabel="ROI box side (px)", ylabel="housing center shift vs smallest ROI (mm)", ylim=(-0.5, 5), title="Housing center vs ROI box size")
    ax.grid(alpha=.3)
    fig.savefig(out / "plots/roi_size_vs_center_shift.png", dpi=110, bbox_inches="tight"); plt.close(fig)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame"); ap.add_argument("out")
    ap.add_argument("--roi", type=int, nargs=4); ap.add_argument("--sweep", type=int, nargs=2, metavar=("CU", "CV"))
    a = ap.parse_args()
    print(json.dumps(sweep(a.frame, *a.sweep, a.out) if a.sweep else report(a.frame, a.out, a.roi), indent=1))
