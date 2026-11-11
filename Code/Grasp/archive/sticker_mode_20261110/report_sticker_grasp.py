"""ARCHIVED 2026-11-10: plot of the sticker-mode grasp (was in housing_grasp/color_marker_report.py)."""
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def report_sticker_grasp(out, gp, fp, c, p, jaw, open_w):
    """Top view of the marker-sticker grasp: sticker points, housing footprint axes, jaw line, open fingers."""
    out.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(gp[:, 0] * 1e3, gp[:, 1] * 1e3, s=3, c="limegreen", label="marker sticker points (%d)" % len(gp))
    m = fp["center_xy"] * 1e3
    for v, col, nm, L in ((fp["long_axis"], "k", "housing long axis %.1f mm" % (fp["length_m"] * 1e3), fp["length_m"]), (fp["jaw_axis"], "tab:orange", "jaw axis (across, %.1f mm)" % (fp["width_m"] * 1e3), fp["width_m"])):
        ax.plot([m[0] - v[0] * L * 500, m[0] + v[0] * L * 500], [m[1] - v[1] * L * 500, m[1] + v[1] * L * 500], color=col, lw=2, label=nm)
    for sgn in (1, -1):  # open finger pads
        q = p[:2] * 1e3 + sgn * jaw[:2] * open_w * 500
        ax.plot(q[0], q[1], "s", color="tab:red", ms=9, label="open fingers (%.0f mm)" % (open_w * 1e3) if sgn == 1 else None)
    ax.plot(p[0] * 1e3, p[1] * 1e3, "r*", ms=16, label="TCP (grasp point)")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_title("Marker-sticker grasp, top view (TCP z = %.1f mm)" % (p[2] * 1e3))
    fig.tight_layout(); fig.savefig(out / "01_sticker_grasp_topview.png", dpi=100); plt.close(fig)
