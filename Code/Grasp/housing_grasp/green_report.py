"""Plots of the merged-scan green stages (matplotlib, no display needed)."""
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _near(P, c, r, frac=1.0):
    P = P[np.linalg.norm(P[:, :2] - c[:2], axis=1) < r]
    return P[np.random.default_rng(0).random(len(P)) < frac]


def report_green(out, P, r):
    """Writes 01_table_removal.png, 02_clusters_topview.png, 03_green_patch_frame.png into out."""
    out.mkdir(parents=True, exist_ok=True)
    f = r["frame"]
    c = f["centroid"]
    n, d = r["table_n"], r["table_d"]
    A = _near(P, c, 0.12, 0.2)
    above = A @ n + d > 0.003

    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    for a, i, t in ((ax[0], 0, "x vs z"), (ax[1], 1, "y vs z")):
        a.scatter(A[~above, i] * 1e3, A[~above, 2] * 1e3, s=0.5, c="0.65", label="table (RANSAC inliers and below)")
        a.scatter(A[above, i] * 1e3, A[above, 2] * 1e3, s=0.5, c="tab:blue", label="above the table plane")
        a.set_xlabel("xy"[i] + " (mm)"); a.set_ylabel("z (mm)"); a.set_title("Side view " + t); a.legend(markerscale=10, fontsize=8)
    fig.suptitle("Table removal: RANSAC plane, tilt %.1f deg, %d inliers (merged 15-view scan)" % (np.degrees(np.arccos(n[2])), r["table_inliers"]))
    fig.tight_layout(); fig.savefig(out / "01_table_removal.png", dpi=100); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 7))
    O, lab = r["object_points"], r["object_labels"]
    m = np.linalg.norm(O[:, :2] - c[:2], axis=1) < 0.15
    ax.scatter(O[m, 0] * 1e3, O[m, 1] * 1e3, s=2, c=lab[m], cmap="tab20", label="above-table clusters (DBSCAN)")
    for k, C in enumerate(r["green_clusters"][:3]):
        ax.scatter(C[:, 0] * 1e3, C[:, 1] * 1e3, s=2, c="lime", edgecolors="none", label="green cluster %d (%d pts)" % (k, len(C)) if k == 0 else None)
    ax.plot(c[0] * 1e3, c[1] * 1e3, "r*", ms=16, label="target = green centroid")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Above-table clusters and the green marker, top view")
    fig.tight_layout(); fig.savefig(out / "02_clusters_topview.png", dpi=100); plt.close(fig)

    C = r["green_clusters"][0]
    fig, ax = plt.subplots(1, 3, figsize=(15, 5))
    for a, (i, j, t) in zip(ax, ((0, 1, "top: x vs y"), (0, 2, "side: x vs z"), (1, 2, "side: y vs z"))):
        a.scatter(C[:, i] * 1e3, C[:, j] * 1e3, s=1.5, c="limegreen")
        for v, col, nm, L in ((f["long_axis"], "k", "long", 0.02), (f["jaw_axis"], "tab:orange", "jaw", 0.02), (f["approach"], "r", "approach", 0.03)):
            a.annotate("", xy=((c[i] + v[i] * L) * 1e3, (c[j] + v[j] * L) * 1e3), xytext=(c[i] * 1e3, c[j] * 1e3), arrowprops=dict(arrowstyle="->", color=col, lw=2))
            a.plot([], [], color=col, label=nm)
        a.set_xlabel("xyz"[i] + " (mm)"); a.set_ylabel("xyz"[j] + " (mm)"); a.set_aspect("equal"); a.set_title(t); a.legend(fontsize=8)
    fig.suptitle("Green patch (merged scan): %.0f x %.0f x %.0f mm, centroid (%.0f, %.0f, %.0f) mm" % (
        f["length_m"] * 1e3, f["width_m"] * 1e3, f["thickness_m"] * 1e3, *(c * 1e3)))
    fig.tight_layout(); fig.savefig(out / "03_green_patch_frame.png", dpi=100); plt.close(fig)


def report_green_grasp(out, gp, fp, c, p, jaw, open_w):
    """Top view of the green-sticker grasp: sticker points, housing footprint axes, jaw line, open fingers."""
    out.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(gp[:, 0] * 1e3, gp[:, 1] * 1e3, s=3, c="limegreen", label="green sticker points (%d)" % len(gp))
    m = fp["center_xy"] * 1e3
    for v, col, nm, L in ((fp["long_axis"], "k", "housing long axis %.1f mm" % (fp["length_m"] * 1e3), fp["length_m"]), (fp["jaw_axis"], "tab:orange", "jaw axis (across, %.1f mm)" % (fp["width_m"] * 1e3), fp["width_m"])):
        ax.plot([m[0] - v[0] * L * 500, m[0] + v[0] * L * 500], [m[1] - v[1] * L * 500, m[1] + v[1] * L * 500], color=col, lw=2, label=nm)
    for sgn in (1, -1):  # open finger pads
        q = p[:2] * 1e3 + sgn * jaw[:2] * open_w * 500
        ax.plot(q[0], q[1], "s", color="tab:red", ms=9, label="open fingers (%.0f mm)" % (open_w * 1e3) if sgn == 1 else None)
    ax.plot(p[0] * 1e3, p[1] * 1e3, "r*", ms=16, label="TCP (grasp point)")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_title("Green-sticker grasp, top view (TCP z = %.1f mm)" % (p[2] * 1e3))
    fig.tight_layout(); fig.savefig(out / "01_green_sticker_grasp_topview.png", dpi=100); plt.close(fig)


def report_head_grasp(out, P, G, cands):
    """Image mode, top view: head-only cloud (image mask lifted with its own depth), green sticker points, candidate jaw lines (best in red)."""
    out.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.scatter(P[:, 0] * 1e3, P[:, 1] * 1e3, s=4, c=P[:, 2] * 1e3, cmap="viridis", label="head pixels lifted with their own depth (%d), colour = z" % len(P))
    ax.scatter(G[:, 0] * 1e3, G[:, 1] * 1e3, s=2, c="limegreen", label="green sticker (%d)" % len(G))
    for k, q in enumerate(cands):
        a, b = q["p"][:2] * 1e3 - q["jaw"][:2] * q["open_width_m"] * 500, q["p"][:2] * 1e3 + q["jaw"][:2] * q["open_width_m"] * 500
        ax.plot([a[0], b[0]], [a[1], b[1]], color="red" if k == 0 else "orange", lw=2 if k == 0 else 1, label="candidate %d: opening %.0f mm, TCP z %.1f mm" % (k, q["open_width_m"] * 1e3, q["p"][2] * 1e3))
        ax.annotate(str(k), q["p"][:2] * 1e3, color="k", fontsize=11, xytext=(4, 4), textcoords="offset points")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=7, loc="upper right")
    ax.set_title("Image mode: head segmented in the image, grasp candidates on its cloud")
    fig.tight_layout(); fig.savefig(out / "01_head_image_mode_grasp_topview.png", dpi=100); plt.close(fig)


def report_cloud_candidates(out, G, rect, cands, top=8):
    """Cloud mode, top view: sticker points, the image-based sticker rectangle, the best candidates as jaw lines (best in red)."""
    out.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.scatter(G[:, 0] * 1e3, G[:, 1] * 1e3, s=3, c="limegreen", label="green sticker points, 3D (%d)" % len(G))
    cx, la, sa = rect["center_xy"], rect["long_axis"], rect["short_axis"]
    box = np.array([cx + s1 * la * rect["long_m"] / 2 + s2 * sa * rect["short_m"] / 2 for s1, s2 in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))]) * 1e3
    ax.plot(box[:, 0], box[:, 1], "k-", lw=2, label="sticker rectangle from the image: %.1f x %.1f mm" % (rect["long_m"] * 1e3, rect["short_m"] * 1e3))
    for k, q in enumerate(cands[:top]):
        a, b = q["p"][:2] * 1e3 - q["jaw"][:2] * q["open_width_m"] * 500, q["p"][:2] * 1e3 + q["jaw"][:2] * q["open_width_m"] * 500
        ax.plot([a[0], b[0]], [a[1], b[1]], color="red" if k == 0 else "orange", lw=2 if k == 0 else 1, alpha=1 if k == 0 else 0.6,
                label="best: %s edge, yaw %+.0f, opening %.0f mm" % (q["edge"], q["yaw_deg"], q["open_width_m"] * 1e3) if k == 0 else None)
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=7, loc="upper right")
    ax.set_title("Cloud mode: candidates searched on the green region (%d candidates, top %d drawn)" % (len(cands), min(top, len(cands))))
    fig.tight_layout(); fig.savefig(out / "01_green_cloud_candidates_topview.png", dpi=100); plt.close(fig)
