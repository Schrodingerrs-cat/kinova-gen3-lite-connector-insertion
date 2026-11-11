"""Plots of the merged-scan colour-marker stages (matplotlib, no display needed)."""
import os

import matplotlib
import mpl_toolkits
import numpy as np

mpl_toolkits.__path__.insert(0, os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)), "mpl_toolkits"))  # the system mpl_toolkits (3.5) shadows the pip matplotlib's; 3D plots need the matching one
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _near(P, c, r, frac=1.0):
    P = P[np.linalg.norm(P[:, :2] - c[:2], axis=1) < r]
    return P[np.random.default_rng(0).random(len(P)) < frac]


def report_color_target(out, P, r):
    """Writes 01_table_removal.png, 02_clusters_topview.png, 03_marker_patch_frame.png into out."""
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
    for k, C in enumerate(r["marker_clusters"][:3]):
        ax.scatter(C[:, 0] * 1e3, C[:, 1] * 1e3, s=2, c="lime", edgecolors="none", label="marker cluster %d (%d pts)" % (k, len(C)) if k == 0 else None)
    ax.plot(c[0] * 1e3, c[1] * 1e3, "r*", ms=16, label="target = marker centroid")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Above-table clusters and the colour marker, top view")
    fig.tight_layout(); fig.savefig(out / "02_clusters_topview.png", dpi=100); plt.close(fig)

    C = r["marker_clusters"][0]
    fig, ax = plt.subplots(1, 3, figsize=(15, 5))
    for a, (i, j, t) in zip(ax, ((0, 1, "top: x vs y"), (0, 2, "side: x vs z"), (1, 2, "side: y vs z"))):
        a.scatter(C[:, i] * 1e3, C[:, j] * 1e3, s=1.5, c="limegreen")
        for v, col, nm, L in ((f["long_axis"], "k", "long", 0.02), (f["jaw_axis"], "tab:orange", "jaw", 0.02), (f["approach"], "r", "approach", 0.03)):
            a.annotate("", xy=((c[i] + v[i] * L) * 1e3, (c[j] + v[j] * L) * 1e3), xytext=(c[i] * 1e3, c[j] * 1e3), arrowprops=dict(arrowstyle="->", color=col, lw=2))
            a.plot([], [], color=col, label=nm)
        a.set_xlabel("xyz"[i] + " (mm)"); a.set_ylabel("xyz"[j] + " (mm)"); a.set_aspect("equal"); a.set_title(t); a.legend(fontsize=8)
    fig.suptitle("Marker patch (merged scan): %.0f x %.0f x %.0f mm, centroid (%.0f, %.0f, %.0f) mm" % (
        f["length_m"] * 1e3, f["width_m"] * 1e3, f["thickness_m"] * 1e3, *(c * 1e3)))
    fig.tight_layout(); fig.savefig(out / "03_marker_patch_frame.png", dpi=100); plt.close(fig)


def report_grid_search(out, P, N3, N2, G, c, cands, stats, finger_w, scene=None, finger_t=0.0077, finger_box_h=0.012):
    """Image mode grid search. Figures (mm): 01 head cloud with 3D normals, 02 head outline normals (top view, used for the pad contacts), 03 every valid candidate
    (jaw line at its centre, colour = cost) with the best opening, 04 grasp points in 3D (object, centroid, grasp points), 05 candidate counts per step, 06 best cost per yaw."""
    from matplotlib.collections import LineCollection
    out.mkdir(parents=True, exist_ok=True)
    best = cands[0]
    jb, pb = best["jaw"][:2], best["p"]
    contacts = np.array([pb[:2] + s * jb * best["width_m"] / 2 for s in (-1, 1)])
    sub = np.arange(0, len(P), max(1, len(P) // 120))
    S = 1e3

    fig = plt.figure(figsize=(8, 7)); ax = fig.add_subplot(projection="3d")
    ax.scatter(*(P.T * S), s=3, c="tab:green", alpha=0.35, label="head cloud (%d points)" % len(P))
    ax.quiver(*(P[sub].T * S), *(N3[sub].T * 4.0), color="tab:blue", linewidth=0.8, label="3D normals (PCA of 15 neighbours, pointing outward)")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)"); ax.legend(fontsize=8, loc="upper left")
    ax.set_title("Head cloud and 3D surface normals (every %dth point drawn)" % (len(P) // len(sub)))
    fig.tight_layout(); fig.savefig(out / "01_head_cloud_normals_3d.png", dpi=110); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    ax.scatter(P[:, 0] * S, P[:, 1] * S, s=4, c="tab:green", alpha=0.35, label="head cloud (top view)")
    ax.quiver(P[sub, 0] * S, P[sub, 1] * S, N2[sub, 0], N2[sub, 1], color="tab:blue", angles="xy", scale=22, width=0.003, label="outline normals (horizontal, outward)")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Head outline normals used to score the pad contacts")
    fig.tight_layout(); fig.savefig(out / "02_head_outline_normals_topview.png", dpi=110); plt.close(fig)

    ctr = np.array([q["p"][:2] for q in cands]) * S
    cost = np.array([q["cost"] for q in cands])
    fig, ax = plt.subplots(figsize=(8, 6.5))
    ax.scatter(P[:, 0] * S, P[:, 1] * S, s=3, c="lightgrey", label="head cloud")
    sc = ax.scatter(ctr[:, 0], ctr[:, 1], s=7, c=cost, cmap="viridis_r", label="candidate jaw centres (%d)" % len(cands))
    plt.colorbar(sc, ax=ax, label="cost (lower is better)", fraction=0.046)
    for q in cands[:60]:   # the best 60 as jaw-gap lines
        h = q["jaw"][:2] * q["width_m"] * S / 2
        ax.plot(*np.stack([q["p"][:2] * S - h, q["p"][:2] * S + h]).T, color="black", lw=0.4, alpha=0.5)
    ax.plot(*contacts.T * S, "-", color="red", lw=2.5, label="best: gap %.1f mm (opening %.1f mm), yaw %.0f deg, %.0f mm deeper" % (best["width_m"] * S, best["open_width_m"] * S, best["yaw_deg"], best["depth_extra_m"] * S))
    ax.plot(*contacts.T * S, "s", color="red", ms=8)
    ax.plot(c[0] * S, c[1] * S, "*", color="gold", mec="k", ms=14, label="sticker median")
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=8, loc="upper right")
    ax.set_title("%d valid grasp candidates from the grid search (best 60 gaps as lines)" % len(cands))
    fig.tight_layout(); fig.savefig(out / "03_grasp_candidates_topview.png", dpi=110); plt.close(fig)

    fig = plt.figure(figsize=(8, 7)); ax = fig.add_subplot(projection="3d")
    ax.scatter(*(P.T * S), s=3, c="tab:green", alpha=0.5, label="object")
    ax.scatter(*(P.mean(0) * S), s=140, c="gold", edgecolors="k", label="centroid")
    gp3 = np.c_[contacts, np.full(2, pb[2])] * S
    ax.scatter(*gp3.T, s=160, c="red", edgecolors="k", label="grasp points (pad contacts of the best candidate)")
    ax.plot(*gp3.T, color="red", lw=1.5)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)"); ax.legend(fontsize=8, loc="upper left")
    ax.set_title("Head cloud, centroid and the best grasp points")
    fig.tight_layout(); fig.savefig(out / "04_grasp_points_3d.png", dpi=110); plt.close(fig)

    steps = [("generated\n(grid x yaw x depth)", stats["generated"]), ("enough points\nin pad window", stats["generated"] - stats["few_points"]),
             ("jaw centred in\nthe head gap", stats["generated"] - stats["few_points"] - stats["centre_off"]),
             ("opening fits\nthe gripper", stats["generated"] - stats["few_points"] - stats["centre_off"] - stats["too_wide"]),
             ("both pads have\ncontact points", stats["generated"] - stats["few_points"] - stats["centre_off"] - stats["too_wide"] - stats["no_contact"]),
             ("fingers clear\nat that depth (valid)", stats["valid"])]
    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(range(len(steps)), [v for _, v in steps], color="tab:blue")
    ax.bar_label(bars, fontsize=9)
    ax.set_xticks(range(len(steps))); ax.set_xticklabels([n for n, _ in steps], fontsize=8); ax.set_ylabel("candidates")
    ax.set_title("Candidates left after each filter")
    fig.tight_layout(); fig.savefig(out / "05_candidate_counts.png", dpi=110); plt.close(fig)

    if scene is not None:   # side view along the best jaw axis: head, scene (panel) points, the two fingertips at every depth tried
        a = np.array([-jb[1], jb[0]])

        def sidev(Q):
            rel = Q[:, :2] - pb[:2]
            keep = np.abs(rel @ a) <= finger_w / 2
            return (rel[keep] @ jb) * 1e3, Q[keep, 2] * 1e3
        hx, hz = sidev(P)
        sx, sz = sidev(scene)
        near = np.abs(sx) < 40
        z_usual = pb[2] + best["depth_extra_m"]   # tip height at the usual depth (sticker height - grasp_drop_m)
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.scatter(sx[near], sz[near], s=3, c="lightgrey", label="non-head scene points (panel, table) under the pads")
        ax.scatter(hx, hz, s=4, c="tab:green", label="head points under the pads")
        for d in sorted({q["depth_extra_m"] for q in cands}):
            ztip = (z_usual - d) * S
            for sgn in (-1, 1):
                x0 = sgn * best["open_width_m"] * S / 2
                ax.plot([x0, x0 + sgn * finger_t * S], [ztip, ztip], color="tab:red", lw=2, alpha=0.9 if abs(d - best["depth_extra_m"]) < 1e-9 else 0.35)
            ax.annotate("+%.0f mm" % (d * S), (best["open_width_m"] * S / 2 + finger_t * S + 0.5, ztip), fontsize=8, va="center")
        ax.axhline(c[2] * S, color="gold", lw=1, ls="--", label="sticker median height")
        ax.set_xlabel("along the jaw axis from the grasp centre (mm)"); ax.set_ylabel("z (mm)"); ax.legend(fontsize=8, loc="upper left")
        ax.set_title("Side view of the best candidate: fingertip height at each depth tried (best = solid red)")
        fig.tight_layout(); fig.savefig(out / "07_depth_levels_side_view.png", dpi=110); plt.close(fig)

    yaws = sorted({q["yaw_deg"] for q in cands})
    ybest = [min(q["cost"] for q in cands if q["yaw_deg"] == y) for y in yaws]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(yaws, ybest, "o-", color="tab:blue", ms=4)
    ax.plot(best["yaw_deg"], best["cost"], "o", color="red", ms=9, label="best candidate (yaw %.0f deg)" % best["yaw_deg"])
    ax.set_xlabel("jaw yaw in the base frame (deg, 0 = along x)"); ax.set_ylabel("lowest cost at that yaw"); ax.legend(fontsize=9)
    ax.set_title("Best cost for every jaw yaw")
    fig.tight_layout(); fig.savefig(out / "06_best_cost_per_yaw.png", dpi=110); plt.close(fig)


def report_cloud_candidates(out, G, rect, cands, top=8):
    """Cloud mode, top view: sticker points, the image-based sticker rectangle, the best candidates as jaw lines (best in red)."""
    out.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.scatter(G[:, 0] * 1e3, G[:, 1] * 1e3, s=3, c="limegreen", label="marker sticker points, 3D (%d)" % len(G))
    cx, la, sa = rect["center_xy"], rect["long_axis"], rect["short_axis"]
    box = np.array([cx + s1 * la * rect["long_m"] / 2 + s2 * sa * rect["short_m"] / 2 for s1, s2 in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))]) * 1e3
    ax.plot(box[:, 0], box[:, 1], "k-", lw=2, label="sticker rectangle from the image: %.1f x %.1f mm" % (rect["long_m"] * 1e3, rect["short_m"] * 1e3))
    for k, q in enumerate(cands[:top]):
        a, b = q["p"][:2] * 1e3 - q["jaw"][:2] * q["open_width_m"] * 500, q["p"][:2] * 1e3 + q["jaw"][:2] * q["open_width_m"] * 500
        ax.plot([a[0], b[0]], [a[1], b[1]], color="red" if k == 0 else "orange", lw=2 if k == 0 else 1, alpha=1 if k == 0 else 0.6,
                label="best: %s edge, yaw %+.0f, opening %.0f mm" % (q["edge"], q["yaw_deg"], q["open_width_m"] * 1e3) if k == 0 else None)
    ax.set_xlabel("base x (mm)"); ax.set_ylabel("base y (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=7, loc="upper right")
    ax.set_title("Cloud mode: candidates searched on the marker region (%d candidates, top %d drawn)" % (len(cands), min(top, len(cands))))
    fig.tight_layout(); fig.savefig(out / "01_marker_cloud_candidates_topview.png", dpi=100); plt.close(fig)
