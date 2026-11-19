"""CAD mode figures (matplotlib, no display): model-on-image overlays per view, the fit in side projections with the panel and the best grasp."""
import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _project(P_mm, view):
    c = (P_mm / 1000 - view["T_BC"][:3, 3]) @ view["T_BC"][:3, :3]
    K = view["K"]
    return np.c_[K[0, 0] * c[:, 0] / c[:, 2] + K[0, 2], K[1, 1] * c[:, 1] / c[:, 2] + K[1, 2]], c[:, 2]


def report_cad_fit(out, views, obs, M, T, plane, cands, rep):
    out.mkdir(parents=True, exist_ok=True)
    posed = M @ T[:3, :3].T + T[:3, 3]
    tiles = []
    for i, v in enumerate(views):
        img = v["color"].copy()
        for P, col, step in ((posed, (255, 0, 255), 25), (obs[i], (0, 255, 0), 1)):
            uv, z = _project(P[::step], v)
            for u, w in uv[z > 0].astype(int):
                if 0 <= u < img.shape[1] and 0 <= w < img.shape[0]:
                    cv2.circle(img, (u, w), 1, col, -1)
        uv, _ = _project(posed[::25], v)
        cu, cv_ = np.median(uv, 0).astype(int)
        crop = img[max(0, cv_ - 90):cv_ + 90, max(0, cu - 120):cu + 120]
        cv2.putText(crop, "view %d: %.0f%% held-out within 2 mm" % (i, rep["heldout"][i]["within_pct"]), (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        tiles.append(cv2.resize(crop, (480, 360), interpolation=cv2.INTER_NEAREST))
    cv2.imwrite(str(out / "01_cad_overlay_views.png"), np.hstack(tiles))

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
    c0, best = T[:3, 3], cands[0] if cands else None
    for ax, (i, j, lx) in zip(axes, ((0, 2, "base x (mm)"), (1, 2, "base y (mm)"))):
        for k, P in enumerate(obs):
            ax.scatter(P[:, i], P[:, j], s=5, label="observed, view %d" % k)
        ax.scatter(posed[::8, i], posed[::8, j], s=1.5, c="k", alpha=0.25, label="fitted model")
        lim, o = np.array(ax.get_xlim()), 1 - i
        pz = -(plane["d"] + plane["n"][i] * lim + plane["n"][o] * T[o, 3]) / plane["n"][2]      # the other horizontal axis at the connector
        ax.plot(lim, pz, "r--", lw=1, label="panel plane")
        if best is not None:
            ax.plot(best["p"][i] * 1e3, best["p"][j] * 1e3, "r*", ms=14, label="best grasp (fingertip)")
        ax.set_xlabel(lx); ax.set_ylabel("base z (mm)"); ax.set_aspect("equal"); ax.legend(fontsize=7, loc="upper right")
    fig.suptitle("CAD fit: held-out p80 %.1f mm, tilt %.1f deg, implied seat %.1f mm" % (np.median([h["p80_mm"] for h in rep["heldout"]]), rep["tilt_deg"], rep["implied_seat_u_mm"]))
    fig.tight_layout(); fig.savefig(out / "02_cad_fit_side_views.png", dpi=110); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.bar(range(len(rep["heldout"])), [h["within_pct"] for h in rep["heldout"]], color="tab:blue")
    ax.axhline(70, color="r", ls="--", label="gate")
    ax.set_xlabel("held-out view (fit on the other two)"); ax.set_ylabel("points within 2 mm of the model (%)"); ax.set_ylim(0, 100); ax.legend()
    ax.set_title("Leave-one-view-out check")
    fig.tight_layout(); fig.savefig(out / "03_cad_heldout.png", dpi=110); plt.close(fig)
