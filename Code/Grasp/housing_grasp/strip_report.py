"""Strip mode figures: the fitted rectangle on each close view, and the plan view (strip, cables, rectangle, fingers)."""
import cv2
import matplotlib
import numpy as np

from .cad_report import _project
from .strip_fit import direction, from_uv

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _corners(centre, ext, yaw):
    s = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]]) * ext / 2
    return centre + from_uv(s, yaw)


def report_strip_fit(out, frames, strip, obs, fit, finger_w, finger_t):
    out.mkdir(parents=True, exist_ok=True)
    has_rect = True
    tiles = []
    for i, (v, o) in enumerate(zip(frames, obs)):
        img = v["color"].copy()
        for P, col in ((o["pts"][::6], (0, 255, 0)),) + (((np.c_[_corners(fit["centre_mm"], fit["extent_mm"], fit["yaw_deg"]), np.full(5, fit["top_mm"])], (255, 0, 255)),) if has_rect else ()):
            uv, z = _project(P, v)
            pts = uv[z > 0].astype(int)
            if col == (255, 0, 255):
                cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, col, 2)
            else:
                for u, w in pts:
                    if 0 <= u < img.shape[1] and 0 <= w < img.shape[0]:
                        cv2.circle(img, (u, w), 1, col, -1)
        uv, _ = _project(o["sticker"][::10], v)
        cu, cv_ = np.median(uv, 0).astype(int)
        crop = img[max(0, cv_ - 110):cv_ + 110, max(0, cu - 150):cu + 150]
        cv2.putText(crop, ("home" if i == 0 else "view %d" % i) + ": points green, rectangle pink", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        tiles.append(cv2.resize(crop, (600, 440), interpolation=cv2.INTER_NEAREST))
    cv2.imwrite(str(out / "01_strip_overlay_views.png"), np.hstack(tiles))

    fig, ax = plt.subplots(figsize=(8, 6))
    if strip:
        ax.plot(*_corners(strip["centre"], np.array(strip["size"]), strip["yaw"]).T, "k-", lw=1.5, label="white strip (measured)")
    for k, o in enumerate(obs):
        ax.scatter(o["pts"][::4, 0], o["pts"][::4, 1], s=3, label="plug points, %s" % ("home" if k == 0 else "view %d" % k))
    if has_rect:
        ax.plot(*_corners(fit["centre_mm"], fit["extent_mm"], fit["yaw_deg"]).T, "m-", lw=2, label="rectangle at the strip's yaw")
        jaw = direction(fit["jaw_yaw_deg"])
        for sgn in (1, -1):
            c = fit["centre_mm"] + sgn * jaw * (fit["candidate"]["open_width_m"] * 500 + finger_t / 2)
            ax.plot(*_corners(c, np.array([finger_t, finger_w]), fit["jaw_yaw_deg"]).T, "-", c="tab:orange", lw=1.5, label="open fingers" if sgn == 1 else None)
    ax.set_aspect("equal")
    ax.set_xlabel("base x (mm)")
    ax.set_ylabel("base y (mm)")
    ax.set_title("Strip mode, plan view: rectangle %.1f x %.1f mm, top %.1f mm above the crib" % (*fit["extent_mm"], fit["top_mm"] - strip["top_z"]), fontsize=10)
    ax.legend(fontsize=8, loc="center left", bbox_to_anchor=(1.01, 0.5))
    fig.savefig(out / "02_strip_plan_view.png", dpi=110, bbox_inches="tight")
    plt.close(fig)
