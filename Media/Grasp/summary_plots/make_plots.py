"""Plots from the 2026-11-21 cad runs. Run from anywhere: python3 Media/Grasp/summary_plots/make_plots.py (needs the Code/Grasp environment: source ros_env.sh)."""
import glob
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
GRASP = HERE.parent
sys.path.insert(0, str(GRASP.parents[1] / "Code" / "Grasp"))
from housing_grasp.cad_fit import backproject  # noqa: E402  (first: sets up matplotlib)
from housing_grasp.color_marker import marker_mask, nearest_blob  # noqa: E402
from housing_grasp.config import load_config  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ---- 1. held-out accuracy of every cad fit
plans = {}
for p in sorted(glob.glob(str(GRASP / "**/plan/*/plan.json"), recursive=True) + glob.glob(str(GRASP / "work/runs/20261121_*/plan.json"))):
    d = json.load(open(p))
    if d.get("method") == "cad_fit":
        plans.setdefault(pathlib.Path(p).parent.name, d)
rows = [(k[9:11] + ":" + k[11:13] + ":" + k[13:15], d["connector"], [h["within_pct"] for h in d["fit"]["heldout"]]) for k, d in sorted(plans.items()) if k.startswith("20261121")]
fig, ax = plt.subplots(figsize=(11, 5))
w = 0.26
for v, col in enumerate(("tab:blue", "tab:orange", "tab:green")):
    bars = ax.bar(np.arange(len(rows)) + (v - 1) * w, [r[2][v] for r in rows], w, color=col, label="held-out view %d" % v)
    ax.bar_label(bars, fmt="%.0f", fontsize=7, padding=1)
ax.axhline(70, color="k", ls="--", lw=1, label="70 % (the old acceptance gate)")
ax.set_xticks(range(len(rows)))
ax.set_xticklabels(["%s\n%s" % (r[0], r[1]) for r in rows], fontsize=8)
ax.set_xlabel("time of the fit (2026-11-21) and connector")
ax.set_ylabel("held-out points within 2 mm (%)")
ax.set_ylim(0, 100)
ax.set_title("CAD fit: how well a fit made from two views predicts the third (labels in %)\n0 = the views do not agree with each other", fontsize=11)
ax.legend(fontsize=8, loc="upper right")
fig.tight_layout()
fig.savefig(HERE / "01_cad_fit_heldout_accuracy.png", dpi=110)
plt.close(fig)

# ---- 2. the same sticker seen from the 3 close views: offset from the run's own mean
g = load_config(GRASP.parents[1] / "Code" / "Grasp" / "config" / "grasp.yaml", allow_unscanned=True)["color_marker"]
fig, ax = plt.subplots(figsize=(7.5, 7))
cols, marks = ("tab:blue", "tab:orange", "tab:green"), ("o", "s", "^")
spreads, n = [], 0
for d in sorted(glob.glob(str(GRASP / "work/cad/20261121_*"))):
    d = pathlib.Path(d)
    if not (d / "target.json").exists():
        continue
    tgt = np.array(json.load(open(d / "target.json"))["target_m"])
    cen = []
    for p in sorted(d.glob("view_*.npz")):
        v = np.load(p)
        P = backproject(v["depth"], nearest_blob(marker_mask(v["color"], g), v["K"], v["T_BC"], tgt), v["K"], v["T_BC"])
        cen.append(np.median(P, 0)[:2] if len(P) > 15 else np.full(2, np.nan))
    cen = np.array(cen)
    if len(cen) != 3 or np.isnan(cen).any():
        continue
    off = cen - cen.mean(0)
    spreads.append(np.ptp(cen, axis=0).max())
    for k in range(3):
        ax.scatter(*off[k], c=cols[k], marker=marks[k], s=55, alpha=0.8, label="close view %d" % k if n == 0 else None)
    n += 1
ax.axhline(0, color="grey", lw=0.5)
ax.axvline(0, color="grey", lw=0.5)
ax.set_aspect("equal")
ax.set_xlabel("sticker x minus the run's mean (mm)")
ax.set_ylabel("sticker y minus the run's mean (mm)")
ax.set_title("Same sticker, 3 close views, %d runs: each view puts it in a different place\n(largest x or y spread per run: %.0f to %.0f mm; the same view lands in a similar place in most runs)" % (n, min(spreads), max(spreads)), fontsize=9)
ax.legend(fontsize=9, loc="upper right")
fig.tight_layout()
fig.savefig(HERE / "02_view_to_view_sticker_offsets.png", dpi=110)
plt.close(fig)
print("rows", len(rows), "| runs in plot 2:", n, "| spreads (mm):", np.round(spreads, 1).tolist())
