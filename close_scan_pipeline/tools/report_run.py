#!/usr/bin/env python3
"""Plots, metrics and README for one run folder (stage 1 close scan, and the grasp if there is one). Offline.

  python3 ~/kinova/work/tools/report_run.py RUN_DIR [--note "operator observation"]

Writes RUN/plots/*.png, RUN/metrics.json and RUN/README.md. Called automatically at the end of stage1_close.py and grasp_plug.py.
Plots (one story each): 01 views with the plug points, 02 board yaw per view, 03 plug wide-face yaw vs the strip direction, 04 plug points top view with the fitted box,
05 grasp depth (side view, measured board and plug top, jaw band), 06 grasp sequence images, 07 hold check numbers.
"""
import argparse
import json
import pathlib
import sys

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board_yaw as by  # noqa: E402
import connector  # noqa: E402
import plug_points as pp  # noqa: E402

plt.rcParams.update({"font.size": 11, "axes.titlesize": 12, "axes.grid": True, "grid.alpha": 0.3})
COL = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown", "tab:cyan"]


def circ_mean(a):
    return float(np.degrees(np.angle(np.exp(2j * np.radians(a)).mean())) / 2 % 180)


def build(run):
    views = sorted((run / "method2_perception/scan").glob("view_*.npz"))
    s1 = json.loads((run / "stage1_close.json").read_text())
    if "yaw_cue" not in s1:
        s1["yaw_cue"] = "wide_faces" if "yaw_left" in s1 else "none"
    part = s1.get("part", "NAUO49")
    prof = connector.load(s1.get("connector", "usb"))
    pr = pp.prior(views, part)
    data = pp.points(views, pr, prof["points"]["radius_m"], top_tol=prof["points"].get("top_tol_m", 0.006), open_mm=prof["points"].get("open_mm"), mode=prof["points"].get("mode", "teal_or_dark"))
    boards = [by.board(v) for v in views]
    wf = pp.wide_face_yaw(views, data, pr["board_yaw"])
    offs = json.loads((run / "orbit.json").read_text())["offsets_m"] if (run / "orbit.json").exists() else [0] * len(views)
    return views, s1, pr, data, boards, wf, offs


def plot_views(run, data, offs, s1):
    fig, ax = plt.subplots(1, len(data), figsize=(3.2 * len(data), 3.6))
    for a, d, o in zip(np.atleast_1d(ax), data, offs):
        im = d["color"].copy()
        im[d["mask"]] = (255, 0, 255)
        vv, uu = np.nonzero(d["mask"])
        cy, cx = int(np.median(vv)), int(np.median(uu))
        a.imshow(cv2.cvtColor(im[max(0, cy - 90):cy + 90, max(0, cx - 110):cx + 110], cv2.COLOR_BGR2RGB))
        a.set_title("camera %+.0f mm sideways" % (o * 1e3))
        a.axis("off")
    fig.suptitle("Plug points (magenta) seen from the left to the right of the plug")
    fig.tight_layout()
    fig.savefig(run / "plots/01_views_plug_overlay.png", dpi=110)
    plt.close(fig)


def plot_board(run, boards):
    y = [b["yaw"] for b in boards]
    fig, ax = plt.subplots(figsize=(6, 3.8))
    ax.plot(range(len(y)), y, "o-", color="tab:blue", label="board strip yaw per view")
    ax.axhline(np.mean(y), color="k", ls="--", label="mean %.2f deg" % np.mean(y))
    ax.set_xlabel("view")
    ax.set_ylabel("board strip yaw (deg)")
    ax.set_title("Board strip yaw per view (spread %.2f deg)" % np.ptp(y))
    ax.set_xticks(range(len(y)))
    ax.set_ylim(np.mean(y) - 2, np.mean(y) + 2)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(run / "plots/02_board_yaw_per_view.png", dpi=110)
    plt.close(fig)


def plot_faces(run, wf, strip):
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.axhspan(strip - 10, strip + 10, color="tab:green", alpha=0.15, label="strip direction +-10 deg")
    ax.axhline(strip, color="tab:green")
    for k, w in enumerate(wf):
        if w["yaw"] is None or w["pixels"] < 20:
            continue
        ax.scatter(k, w["yaw"], s=20 + w["pixels"] / 3, color="tab:blue" if w["side"] == "+y" else "tab:red", alpha=0.8)
        ax.annotate("%d px" % w["pixels"], (k, w["yaw"]), textcoords="offset points", xytext=(8, 6), fontsize=9)
    ax.scatter([], [], color="tab:blue", label="left face (normal +y)")
    ax.scatter([], [], color="tab:red", label="right face (normal -y)")
    ax.set_xlabel("view")
    ax.set_ylabel("wide-face normal direction (deg)")
    ax.set_title("Plug wide-face yaw per view (marker size = pixels)")
    ax.set_xticks(range(len(wf)))
    ax.set_xlim(-0.5, len(wf) - 0.3)
    ax.legend(loc="lower left", fontsize=9)
    fig.tight_layout()
    fig.savefig(run / "plots/03_plug_wide_face_yaw.png", dpi=110)
    plt.close(fig)


def plot_top(run, data, s1, pr):
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    for k, d in enumerate(data):
        ax.scatter(d["pts"][:, 1] * 1e3, d["pts"][:, 0] * 1e3, s=3, color=COL[k % len(COL)], alpha=0.5, label="view %d" % k)
    if s1.get("yaw_cue") == "wide_faces":
        a_ = np.radians(s1["yaw_deg"] - 90)
        ex, ey = np.array([np.cos(a_), np.sin(a_)]) * 7.5e-3, np.array([-np.sin(a_), np.cos(a_)]) * 3.75e-3
        c = np.array([s1["x"], s1["y"]])
        xy = np.array([c - ex - ey, c + ex - ey, c + ex + ey, c - ex + ey, c - ex - ey])
        ax.plot(xy[:, 1] * 1e3, xy[:, 0] * 1e3, "k-", lw=1.5, label="CAD footprint 15 x 7.5 mm\nat the reported yaw %.1f deg" % s1["yaw_deg"])
    ax.plot(s1["y"] * 1e3, s1["x"] * 1e3, "k+", ms=10, mew=2, label="reported centre")
    ax.set_xlabel("base y, left is + (mm)")
    ax.set_ylabel("base x, away from the arm (mm)")
    ax.invert_xaxis()
    ax.set_aspect("equal")
    ax.set_title("Plug points, top view (reported centre x %.1f y %.1f mm)" % (s1["x"] * 1e3, s1["y"] * 1e3))
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=9, markerscale=2)
    fig.tight_layout()
    fig.savefig(run / "plots/04_plug_points_top_view.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


def plot_depth(run, data, g, pr):
    P = np.vstack([d["pts"] for d in data])
    c = np.median(P[:, :2], axis=0)
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    ax.scatter((P[:, 1] - c[1]) * 1e3, P[:, 2] * 1e3, s=3, color="tab:gray", alpha=0.4, label="plug points")
    ax.axhline(g["board_top"] * 1e3, color="tab:brown", label="board top %.1f mm" % (g["board_top"] * 1e3))
    ax.axhline(g["plug_top"] * 1e3, color="tab:purple", label="plug top %.1f mm" % (g["plug_top"] * 1e3))
    tcp = g["T_BG"][2][3] * 1e3
    ax.axhspan(tcp, tcp + 12, color="tab:orange", alpha=0.3, label="jaw band %.1f to %.1f mm" % (tcp, tcp + 12))
    ax.set_xlabel("sideways from the plug centre, left is + (mm)")
    ax.set_ylabel("height in the base frame (mm)")
    ax.set_title("Grasp depth: plug stands %.0f mm above the board" % ((g["plug_top"] - g["board_top"]) * 1e3))
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=9, markerscale=3)
    fig.tight_layout()
    fig.savefig(run / "plots/05_grasp_depth.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


def plot_sequence(run, folder, standoff=0.09):
    names = [("1_pregrasp", "pre-grasp %.0f mm above" % (standoff * 1e3)), ("1b_20mm_above", "20 mm above"), ("2_at_grasp", "at the grasp pose"), ("3_closed", "jaws closed"), ("4_lifted", "lifted 80 mm")]
    items = [(n, t) for n, t in names if (folder / (n + ".png")).exists()]
    fig, ax = plt.subplots(2, 3, figsize=(12, 5.6))
    for a in ax.ravel():
        a.axis("off")
    for a, (n, t) in zip(ax.ravel(), items):
        a.imshow(cv2.cvtColor(cv2.imread(str(folder / (n + ".png"))), cv2.COLOR_BGR2RGB))
        a.set_title(t)
    fig.suptitle("Wrist camera during the grasp")
    fig.tight_layout()
    fig.savefig(run / "plots/06_grasp_sequence.png", dpi=110)
    plt.close(fig)


def plot_hold(run, st):
    fig, ax = plt.subplots(figsize=(5.4, 3.8))
    vals = [st["depth_between_tips"], st["table_depth"]]
    ax.bar(["between the\nfingertips", "table in the\nlower corners"], vals, color=["tab:green" if st["held"] else "tab:red", "tab:gray"])
    ax.axhline(0.20, color="k", ls="--", label="tips must be < 0.20 m")
    for i, v in enumerate(vals):
        ax.text(i, v + 0.005, "%.3f m" % v, ha="center")
    ax.set_ylabel("depth from the lens (m)")
    ax.set_ylim(0, max(vals) * 1.2)
    ax.set_title("Hold check after the 80 mm lift: %s" % ("object in hand" if st["held"] else "nothing between the tips"))
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(run / "plots/07_hold_check.png", dpi=110)
    plt.close(fig)


def main(run, note=None):
    run = pathlib.Path(run).resolve()
    (run / "plots").mkdir(exist_ok=True)
    views, s1, pr, data, boards, wf, offs = build(run)
    plot_views(run, data, offs, s1)
    plot_board(run, boards)
    made = ["01_views_plug_overlay", "02_board_yaw_per_view"]
    if s1.get("yaw_cue") == "wide_faces":
        plot_faces(run, wf, pr["board_yaw"])
        made.append("03_plug_wide_face_yaw")
    plot_top(run, data, s1, pr)
    made.append("04_plug_points_top_view")
    gj = run / "grasp/grasp.json"
    g = json.loads(gj.read_text()) if gj.exists() else None
    if g:
        plot_depth(run, data, g, pr)
        plot_sequence(run, run / "grasp", g.get("standoff_m", 0.09))
        made += ["05_grasp_depth", "06_grasp_sequence"]
        chk = [s for s in g["stages"] if "camera check" in s["name"]]
        if chk:
            plot_hold(run, chk[0])
            made.append("07_hold_check")
    m = dict(run=run.name, stage1=s1, grasp=g)
    (run / "metrics.json").write_text(json.dumps(m, indent=1))
    old = run / "README.md"
    if note is None and old.exists() and "## Operator observation\n" in old.read_text():
        note = old.read_text().split("## Operator observation\n", 1)[1].split("\n## ", 1)[0].strip()
    L = ["# %s\n" % run.name, "Stage 1 (close scan) **%s**. %d views.\n" % ("PASS" if s1["passed"] else "FAIL", s1["views"]), "| gate | result | detail |\n|---|---|---|\n"]
    L += ["| %s | %s | %s |\n" % (x["name"], "ok" if x["ok"] else "FAIL", x["detail"]) for x in s1["gates"]]
    L.append("\nPlug centre x %.4f y %.4f m; in the board frame %+.1f mm along the strip, %+.1f mm across it.\n" % (s1["x"], s1["y"], s1["along_strip_mm"], s1["across_strip_mm"]))
    if g:
        L += ["\n## Grasp\n", "Board top %.4f m, plug top %.4f m (plug stands %.0f mm). Grasp tcp z %.4f m.\n\n" % (g["board_top"], g["plug_top"], (g["plug_top"] - g["board_top"]) * 1e3, g["T_BG"][2][3]), "| stage | detail |\n|---|---|\n"]
        for s in g["stages"]:
            L.append("| %s | %s |\n" % (s["name"], {k: v for k, v in s.items() if k not in ("name", "t", "gripper")}))
        L.append("\nThe software result is the wrist-camera rule only; physical hold is judged by the operator.\n")
    if note:
        L += ["\n## Operator observation\n", note + "\n"]
    L += ["\n## Plots (plots/)\n"] + ["- `%s.png`\n" % n for n in made]
    (run / "README.md").write_text("".join(L))
    print("report: %d plots, metrics.json, README.md in %s" % (len(made), run))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=pathlib.Path)
    ap.add_argument("--note")
    a = ap.parse_args()
    main(a.run, a.note)
