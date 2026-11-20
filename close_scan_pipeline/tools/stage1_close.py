#!/usr/bin/env python3
"""Stage 1 check for the close scan (scan_orbit.py): plug pose + pass/fail from saved views. Offline, moves nothing. No CAD part is fitted.

  python3 ~/kinova/work/tools/stage1_close.py RUN_DIR [--placement P1 --cable A] [--ref X Y] [--no-log]

Plug points: sticker-teal or dark pixels near the board-predicted plug, below the plug top + 6 mm, cable excluded (tools/plug_points.py). Gates, all must hold:
  1 plug points in every view (>= 200 each) and the board strip found in every view (150 x 51 mm +-15 %) with its yaw agreeing between views within 2 deg
  2 wide-face yaw: the plug's flat (+-y) faces seen from the left (normal +y) and from the right (normal -y), each with >= 150 pixels, agree within 5 deg and the mean is within 10 deg of the board strip direction
    (the CAD relation: the wide faces are parallel to the strip)
  3 plug centre repeatable: leaving each view out in turn moves the box centre by <= 3 mm (largest pairwise)
  4 (only with --ref X Y) the plug centre is within 8 mm of the hand-measured position
Reported: yaw of the wide faces (deg), plug centre in the base frame and in the board frame (along the strip / across it, relative to the strip centre), footprint, height range.
"""
import argparse
import csv
import json
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board_yaw as by  # noqa: E402
import connector  # noqa: E402
import plug_points as pp  # noqa: E402

RUNS = pathlib.Path.home() / "kinova/work/grasp_runs"


def evaluate(run, ref=None, name="usb"):
    views = sorted((run / "method2_perception/scan").glob("view_*.npz"))
    prof = connector.load(name)
    gates, out = [], dict(run=str(run), views=len(views), connector=name, part=prof["part"], yaw_cue=prof["yaw_cue"])
    B = [by.board(v) for v in views]
    pr = pp.prior(views, prof["part"])
    data = pp.points(views, pr, prof["points"]["radius_m"])
    npts = [len(d["pts"]) for d in data]
    byaw = (np.array([b["yaw"] for b in B]) + 90) % 180 - 90
    bspread = float(np.ptp(byaw))
    bok = all(130 <= b["size"][0] <= 170 and 40 <= b["size"][1] <= 62 for b in B)
    gates.append(("plug points and board in every view", min(npts) >= prof["min_points"] and bok and bspread <= 2.0, "plug points per view %s (>= %d), board %s mm, board yaw spread %.2f deg (<= 2)" % (npts, prof["min_points"], [tuple(int(round(s)) for s in b["size"]) for b in B][:3], bspread)))
    strip = pr["board_yaw"]
    if prof["yaw_cue"] == "wide_faces":
        wf = pp.wide_face_yaw(views, data, strip)
        side = {}
        for s in ("+y", "-y"):
            items = [w for w in wf if w["side"] == s and w["pixels"] > 0]
            px = sum(w["pixels"] for w in items)
            if items and px:
                z = sum(w["pixels"] * np.exp(2j * np.radians(w["yaw"])) for w in items)
                side[s] = (float(np.degrees(np.angle(z)) / 2 % 180), px)
        if len(side) == 2 and min(p for _, p in side.values()) >= 150:
            d_lr = abs((side["+y"][0] - side["-y"][0] + 90) % 180 - 90)
            z = sum(p * np.exp(2j * np.radians(a)) for a, p in side.values())
            yaw = float(np.degrees(np.angle(z)) / 2 % 180)
            d_strip = (yaw - strip + 90) % 180 - 90
            gates.append(("wide-face yaw agrees left vs right and with the strip", d_lr <= 5 and abs(d_strip) <= 10, "left face %.1f deg (%d px), right face %.1f deg (%d px), difference %.1f (<= 5); mean %.1f deg, %+.1f from the strip (<= 10)" % (side["+y"][0], side["+y"][1], side["-y"][0], side["-y"][1], d_lr, yaw, d_strip)))
            out.update(yaw_deg=yaw, yaw_left=side["+y"][0], yaw_right=side["-y"][0], yaw_vs_strip=float(d_strip))
        else:
            gates.append(("wide-face yaw agrees left vs right and with the strip", False, "needs >= 150 wide-face pixels from both the left and the right side, got %s" % {k: v[1] for k, v in side.items()}))
    else:
        out.update(yaw_deg=float(strip % 180), yaw_note="no yaw cue for this connector: jaws along the board strip direction")
    rng = np.random.default_rng(1)

    def merged(ix):
        return np.vstack([d["pts"][rng.choice(len(d["pts"]), min(600, len(d["pts"])), replace=False)] for i, d in enumerate(data) if i in ix])
    full = pp.fit_box(merged(set(range(len(data)))))
    cen = np.array([pp.fit_box(merged(set(range(len(data))) - {k}))["centre"] for k in range(len(data))])
    loo = float(max(np.linalg.norm(a - b) for a in cen for b in cen) * 1e3)
    gates.append(("plug centre repeatable leaving a view out", loo <= 3.0, "largest shift %.1f mm (<= 3)" % loo))
    yb = np.radians(strip - 90)
    u, v = np.array([-np.sin(yb), np.cos(yb)]), np.array([np.cos(yb), np.sin(yb)])
    rel = full["centre"] - pr["board_centre"]
    out.update(x=float(full["centre"][0]), y=float(full["centre"][1]), z0=full["z0"], z1=full["z1"], footprint_mm=[full["long"] * 1e3, full["short"] * 1e3], along_strip_mm=float(rel @ u * 1e3), across_strip_mm=float(rel @ v * 1e3), loo_mm=loo, strip_dir=strip)
    if ref is not None:
        err = float(np.hypot(full["centre"][0] - ref[0], full["centre"][1] - ref[1]) * 1e3)
        gates.append(("within 8 mm of the reference", err <= 8, "error %.1f mm" % err))
        out["ref_err_mm"] = err
    out["gates"] = [dict(name=g[0], ok=bool(g[1]), detail=g[2]) for g in gates]
    out["passed"] = all(g[1] for g in gates)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=pathlib.Path)
    ap.add_argument("--placement", default="")
    ap.add_argument("--cable", default="")
    ap.add_argument("--ref", type=float, nargs=2)
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--connector", default="usb")
    a = ap.parse_args()
    res = evaluate(a.run.resolve(), a.ref, a.connector)
    print("Stage 1 (close scan) check of %s" % a.run.name)
    for g in res["gates"]:
        print("  [%s] %s: %s" % ("ok " if g["ok"] else "FAIL", g["name"], g["detail"]))
    print("  plug: wide-face yaw %s, centre x %.4f y %.4f m, in the board frame %+.1f mm along the strip / %+.1f mm across it (relative to the strip centre), footprint %.1f x %.1f mm" % (("%.1f deg" % res["yaw_deg"]) if "yaw_deg" in res else "not accepted", res["x"], res["y"], res["along_strip_mm"], res["across_strip_mm"], *res["footprint_mm"]))
    print("  RESULT: %s" % ("PASS" if res["passed"] else "FAIL"))
    (a.run / "stage1_close.json").write_text(json.dumps(res, indent=1))
    if not a.no_log:
        f = RUNS / "stage1_close_trials.csv"
        new = not f.exists()
        with open(f, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["time", "run", "placement", "cable", "result", "failed_gates", "yaw_deg", "x_m", "y_m", "along_strip_mm", "across_strip_mm", "loo_mm", "yaw_left", "yaw_right"])
            g = lambda k, n=2: "" if k not in res else round(res[k], n)
            w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), a.run.name, a.placement, a.cable, "PASS" if res["passed"] else "FAIL", "; ".join(x["name"] for x in res["gates"] if not x["ok"]), g("yaw_deg"), g("x", 4), g("y", 4), g("along_strip_mm"), g("across_strip_mm"), g("loo_mm"), g("yaw_left"), g("yaw_right")])
        print("  logged to %s" % f)
    import report_run
    report_run.main(a.run)


if __name__ == "__main__":
    main()
