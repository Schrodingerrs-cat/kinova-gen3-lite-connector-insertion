#!/usr/bin/env python3
"""Stage 1 check for the close scan (scan_orbit.py): plug pose + pass/fail from saved views. Offline, moves nothing. No CAD part is fitted.

  python3 ~/kinova/work/tools/stage1_close.py RUN_DIR [--placement P1 --cable A] [--ref X Y] [--no-log]

Plug points: marker-colour or dark pixels near the marker found by colour in the views, below the measured marker top + tolerance, cable excluded (tools/plug_points.py). Gates, all must hold:
  1 plug points in every view (the board outline and its yaw are reported, not a condition: the board may sit at any angle)
  2 plug centre repeatable: leaving each view out in turn moves the box centre by <= 3 mm (largest pairwise; per-connector loo_mm)
  3 USB only (yaw_cue wide_faces): the direction its near-vertical faces mostly face (the wide faces carry more pixels than the narrow ones, from the run's depth) must dominate the perpendicular direction
    (ratio >= 1.3); the opposite faces along it must each have >= 150 pixels and agree within 12 deg. No board or strip direction is used. The other plugs: the jaw direction is chosen from the point cloud at grasp time
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
    pr = pp.prior(views, prof["marker"])
    data = pp.points(views, pr, prof["marker"], prof["points"]["radius_m"], top_tol=prof["points"].get("top_tol_m", 0.006), open_mm=prof["points"].get("open_mm"), mode=prof["points"].get("mode", "teal_or_dark"))
    npts = [len(d["pts"]) for d in data]
    B = [by.board(v, pr["plug_centre"][:2]) for v in views]
    byaw = (np.array([b["yaw"] for b in B]) + 90) % 180 - 90
    gates.append(("plug points in every view", min(npts) >= prof["min_points"], "plug points per view %s (>= %d); board yaw spread %.2f deg is reported only, the board orientation is not a condition" % (npts, prof["min_points"], float(np.ptp(byaw)))))
    strip = pr["board_yaw"]
    rng = np.random.default_rng(1)

    def merged(ix):
        return np.vstack([d["pts"][rng.choice(len(d["pts"]), min(600, len(d["pts"])), replace=False)] for i, d in enumerate(data) if i in ix])
    allix = set(range(len(data)))
    full = pp.fit_box(merged(allix))
    cen = np.array([pp.fit_box(merged(allix - {k}))["centre"] for k in range(len(data))])
    loo = float(max(np.linalg.norm(a - b) for a in cen for b in cen) * 1e3)
    loo_tol = prof.get("loo_mm", 3.0)
    gates.append(("plug centre repeatable leaving a view out", loo <= loo_tol, "largest shift %.1f mm (<= %g)" % (loo, loo_tol)))
    # the plug's own orientation: the direction its near-vertical faces mostly face (the wide faces carry more pixels than the narrow ones)
    cbox = full
    out.update(core_footprint_mm=[cbox["long"] * 1e3, cbox["short"] * 1e3])
    if prof["yaw_cue"] == "wide_faces":
        normal_dir, ratio, n_peak, n_perp = pp.dominant_face_dir(views, data)
        out.update(nominal_normal=normal_dir, face_ratio=float(ratio))
        gates.append(("plug has a clearly wider side", normal_dir is not None and ratio >= 1.3 and n_peak >= 300, "pixels facing the dominant direction %s deg: %d, facing the perpendicular: %d, ratio %.2f (>= 1.3, and >= 300 pixels)" % ("-" if normal_dir is None else "%.0f" % normal_dir, n_peak, n_perp, ratio)))
        if normal_dir is not None and gates[-1][1]:
            wf = pp.wide_face_yaw(views, data, normal_dir, tol=20)
            side = {}
            for sd in ("+y", "-y"):
                items = [w for w in wf if w["side"] == sd and w["pixels"] > 0]
                px = sum(w["pixels"] for w in items)
                if items and px:
                    z = sum(w["pixels"] * np.exp(2j * np.radians(w["yaw"])) for w in items)
                    side[sd] = (float(np.degrees(np.angle(z)) / 2 % 180), px)
            if len(side) == 2 and min(p for _, p in side.values()) >= 150:
                d_lr = abs((side["+y"][0] - side["-y"][0] + 90) % 180 - 90)
                z = sum(p * np.exp(2j * np.radians(a)) for a, p in side.values())
                yaw = float(np.degrees(np.angle(z)) / 2 % 180)
                gates.append(("wide-face yaw: both opposite faces seen and agreeing", d_lr <= 12, "face on one side %.1f deg (%d px), face on the other side %.1f deg (%d px), difference %.1f (<= 12); mean %.1f deg" % (side["+y"][0], side["+y"][1], side["-y"][0], side["-y"][1], d_lr, yaw)))
                out.update(yaw_deg=yaw, yaw_left=side["+y"][0], yaw_right=side["-y"][0])
            else:
                gates.append(("wide-face yaw: both opposite faces seen and agreeing", False, "needs >= 150 wide-face pixels from both opposite sides, got %s" % {k: v[1] for k, v in side.items()}))
    else:
        out.update(yaw_deg=None, yaw_note="footprint %.1f x %.1f mm: no flat-face cue, the jaw direction is chosen from the point cloud at grasp time" % (cbox["long"] * 1e3, cbox["short"] * 1e3))
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
    print("  plug: yaw %s, centre x %.4f y %.4f m, in the board frame %+.1f mm along the strip / %+.1f mm across it (relative to the strip centre), footprint %.1f x %.1f mm" % (("%.1f deg" % res["yaw_deg"]) if res.get("yaw_deg") is not None else ("chosen at grasp time" if "yaw_deg" in res else "not accepted"), res["x"], res["y"], res["along_strip_mm"], res["across_strip_mm"], *res["footprint_mm"]))
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
