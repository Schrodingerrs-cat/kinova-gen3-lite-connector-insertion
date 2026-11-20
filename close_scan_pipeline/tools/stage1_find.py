#!/usr/bin/env python3
"""Stage 1 (find the plug): one trial = 3 camera views -> plug pose + pass/fail. Appends a line to grasp_runs/stage1_trials.csv.

  python3 ~/kinova/work/tools/stage1_find.py --run RUN_DIR                       evaluate saved views offline (RUN_DIR/method2_perception/scan), moves nothing
  python3 ~/kinova/work/tools/stage1_find.py --placement P1 --cable A            show what a live trial would do, moves nothing
  python3 ~/kinova/work/tools/stage1_find.py --placement P1 --cable A --execute  LIVE: the arm goes to the 3 camera views and back home (no grasp, gripper does not move), then the check
  optional: --ref X Y   the plug's real position in the base frame (m, measured by hand); adds a gate "within 8 mm of it"

Live needs the stack (run_stack_method2.sh) up, the arm at project home, the gripper open, e-stop in reach, and ros_env.sh sourced.

Gates (all must hold for PASS):
  1 plug found in all 3 views (dark-teal sticker blob, cable and depth edges removed)
  2 plug top-face centre agrees between views: largest xy distance <= 8 mm and z spread <= 8 mm
  3 board strip found in all 3 views (long side 130-170 mm, short side 40-62 mm) and its yaw agrees between views within 2 deg
  4 plug outline yaw agrees with board yaw - 90 deg (the CAD relation) within 15 deg (mean over the views, mod 180)
  5 (only with --ref) the plug position is within 8 mm of the reference
Reported pose: x, y = mean top-face centre, z = mean top height, yaw = board yaw - 90 deg (mod 180) when gates 3 and 4 hold.
"""
import argparse
import csv
import json
import pathlib
import re
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board_yaw  # noqa: E402
import find_plug_m3  # noqa: E402

RUNS = pathlib.Path.home() / "kinova/work/grasp_runs"
XY_TOL, Z_TOL, BOARD_SPREAD, YAW_TOL, REF_TOL = 8.0, 8.0, 2.0, 15.0, 8.0


def circ_mean_180(a):
    z = np.exp(2j * np.radians(a))
    return float(np.degrees(np.angle(z.mean())) / 2 % 180)


def circ_diff_180(a, b):
    return float((a - b + 90) % 180 - 90)


def evaluate(run, ref=None):
    scan = run / "method2_perception/scan"
    views = sorted(scan.glob("view_*.npz"))
    gates, P, B, notes = [], [], [], []
    for v in views:
        try:
            P.append(find_plug_m3.plug(v))
        except Exception as e:
            notes.append("%s plug: %s" % (v.stem, str(e)[:80] or type(e).__name__))
        try:
            B.append(board_yaw.board(v))
        except Exception as e:
            notes.append("%s board: %s" % (v.stem, str(e)[:80] or type(e).__name__))
    n = len(views)
    gates.append(("plug found in all views", len(P) == n and n >= 3, "%d of %d views" % (len(P), n)))
    out = dict(run=str(run), views=n, notes=notes)
    if len(P) >= 2:
        C = np.array([p["centre"] for p in P])
        xy = max(np.linalg.norm(a[:2] - b[:2]) for a in C for b in C) * 1e3
        zs = np.ptp(C[:, 2]) * 1e3
        gates.append(("plug centre agrees between views", xy <= XY_TOL and zs <= Z_TOL, "xy %.1f mm (<= %.0f), z %.1f mm (<= %.0f)" % (xy, XY_TOL, zs, Z_TOL)))
        out.update(x=float(C[:, 0].mean()), y=float(C[:, 1].mean()), z=float(C[:, 2].mean()), view_xy_mm=float(xy), view_z_mm=float(zs))
    else:
        gates.append(("plug centre agrees between views", False, "fewer than 2 views"))
    board_ok = False
    if len(B) == n and n >= 3:
        spread = float(np.ptp((np.array([b["yaw"] for b in B]) + 90) % 180 - 90))
        sizes_ok = all(130 <= b["size"][0] <= 170 and 40 <= b["size"][1] <= 62 for b in B)
        board_ok = sizes_ok and spread <= BOARD_SPREAD
        gates.append(("board found, yaw agrees", board_ok, "rectangles %s mm, yaw spread %.2f deg (<= %.0f)" % ([tuple(int(round(s)) for s in b["size"]) for b in B], spread, BOARD_SPREAD)))
        out.update(board_yaw=circ_mean_180([b["yaw"] for b in B]), board_spread=spread)
    else:
        gates.append(("board found, yaw agrees", False, "%d of %d views" % (len(B), n)))
    if len(P) >= 2 and "board_yaw" in out:
        outline = circ_mean_180([p["yaw"] for p in P])
        expect = (out["board_yaw"] - 90) % 180
        resid = circ_diff_180(outline, expect)
        gates.append(("plug outline yaw agrees with board yaw - 90", abs(resid) <= YAW_TOL, "outline %.1f deg, board-derived %.1f deg, difference %+.1f (<= %.0f)" % (outline, expect, resid, YAW_TOL)))
        out.update(outline_yaw=outline, yaw_resid=resid)
        if board_ok and abs(resid) <= YAW_TOL:
            out["yaw"] = expect
    else:
        gates.append(("plug outline yaw agrees with board yaw - 90", False, "needs plug and board"))
    if ref is not None and "x" in out:
        err = float(np.hypot(out["x"] - ref[0], out["y"] - ref[1]) * 1e3)
        gates.append(("within %.0f mm of the reference" % REF_TOL, err <= REF_TOL, "error %.1f mm" % err))
        out["ref_err_mm"] = err
    out["gates"] = [dict(name=g[0], ok=bool(g[1]), detail=g[2]) for g in gates]
    out["passed"] = all(g[1] for g in gates)
    return out, P, B


def summary_image(run, P, B):
    tiles = []
    for p in P:
        im = p["colour"].copy()
        im[p["mask"]] = (0, 0, 255)
        x, y = [int(v) for v in p["seed"]]
        tiles.append(cv2.resize(im[max(0, y - 80):y + 80, max(0, x - 100):x + 100], None, fx=2, fy=2))
    if tiles:
        h = min(t.shape[0] for t in tiles)
        cv2.imwrite(str(run / "plots/stage1_plug_mask.png"), np.hstack([t[:h] for t in tiles]))
    if B:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, len(B), figsize=(4 * len(B), 5))
        for a, (k, b) in zip(np.atleast_1d(ax), enumerate(B)):
            yy, xx = np.nonzero(b["grid"])
            a.scatter(0.2 + xx / 1000, -0.15 + yy / 1000, s=1)
            cc = np.vstack([b["corners"], b["corners"][:1]])
            a.plot(0.2 + cc[:, 0] / 1000, -0.15 + cc[:, 1] / 1000, "r-")
            a.set_aspect("equal"); a.set_title("view %d, board yaw %.1f" % (k, b["yaw"])); a.set_xlabel("base x (m)")
        plt.savefig(run / "plots/stage1_board.png", dpi=80, bbox_inches="tight")


def report(res):
    for g in res["gates"]:
        print("  [%s] %s: %s" % ("ok " if g["ok"] else "FAIL", g["name"], g["detail"]))
    for n in res["notes"]:
        print("  note:", n)
    if "x" in res:
        print("  plug top-face centre x %.4f y %.4f z %.4f m%s" % (res["x"], res["y"], res["z"], (", yaw %.1f deg" % res["yaw"]) if "yaw" in res else ", yaw not accepted"))
    print("  RESULT: %s" % ("PASS" if res["passed"] else "FAIL"))


def log_line(res, placement, cable):
    f = RUNS / "stage1_trials.csv"
    new = not f.exists()
    with open(f, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["time", "run", "placement", "cable", "result", "failed_gates", "x_m", "y_m", "z_m", "yaw_deg", "view_xy_mm", "view_z_mm", "board_yaw_deg", "yaw_diff_deg", "ref_err_mm"])
        g = lambda k: "" if k not in res else round(res[k], 4 if k in ("x", "y", "z") else 2)
        w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), pathlib.Path(res["run"]).name, placement, cable, "PASS" if res["passed"] else "FAIL", "; ".join(x["name"] for x in res["gates"] if not x["ok"]),
                    g("x"), g("y"), g("z"), g("yaw"), g("view_xy_mm"), g("view_z_mm"), g("board_yaw"), g("yaw_resid"), g("ref_err_mm")])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=pathlib.Path)
    ap.add_argument("--placement", default="")
    ap.add_argument("--cable", default="")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--ref", type=float, nargs=2, metavar=("X", "Y"))
    ap.add_argument("--no-log", action="store_true", help="do not append to stage1_trials.csv")
    a = ap.parse_args()
    if a.run:
        run = a.run.resolve()
    else:
        if not (a.placement and a.cable):
            sys.exit("live trial needs --placement and --cable (labels such as P1 and A)")
        name = "stage1_%s_%s" % (a.placement, a.cable)
        cmd = [sys.executable, str(HERE / "run_method2_full.py"), "--find-only", "--execute", "--name", name]
        if not a.execute:
            print("would run: %s\nthen evaluate the 3 new views. Add --execute to run it (arm moves along 3 camera poses and back home, no grasp)." % " ".join(cmd))
            return
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        run = None
        for line in proc.stdout:
            if "Axes3D" in line or "warnings.warn" in line or "multicast" in line:
                continue
            print(line, end="", flush=True)
            m = re.search(r"saved to (\S+)", line)
            if m:
                run = pathlib.Path(m.group(1))
        proc.wait()
        if run is None or not (run / "method2_perception/scan/view_00.npz").exists():
            sys.exit("the arm run did not produce views, see its output above (no evaluation)")
    print("\nStage 1 check of %s" % run.name)
    res, P, B = evaluate(run, a.ref)
    report(res)
    (run / "plots").mkdir(exist_ok=True)
    summary_image(run, P, B)
    (run / "stage1.json").write_text(json.dumps(res, indent=1))
    if not a.no_log:
        log_line(res, a.placement, a.cable)
        print("  logged to %s" % (RUNS / "stage1_trials.csv"))


if __name__ == "__main__":
    main()
