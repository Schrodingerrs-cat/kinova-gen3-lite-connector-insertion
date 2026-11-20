#!/usr/bin/env python3
"""Board yaw from the white board strip, per saved view. Offline, moves nothing.

  python3 ~/kinova/work/tools/board_yaw.py SCAN_DIR [--plot OUT_DIR]      SCAN_DIR holds view_00.npz ... (method 2 scan folder)

The plug sits in a fixed cradle on the board, so the board's yaw is an independent cue for the plug's yaw. Per view: pale-white pixels with depth are put in the base
frame, the ones on the board strip's top face are chosen by height (dominant white height above the table), and a rotated rectangle is fitted to their outline.
Prints the long-edge yaw (mod 180 deg), the rectangle size and the strip centre. Agreement between views is the check that the cue is usable.
"""
import argparse
import pathlib

import cv2
import numpy as np

TABLE = -0.041


def board(view):
    f = np.load(view)
    d, c, K, T = [f[k] for k in ("depth", "color", "K", "T_BC")]
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    ok = (d > 0.1) & (d < 0.8)
    z = d[ok]
    P = np.c_[(u[ok] - K[0, 2]) * z / K[0, 0], (v[ok] - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
    H, S, V = [hsv[..., i][ok].astype(int) for i in range(3)]
    white = (S < 110) & (V > 150) & (H >= 85) & (H <= 125)
    box = (P[:, 0] > 0.2) & (P[:, 0] < 0.4) & (P[:, 1] > -0.15) & (P[:, 1] < 0.25) & (P[:, 2] > TABLE + 0.012)
    Q = P[white & box]
    hist, edges = np.histogram(Q[:, 2], bins=np.arange(TABLE, 0.12, 0.003))
    zc = edges[np.argmax(hist)] + 0.0015
    Q = Q[np.abs(Q[:, 2] - zc) < 0.006]
    g = np.zeros((400, 300), np.uint8)
    gx = ((Q[:, 0] - 0.2) * 1000).astype(int)
    gy = ((Q[:, 1] + 0.15) * 1000).astype(int)
    keep = (gx >= 0) & (gx < 300) & (gy >= 0) & (gy < 400)
    g[gy[keep], gx[keep]] = 1
    g = cv2.morphologyEx(g, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(g)
    i = 1 + int(np.argmax(st[1:, 4]))
    ys, xs = np.nonzero(lab == i)
    hull = cv2.convexHull(np.c_[xs, ys].astype(np.float32))
    (cx, cy), (w, h), _ = cv2.minAreaRect(hull)
    corners = cv2.boxPoints(cv2.minAreaRect(hull))
    e = [corners[(k + 1) % 4] - corners[k] for k in range(4)]
    long_edge = max(e, key=lambda a: np.hypot(*a))
    yaw = np.degrees(np.arctan2(long_edge[1], long_edge[0])) % 180
    return dict(yaw=yaw, size=sorted([w, h], reverse=True), centre=(0.2 + cx / 1000, -0.15 + cy / 1000), z=zc, pts=len(Q), grid=lab == i, corners=corners)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scan", type=pathlib.Path)
    ap.add_argument("--plot", type=pathlib.Path)
    args = ap.parse_args()
    results = []
    for view in sorted(args.scan.glob("view_*.npz")):
        r = board(view)
        results.append(r)
        print("%s: board yaw %6.2f deg | rectangle %.0f x %.0f mm | centre x %.4f y %.4f m | top face z %.3f m | %d points" % (view.stem, r["yaw"], *r["size"], *r["centre"], r["z"], r["pts"]))
    ys = np.array([r["yaw"] for r in results])
    ys = (ys + 90) % 180 - 90
    print("yaw between views: mean %.2f deg, spread (max-min) %.2f deg" % (ys.mean() % 180, np.ptp(ys)))
    if args.plot:
        args.plot.mkdir(parents=True, exist_ok=True)
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, len(results), figsize=(5 * len(results), 5))
        for a, (k, r) in zip(np.atleast_1d(ax), enumerate(results)):
            yy, xx = np.nonzero(r["grid"])
            a.scatter(0.2 + xx / 1000, -0.15 + yy / 1000, s=1, c="tab:blue")
            cc = np.vstack([r["corners"], r["corners"][:1]])
            a.plot(0.2 + cc[:, 0] / 1000, -0.15 + cc[:, 1] / 1000, "r-")
            a.set_aspect("equal"); a.set_title("view %d  yaw %.1f" % (k, r["yaw"])); a.set_xlabel("base x (m)"); a.set_ylabel("base y (m)")
        plt.savefig(args.plot / "board_yaw.png", dpi=80, bbox_inches="tight")


if __name__ == "__main__":
    main()
