#!/usr/bin/env python3
"""Stage 1 finder, offline on saved views (method 2 scan folder). Moves nothing.

  python3 ~/kinova/work/tools/find_plug_m3.py SCAN_DIR [--plot OUT_DIR]

Per view: (1) plug seed = largest dark-teal blob (hue 80-105, S >= 70, V 35-140) in the workspace; (2) plug mask = dark or teal pixels around the seed, cable and bolts removed by a
morphological opening wider than the cable, depth edges removed (pixels that differ from the local median depth by more than 4 mm); (3) plug points in the base frame, top face = the
highest 8 mm of them; (4) a rotated rectangle fitted to the top face outline gives centre, size and yaw (long edge, mod 180 deg). Also the board yaw from tools/board_yaw.py logic is
not repeated here: run board_yaw.py for it. Prints per view and the agreement between views.
"""
import argparse
import pathlib

import cv2
import numpy as np

WS = (0.18, 0.58, -0.32, 0.32)


def seed(view, near=None, radius=0.04):
    """Marker blob centre in the base frame from one view: (x, y, z) of the median blob point and the blob size in px. Largest dark-teal blob in the workspace, or with near=(x, y) the blob
    whose centre is closest to that point (within radius), so another connector's sticker does not confuse it."""
    f = np.load(view)
    d, c, K, T = [f[k] for k in ("depth", "color", "K", "T_BC")]
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    teal = ((hsv[..., 0] >= 80) & (hsv[..., 0] <= 105) & (hsv[..., 1] >= 70) & (hsv[..., 2] >= 35) & (hsv[..., 2] <= 140)).astype(np.uint8)
    teal = cv2.morphologyEx(teal, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(teal)
    out = []
    for i in range(1, n):
        if st[i, 4] < 60 or not (cen[i][0] < 700 and cen[i][1] < 420):
            continue
        vv, uu = np.nonzero((lab == i) & (d > 0.1) & (d < 0.8))
        if len(vv) < 30:
            continue
        z = d[vv, uu]
        P = np.median(np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3], axis=0)
        out.append((P, int(st[i, 4])))
    if near is not None:
        out = [o for o in out if np.hypot(o[0][0] - near[0], o[0][1] - near[1]) <= radius]
        out.sort(key=lambda o: np.hypot(o[0][0] - near[0], o[0][1] - near[1]))
    else:
        out.sort(key=lambda o: -o[1])
    if not out:
        raise RuntimeError("no marker blob in the view" + ("" if near is None else " near (%.3f, %.3f)" % tuple(near)))
    return out[0]


def plug(view, open_mm=8.0):
    f = np.load(view)
    d, c, K, T = [f[k] for k in ("depth", "color", "K", "T_BC")]
    d = np.where((d > 0.1) & (d < 0.8), d, 0).astype(np.float32)
    med = cv2.medianBlur(d, 5)
    d = np.where((d > 0) & (np.abs(d - med) < 0.004), d, 0)
    d = np.where(cv2.erode((d > 0).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0, d, 0)
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    teal = (H >= 80) & (H <= 105) & (S >= 70) & (V >= 35) & (V <= 140)
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    ok = d > 0
    z = d[ok]
    P = np.full(d.shape + (3,), np.nan)
    P[ok] = np.c_[(u[ok] - K[0, 2]) * z / K[0, 0], (v[ok] - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
    inws = (P[..., 0] >= WS[0]) & (P[..., 0] <= WS[1]) & (P[..., 1] >= WS[2]) & (P[..., 1] <= WS[3])
    seed_mask = cv2.morphologyEx((teal & inws).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(seed_mask)
    cand = [i for i in range(1, n) if st[i, 4] >= 60 and cen[i][0] < 700 and cen[i][1] < 420]
    i = max(cand, key=lambda j: st[j, 4])
    sx, sy = cen[i]
    zc = np.nanmedian(P[lab == i][:, 2])
    r_px = int(0.028 * K[0, 0] / np.linalg.norm(T[:3, 3] - np.r_[np.nanmedian(P[lab == i][:, :2], axis=0), zc]))
    near = np.zeros(d.shape, bool)
    near[max(0, int(sy) - r_px):int(sy) + r_px + 1, max(0, int(sx) - r_px):int(sx) + r_px + 1] = True
    mask = ((teal | (V < 90)) & near & ok).astype(np.uint8)
    px_per_mm = K[0, 0] / np.linalg.norm(T[:3, 3] - np.r_[np.nanmedian(P[lab == i][:, :2], axis=0), zc]) / 1000
    k = max(3, int(round(open_mm * px_per_mm)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    n, lab2, st2, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        raise RuntimeError("%s: nothing left after cable removal" % view.stem)
    j = 1 + int(np.argmax(st2[1:, 4]))
    Q = P[(lab2 == j)]
    Q = Q[np.isfinite(Q[:, 0])]
    ztop = np.percentile(Q[:, 2], 95)
    top = Q[Q[:, 2] > ztop - 0.008]
    cx, cy = np.median(top[:, 0]), np.median(top[:, 1])
    g = np.c_[(Q[:, 0] - Q[:, 0].mean()) * 1000, (Q[:, 1] - Q[:, 1].mean()) * 1000].astype(np.float32)
    rect = cv2.minAreaRect(cv2.convexHull(g))
    corners = cv2.boxPoints(rect)
    e = [corners[(m + 1) % 4] - corners[m] for m in range(4)]
    le = max(e, key=lambda a: np.hypot(*a))
    yaw = np.degrees(np.arctan2(le[1], le[0])) % 180
    w, vec = np.linalg.eigh(np.cov(g.T))
    pca = np.degrees(np.arctan2(vec[1, 1], vec[0, 1])) % 180
    aspect = float(np.sqrt(w[1] / w[0]))
    return dict(yaw=yaw, pca=pca, aspect=aspect, size=sorted(rect[1], reverse=True), centre=(cx, cy, ztop), n_all=len(Q), n_top=len(top), seed=(sx, sy), mask=lab2 == j, top=top, corners=corners, colour=c)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scan", type=pathlib.Path)
    ap.add_argument("--plot", type=pathlib.Path)
    ap.add_argument("--open-mm", type=float, default=8.0, help="opening width that removes the cable")
    args = ap.parse_args()
    res = []
    for view in sorted(args.scan.glob("view_*.npz")):
        r = plug(view, args.open_mm)
        res.append(r)
        print("%s: plug top face centre x %.4f y %.4f z %.4f m | footprint %.0f x %.0f mm | yaw: rectangle %6.1f / PCA %6.1f deg (elongation %.1f) | %d plug px" % (view.stem, *r["centre"], *r["size"], r["yaw"], r["pca"], r["aspect"], r["n_all"]))
    C = np.array([r["centre"] for r in res])
    Y = (np.array([r["yaw"] for r in res]) + 90) % 180 - 90
    Yp = (np.array([r["pca"] for r in res]) + 90) % 180 - 90
    print("agreement between views: x %.1f mm, y %.1f mm, z %.1f mm | yaw spread rectangle %.1f deg, PCA %.1f deg (mean %.1f)" % (*(np.ptp(C, axis=0) * 1e3), np.ptp(Y), np.ptp(Yp), Yp.mean() % 180))
    if args.plot:
        args.plot.mkdir(parents=True, exist_ok=True)
        tiles = []
        for r in res:
            im = r["colour"].copy()
            im[r["mask"]] = (0, 0, 255)
            x, y = [int(v) for v in r["seed"]]
            tiles.append(cv2.resize(im[max(0, y - 80):y + 80, max(0, x - 100):x + 100], None, fx=2.5, fy=2.5))
        h = min(t.shape[0] for t in tiles)
        cv2.imwrite(str(args.plot / "plug_mask.png"), np.hstack([t[:h] for t in tiles]))


if __name__ == "__main__":
    main()
