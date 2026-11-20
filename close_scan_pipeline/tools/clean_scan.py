#!/usr/bin/env python3
"""Cleans the plug region of saved views before the CAD fit: removes the cable and depth-edge (mixed) pixels from the depth image. Offline, moves nothing.

  python3 ~/kinova/work/tools/clean_scan.py SCAN_DIR ROIS_JSON OUT_DIR [--open-mm 8]

Inside each view's ROI (from find_scan_target_rois.py): the pixels the fit would use (dark V < 90, or the teal sticker) are opened with an elliptical kernel wider than the cable;
what the opening removes (thin dark structure = the cable) gets depth 0. Pixels whose depth differs from the 5x5 median by more than 4 mm (mixed pixels on depth edges) also get depth 0.
Writes cleaned view_NN.npz and a copy of fusion.json to OUT_DIR, so tools/fit_cad_multiview.py can be run on OUT_DIR unchanged.
"""
import argparse
import json
import pathlib
import shutil

import cv2
import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("scan", type=pathlib.Path)
ap.add_argument("rois", type=pathlib.Path)
ap.add_argument("out", type=pathlib.Path)
ap.add_argument("--open-mm", type=float, default=8.0)
ap.add_argument("--max-z", type=float, help="drop ROI pixels whose base-frame height is above this (m): the cable rising from the plug; the plug top is at about 0.025")
ap.add_argument("--edges-only", action="store_true", help="skip the cable removal, only remove depth-edge pixels")
a = ap.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
rois = json.loads(a.rois.read_text())
for view in sorted(a.scan.glob("view_*.npz")):
    f = dict(np.load(view))
    d, c, K = f["depth"].astype(np.float32), f["color"], f["K"]
    x0, y0, x1, y1 = rois[view.stem]
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    roi = np.zeros(d.shape, bool)
    roi[y0:y1, x0:x1] = True
    teal = (hsv[..., 0] >= 80) & (hsv[..., 0] <= 105) & (hsv[..., 1] >= 70) & (hsv[..., 2] >= 35) & (hsv[..., 2] <= 140)
    region = ((hsv[..., 2] < 90) | teal) & roi & (d > 0)
    rng = float(np.median(d[region])) if region.any() else 0.2
    k = max(3, int(round(a.open_mm * K[0, 0] / rng / 1000)))
    opened = cv2.morphologyEx(region.astype(np.uint8), cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))) > 0
    cable = region & ~opened
    med = cv2.medianBlur(d, 5)
    edge = roi & (d > 0) & (np.abs(d - med) > 0.004)
    d2 = d.copy()
    high = np.zeros(d.shape, bool)
    if a.max_z is not None:
        T = f["T_BC"]
        vv, uu = np.nonzero(roi & (d > 0))
        z = d[vv, uu]
        P = np.c_[(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
        high[vv[P[:, 2] > a.max_z], uu[P[:, 2] > a.max_z]] = True
    d2[(edge if a.edges_only else (cable | edge)) | high] = 0
    f["depth"] = d2
    np.savez_compressed(a.out / view.name, **f)
    print("%s: ROI region %d px, cable (opening) %d px, depth-edge %d px, above max-z %d px" % (view.stem, region.sum(), 0 if a.edges_only else cable.sum(), edge.sum(), high.sum()))
shutil.copy(a.scan / "fusion.json", a.out / "fusion.json")
