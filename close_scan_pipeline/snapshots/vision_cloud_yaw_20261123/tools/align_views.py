#!/usr/bin/env python3
"""Per-view correction for the CAD fit: registers every view's board + plug cloud (base frame) onto the first view and writes the corrections into SCAN_DIR/fusion.json. Offline, moves nothing.

  python3 ~/kinova/work/tools/align_views.py SCAN_DIR [--ref 0]

The shift between views of a static scene is the camera-mapping error for that camera pose (arm model x hand-eye). Corrections are rigid, applied as T_corr @ T_BC. Trimmed point-to-point ICP in the region
x 0.20-0.35, y -0.12..0.08 (base), z -0.03..0.06 (board top face, plug, cable; the table is excluded). The reference view keeps the identity, so the absolute position is that view's.
Fit afterwards WITHOUT --raw-poses so tools/fit_cad_multiview.py uses the corrections.
"""
import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mapping_check as mc  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("scan", type=pathlib.Path)
ap.add_argument("--ref", type=int, default=0)
a = ap.parse_args()
views = sorted(a.scan.glob("view_*.npz"))
clouds = []
for v in views:
    f = np.load(v)
    d, K, T = f["depth"], f["K"], f["T_BC"]
    vv, uu = np.mgrid[0:d.shape[0]:2, 0:d.shape[1]:2]
    z = d[::2, ::2]
    ok = (z > 0.1) & (z < 0.7)
    P = np.c_[(uu[ok] - K[0, 2]) * z[ok] / K[0, 0], (vv[ok] - K[1, 2]) * z[ok] / K[1, 1], z[ok]] @ T[:3, :3].T + T[:3, 3]
    box = (P[:, 0] > 0.20) & (P[:, 0] < 0.35) & (P[:, 1] > -0.12) & (P[:, 1] < 0.08) & (P[:, 2] > -0.03) & (P[:, 2] < 0.06)
    clouds.append(P[box])
per = []
print("reference view %d (%d points)" % (a.ref, len(clouds[a.ref])))
for k, (v, c) in enumerate(zip(views, clouds)):
    Tc = np.eye(4)
    if k != a.ref:
        R, t, rms, frac = mc.register(c, clouds[a.ref])
        Tc[:3, :3], Tc[:3, 3] = R, t
        rot = np.degrees(np.linalg.norm(__import__("scipy.spatial.transform", fromlist=["Rotation"]).Rotation.from_matrix(R).as_rotvec()))
        print("%s: shift (%+.1f, %+.1f, %+.1f) mm, turn %.2f deg | rms %.1f mm, %.0f %% within 3 mm after" % (v.stem, *(t * 1e3), rot, rms * 1e3, 100 * frac))
    per.append(dict(view=k, T=Tc.tolist()))
fz = json.loads((a.scan / "fusion.json").read_text()) if (a.scan / "fusion.json").exists() else dict(views=list(range(len(views))), skipped=[])
fz["per_view"] = per
fz["method"] = "per-view rigid corrections by trimmed ICP of the board + plug cloud onto view %d (tools/align_views.py)" % a.ref
(a.scan / "fusion.json").write_text(json.dumps(fz, indent=1))
