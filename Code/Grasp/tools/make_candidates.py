#!/usr/bin/env python3
"""Offline grasp candidates for the connectors of config/connectors.yaml (no robot):
python3 tools/make_candidates.py NAME [--out FILE]      one connector
python3 tools/make_candidates.py --all [--force]         every connector, to the file named in its config entry (existing files are kept unless --force)
Pads touch the contact part (body_u_mm). Tip height u0 starts `place.clearance_mm` above the panel seat. Round (axial) connectors: one yaw, pads centred;
the jaw yaw is chosen later for the arm. Candidate = dict(yaw_deg, s_mm, mid_mm, u0_mm, gap_mm, contact, face_mm, anti, cost) in the model frame."""
import argparse
import hashlib
import json
import pathlib
import sys

import numpy as np
import yaml

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from housing_grasp import cad_fit, connectors  # noqa: E402
from housing_grasp.model_grasp import search  # noqa: E402


def make(name, out=None, keep=200):
    cfg, spec = cad_fit.load_config(name)
    gr = yaml.safe_load((HERE / "config" / "gripper_scan.yaml").read_text())
    pad_w, pad_len = gr["finger_width_m"] * 1000, gr["finger_length_m"] * 1000
    axial = spec["symmetry"] == "axial"
    M, N = cad_fit.model_cloud(spec, 150000, u_range=spec["body_u_mm"])
    if axial:                                                       # side wall of the round body only: no end faces, no taper
        side = np.abs(N[:, 2]) < 0.3
        M, N = M[side], N[side]
    u0s = range(int(np.ceil(spec["seat_u_mm"] + cfg["place"]["clearance_mm"])), int(spec["seat_u_mm"] + 13))
    rows = search(M, N, pad_w=pad_w, pad_len=pad_len, u0s=u0s, yaw_step=180 if axial else 5, s_step=1.0, band=0.5)
    if axial:
        rows = [r for r in rows if abs(r["s_mm"]) < 0.01]           # pads centred on the round body
    if not rows:
        raise SystemExit("no candidate: check body_u_mm / seat_u_mm of " + name)
    rows = rows[:keep]
    out = out or connectors.ROOT / spec["candidates"]
    out.write_text(json.dumps(rows, indent=1) + "\n")
    step = connectors.ROOT / spec["step"]
    meta = dict(connector=name, step=spec["step"], step_sha256=hashlib.sha256(step.read_bytes()).hexdigest(), count=len(rows), best=rows[0], seat_u_mm=spec["seat_u_mm"],
                body_u_mm=spec["body_u_mm"], u0_choices_mm=list(u0s), pad_mm=[pad_w, pad_len], status="OFFLINE_GEOMETRY_ONLY",
                limitations=["scores are geometric: no friction or grip force", "panel clearance is checked again at run time against the measured plane"])
    out.with_suffix(".metadata.json").write_text(json.dumps(meta, indent=1) + "\n")
    print("%s: wrote %d candidates to %s; best %s" % (name, len(rows), out, {k: round(v, 2) for k, v in rows[0].items()}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="connector name (see config/connectors.yaml)")
    ap.add_argument("--all", action="store_true", help="every connector")
    ap.add_argument("--out", type=pathlib.Path, help="output file (one connector only)")
    ap.add_argument("--keep", type=int, default=200)
    ap.add_argument("--force", action="store_true", help="--all: overwrite candidate files that exist")
    a = ap.parse_args()
    if a.all == bool(a.name) or (a.all and a.out):
        ap.error("give one connector NAME (optionally --out), or --all")
    for name in (connectors.names() if a.all else [a.name]):
        target = connectors.ROOT / cad_fit.load_config(name)[1]["candidates"]
        if a.all and target.exists() and not a.force:
            print("%s: %s exists, kept (--force to rewrite)" % (name, target.name))
            continue
        make(name, a.out, a.keep)


if __name__ == "__main__":
    main()
