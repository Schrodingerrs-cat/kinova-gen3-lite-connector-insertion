#!/usr/bin/env python3
"""Place the CAD fixture (arm collision only) on the board measured in a close scan. Writes setup_cad/fixture_pose.yaml (backup .bak); the grasp script re-adds the scene.
  python3 fixture_from_board.py RUN_DIR [--connector aux]
Board centre, yaw and top come from vision.prior (white board around the marker colour). CAD strip centre (0.254, 0.0), long edge 90 deg, anchor (0.25, -0.05)."""
import argparse
import pathlib
import shutil
import sys

import numpy as np
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import connector  # noqa: E402
import plug_points as pp  # noqa: E402

def place(run, name="aux"):
    views = sorted((pathlib.Path(run) / "method2_perception/scan").glob("view_*.npz"))
    pr = pp.prior(views, connector.load(name)["marker"])
    f = pathlib.Path.home() / "kinova/work/setup_cad/fixture_pose.yaml"
    anchor = np.array([0.25, -0.05])
    yaw = pr["board_yaw"] - 90
    t = np.radians(yaw)
    R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    offset = pr["board_centre"] - (R @ (np.array([0.254, 0.0]) - anchor) + anchor)
    dz = pr["board_top"] + 0.023     # same relation as the earlier placement: top -0.018 <-> z offset 0.005
    shutil.copy(f, str(f) + ".bak")
    out = []
    for line in f.read_text().splitlines():
        if line.startswith("offset_m:"):
            line = "offset_m: [%.4f, %.4f, %.4f]" % (offset[0], offset[1], dz)
        elif line.startswith("yaw_deg:"):
            line = "yaw_deg: %.2f" % yaw
        out.append(line)
    f.write_text("\n".join(out) + "\n")
    print("collision fixture placed on the measured board: centre %s yaw %.2f top %.4f" % (np.round(pr["board_centre"], 4), pr["board_yaw"], pr["board_top"]), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=pathlib.Path)
    ap.add_argument("--connector", default="aux")
    a = ap.parse_args()
    place(a.run, a.connector)
