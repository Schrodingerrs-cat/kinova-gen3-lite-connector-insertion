#!/usr/bin/env python3
"""python3 tools/grasp_overlay.py RUN_DIR_OR_plan.json [...]: the grasp overlay (see housing_grasp/grasp_overlay.py) for every plan in a run folder or for one plan.json."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp.grasp_overlay import overlay  # noqa: E402

for arg in sys.argv[1:]:
    a = pathlib.Path(arg)
    for pj in [a] if a.suffix == ".json" else sorted(a.glob("**/plan/*/plan.json")):
        r = overlay(pj)
        print(pj.parent.name, "->", "no overlay (no candidates / frames)" if r is None else "%s (jaw %s from the crib axis)" % (r[0], "n/a" if r[1] is None else "%.0f deg" % r[1]))
