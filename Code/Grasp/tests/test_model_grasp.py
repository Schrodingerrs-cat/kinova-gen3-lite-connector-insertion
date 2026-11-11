import pathlib
import sys

import numpy as np
import trimesh

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp.model_grasp import plug_frame, search  # noqa: E402


def test_box_plug_best_grasp_is_on_the_wide_faces_low_and_centred():
    box = trimesh.creation.box(extents=[0.015, 0.0075, 0.020])   # metres, axis z from -10 to +10 mm; panel at z = +10 mm -> u = 0..20 mm
    P, N = plug_frame(box, panel_z_mm=10.0)
    c = search(P, N)
    b = c[0]
    assert abs(b["yaw_deg"] - 90) < 1e-6 and abs(b["gap_mm"] - 7.5) < 0.2        # jaw across the 7.5 mm thickness: pads on the 15 x 20 mm faces
    assert b["u0_mm"] == 3 and abs(b["s_mm"]) <= 1                                # as deep as the 3 mm clearance allows, centred on the face
    narrow = min(q["cost"] for q in c if q["yaw_deg"] == 0)                       # jaw across the 15 mm width: the narrow faces
    assert b["cost"] < narrow and b["face_mm"] > 14
