#!/usr/bin/env python3
"""One-off: split setup_cad/setup_assembly.glb into one STL per fixture part, in the robot base frame (metres, nominal CAD placement).
CAD axes (y up, arm along -z) -> base axes: x = -z, y = -x, z = y. The robot's own links (NAUO1 to NAUO7) are left out: RViz draws the real arm from its URDF.
   python3 export_setup_parts.py      writes setup_cad/parts/part_<id>.stl and setup_cad/parts_index.yaml
"""
import pathlib

import numpy as np
import trimesh
import yaml

D = pathlib.Path.home() / "kinova/work/setup_cad"
ARM = {"NAUO%d" % i for i in range(1, 8)}
M = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)
ACRYLIC = {13, 14, 15, 16, 19, 20, 23, 24, 25, 32, 34}
PLUGS = {49, 52, 53, 54, 56}                                          # plugs and sockets (also the "_1" copies on the board)
WHITE = {36, 40, 41, 42, 43, 47, 48, 50, 55}
sc = trimesh.load(D / "setup_assembly.glb")
index = []
for node in sc.graph.nodes_geometry:
    if node in ARM:
        continue
    T, g = sc.graph[node]
    m = sc.geometry[g].copy()
    m.apply_transform(T)
    m.vertices = m.vertices @ M.T
    num = int(node.replace("NAUO", "").split("_")[0])
    role, rgba = ("acrylic", (0.65, 0.82, 1.0, 0.35)) if num in ACRYLIC else ("plug_or_socket", (0.12, 0.12, 0.12, 1.0)) if num in PLUGS else ("white_part", (0.93, 0.93, 0.93, 1.0)) if num in WHITE else ("other", (0.55, 0.55, 0.58, 1.0))
    name = "part_%s.stl" % node
    m.export(D / "parts" / name)
    c, e = (m.bounds[0] + m.bounds[1]) / 2, m.bounds[1] - m.bounds[0]
    index.append(dict(id=node, file=name, role=role, rgba=list(rgba), centre_m=[round(float(v), 4) for v in c], size_m=[round(float(v), 4) for v in e]))
(D / "parts_index.yaml").write_text(yaml.safe_dump(index, sort_keys=False))
print(len(index), "parts written to", D / "parts")
