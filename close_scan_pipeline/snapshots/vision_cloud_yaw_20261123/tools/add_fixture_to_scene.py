#!/usr/bin/env python3
"""Puts the CAD fixture and the table into MoveIt's planning scene so collision checks see them. Moves nothing.

  python3 ~/kinova/work/tools/add_fixture_to_scene.py           add / replace (run again after editing setup_cad/fixture_pose.yaml)
  python3 ~/kinova/work/tools/add_fixture_to_scene.py --remove  take them out again

Added: every CAD part except the target plugs (role plug_or_socket: the thing being grasped) and the arm mount (centre x < 0.12: it touches the arm base by design),
plus a table slab with its top at z = -0.041 m. Object ids start with fixture_ and table. The scene lives in move_group: it is lost when MoveIt restarts
(run_stack_method2.sh re-adds it). The CAD placement is the nominal design, the real cradle can be a centimetre or so off, so treat a collision verdict near the fixture as approximate.
"""
import argparse
import pathlib
import sys
import time

import numpy as np
import rclpy
import trimesh
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Point

D = pathlib.Path.home() / "kinova/work/setup_cad"
TABLE_TOP = -0.041
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--remove", action="store_true")
ap.add_argument("--include-plugs", action="store_true", help="also add the plug/socket parts as convex hulls (for scans: keeps the fingers off the plug). Leave out for a grasp.")
args = ap.parse_args()

rclpy.init()
node = Node("add_fixture_to_scene")
cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
if not cli.wait_for_service(timeout_sec=10):
    sys.exit("/apply_planning_scene not available: is MoveIt (move_group) running?")

parts = yaml.safe_load(open(D / "parts_index.yaml"))
pose = yaml.safe_load(open(D / "fixture_pose.yaml"))
c, t, yaw = np.array(pose["anchor_m"], float), np.array(pose["offset_m"], float), float(pose["yaw_deg"])
R = Rot.from_euler("z", yaw, degrees=True).as_matrix()
shift = c - R @ c + t

scene = PlanningScene()
scene.is_diff = True
ids = []
for p in parts:
    if p["centre_m"][0] < 0.12 or (p["role"] == "plug_or_socket" and not args.include_plugs):
        continue
    ids.append(("plughull_" if p["role"] == "plug_or_socket" else "fixture_") + p["id"])
ids.append("table")

if args.remove:
    for name in ids:
        o = CollisionObject()
        o.id, o.operation = name, CollisionObject.REMOVE
        o.header.frame_id = "base_link"
        scene.world.collision_objects.append(o)
else:
    for p in parts:
        if p["centre_m"][0] < 0.12 or (p["role"] == "plug_or_socket" and not args.include_plugs):
            continue
        m = trimesh.load(D / "parts" / p["file"])
        if p["role"] == "plug_or_socket":
            m = m.convex_hull
        V = m.vertices @ R.T + shift
        mesh = Mesh()
        mesh.vertices = [Point(x=float(a), y=float(b), z=float(z)) for a, b, z in V]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in f]) for f in m.faces]
        o = CollisionObject()
        o.id, o.operation = ("plughull_" if p["role"] == "plug_or_socket" else "fixture_") + p["id"], CollisionObject.ADD
        o.header.frame_id = "base_link"
        o.meshes.append(mesh)
        o.mesh_poses.append(Pose())
        o.mesh_poses[0].orientation.w = 1.0
        scene.world.collision_objects.append(o)
    slab = CollisionObject()
    slab.id, slab.operation = "table", CollisionObject.ADD
    slab.header.frame_id = "base_link"
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.6, 1.6, 0.02])
    slab.primitives.append(box)
    sp = Pose()
    sp.position.x, sp.position.y, sp.position.z, sp.orientation.w = 0.3, 0.0, TABLE_TOP - 0.01, 1.0
    slab.primitive_poses.append(sp)
    scene.world.collision_objects.append(slab)

req = ApplyPlanningScene.Request()
req.scene = scene
fut = cli.call_async(req)
t0 = time.time()
while not fut.done() and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.1)
print(("removed" if args.remove else "applied") + " %d objects: %s (%.1f s)" % (len(scene.world.collision_objects), "ok" if fut.done() and fut.result().success else "FAILED or timed out", time.time() - t0), flush=True)
node.destroy_node()
import os
os._exit(0)
