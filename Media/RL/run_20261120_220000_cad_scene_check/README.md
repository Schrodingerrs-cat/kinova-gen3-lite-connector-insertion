# CAD scene check (2026-11-20)
Not a training run. Visual check that `CAD_files/Moveable arm.STEP` converts correctly to per-part meshes.
- Pipeline: `Sim/RL/tools/export_parts.py` (STEP -> 35 world-frame STLs in `Sim/RL/assets/cad_parts/`, metres, CAD frame, Y-up), `Sim/RL/tools/render_cad_scene.py` (MuJoCo static render, scene rotated so CAD Y is world up).
- Shows: Gen3 Lite on the flat base, plug cradle on the same surface (AC red, jack grey, USB-A blue), vertical wall with three sockets.
- In this CAD pose the gripper is at the USB-A socket (insertion pose).
- An earlier render of this run was tipped over (CAD rendered Z-up); replaced.
- Colours/lighting are for inspection only. Contact geometry, frames and clearances are still UNCALIBRATED.
