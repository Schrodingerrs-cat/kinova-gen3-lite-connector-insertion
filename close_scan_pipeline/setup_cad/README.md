# Setup CAD (assembly of the arm and the connector fixtures)

`setup_assembly.STEP` (SolidWorks 2025 export, millimetres, copied from Downloads/WPI/VBRM/"Moveable arm.STEP"), `setup_assembly.glb` (mesh, metres, part names are generic NAUO ids), `layout_views.png` (three views with the ids).

CAD axes: y up, the arm reaches along -z. Robot base frame: x forward = -z_cad, y left = -x_cad, z up = y_cad (origin on the robot base axis, z = 0 at the base mounting plane).

Designed layout in the robot base frame (mm), from the CAD:
- cradle with three plugs at x = 250: USB-A at y = -50 (z about -5, body 15 x 7 x 50, axis vertical), audio at y = 0, AC at y = +50; cradle block top at z about -23
- upright socket board at x = 550: plate 250 x 250 x 3 centred at z = 72; three socket plates (52 x 52 x 5) at y = -75, 0, +75, z = 127; sockets along x
- plate under the robot base 110 x 110 x 53 (bottom at z = -53)
- robot links NAUO1 to NAUO7 (posed in one configuration; not usable as the current arm pose)

Measured on the real arm (2026-11-19): USB plug model origin about (270.7, -15.0, -6.8), panel plane z about -26, plug top z about +26 to +29. That differs from the CAD (250, -50) by about 20 mm in x and 35 mm in y:
either the real cradle is not where the CAD puts it, or the CAD to base mapping is off; to be settled with a ruler measurement. Use the CAD for shapes and spacing until then.
