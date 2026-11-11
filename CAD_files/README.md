# CAD_files

Geometry of the connector heads and the panel, used offline to plan grasps. Nothing here is read at run time yet.

## What goes here
| File | What | Needed for |
|---|---|---|
| `*.step` / `*.stp` | exact CAD of the connector head (the molded housing, not only the metal plug) | exact widths and wall angles |
| `*.glb` / `*.obj` / `*.ply` | photo scan (Polycam) of the head | shape when no CAD of the molded housing exists |
| `measurements.md` | caliper numbers, see below | scaling and checking every mesh |

Name files by connector and source, for example `usb_a_head_polycam.glb`, `usb_a_head_vendor.step`.

## Caliper numbers to write in `measurements.md`
1. Top face length (long edge) and width, in mm.
2. Head height above the panel, in mm.
3. Slot depth in the panel and its clearance, in mm.
4. Where the sticker goes on the top face (position and which edge lies along the long axis).

A photo scan has no reliable size. Scale it to one caliper number and check it with a second one. A CAD file must match the real part: compare its outer length and width to the calipers before trusting it.

## Photo scan tips (Polycam, photo mode)
- 60 to 100 stills, 60 to 70% overlap, 2 or 3 rings (low, about 45 degrees, from above). The low ring captures the side walls, which are the faces that matter.
- 15 to 25 cm away, soft even light, matte plain surface, head not moved while shooting.
- No sticker on the head during the scan.

## Status
Empty so far (2026-11-10). Large files: decide whether to commit them, ignore them in `.gitignore`, or use Git LFS, before adding anything over a few MB.

## USB A Plug (added 2026-11-10)
- `USB A Plug.step`: SolidWorks 2025 export from a teammate (units mm). `USB_A_Plug.glb`: our conversion (`cascadio`, metres), 31,566 triangles; regenerate with `cascadio.step_to_glb`.
- Size 15.0 x 7.5 x 50.25 mm, long axis = z. Cross-section along it (read from the mesh): metal USB-A shell z +10..+22 mm, 12.0 x 4.5 mm; moulded housing z -10..+10 mm, 15.0 x 7.5 mm (the part to grip); strain-relief boot z -18..-12 mm, 9.8 x 7.5 mm; cable stub z -28..-18 mm, about 6 mm round.
- Seated in the panel socket the metal shell is inside the panel, so the housing starts at the panel surface: housing 0..20 mm above the panel, boot 22..28, cable to 38 (the first depth-based runs measured 14.2-14.5 x 8.2-8.3 x 38.6-39 mm).
- `USB_A_Plug_grasp_candidates_top200.json`: best 200 candidates of `housing_grasp/model_grasp.py` in the upright plug frame (x, y across, u = height above the panel; jaw yaw, pad centre s, tip height u0). Best: jaw along y (pads on the 15 x 20 mm wide faces, gap 7.5 mm), centred, tip 3 mm above the panel; the narrow-face family (gap 15 mm) is far behind. Offline only; needs the plug pose (x, y, yaw) to be used.
