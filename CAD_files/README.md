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
