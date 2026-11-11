# Archived: the sticker mode (2026-11-10)

**What it was.** The default grasp mode until 2026-11-10 (`/grasp/sticker_grasp`, `run_pipeline.py --mode sticker`): the sticker's median 3D position gave the grasp point, the dark housing outline around it (from the colour image, back-projected at the sticker's depth) gave the jaw axis and the width; the opening was the width plus 12 mm per side; the fingertips went `grasp_drop_m` (8 mm) below the sticker top. One grasp plus a jaw-flipped copy, no candidate search.

**Why it is archived.** It only works for this sticker and this head: the jaw axis comes from the outline measured around the sticker, and the sizes (footprint 24.7 x 13.4 mm) are measured, not the part's geometry. A different sticker shape or a different cable needs the code changed. The head mode (grid search on the head cloud, with the STEP model of the plug to follow) replaces it as the default; the cloud mode stays as the other option.

**Results with it (real arm, both n = 1, not tugged):**
- 2026-11-07 `Media/Grasp/run_20261107_142037_green_sticker_grasp`: lifted the connector out of the panel.
- 2026-11-10 `Media/Grasp/run_20261110_132457_end_to_end_autonomous`: wrist camera held, finger check no.
- The user observed that this mode grasped from the narrower faces, the head mode from the wider ones (not measured).

**Files here.** `sticker_grasp_method.py` (the node method), `report_sticker_grasp.py` (its plot). `housing_footprint` stays in `housing_grasp/color_marker.py` because the cloud mode's `extent_source: footprint` uses it. `export_evidence.py` still reads the sticker-mode plan format so the old runs can be exported.
