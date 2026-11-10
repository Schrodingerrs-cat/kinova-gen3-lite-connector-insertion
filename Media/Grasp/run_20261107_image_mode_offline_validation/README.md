# run_20261107_image_mode_offline_validation  (OFFLINE, no robot; PARTIAL_SUCCESS)

## A. Identification
- 2026-11-07, offline on the saved close-view frame of `run_20261107_142037_green_sticker_grasp/plan/*/frame.npz` (depth, colour, K, T_BC). No arm, no camera.
- Objective: check the new IMAGE MODE (`/grasp/head_grasp`, `grasp_mode:=head`) against the sticker-mode grasp that physically worked.
- Hypothesis: segmenting the head in the image (green + dark pixels connected to the sticker), lifting only those pixels to 3D with their own depth and synthesizing candidates on that cloud gives the same grasp as the sticker mode.
- Status: **PARTIAL_SUCCESS** (agrees offline; never executed on the arm).

## B. Implementation
`housing_grasp/green.py`: `head_mask` (image-space segmentation), `head_points` (mask pixels lifted with their own depth, wide height band +-60 mm, no flying-pixel filter), `synthesize_head_grasps` (PCA of the head footprint -> jaw axis across the head; 5 pad positions +-6 mm around the sticker; opening = measured width in the pad window + 2 x 12 mm; TCP 8 mm below the sticker height in that window; candidates whose height differs by > 4 mm from the best are dropped). Config: `green.*` in `config/grasp.yaml` (copied in the run `config` of the node run).

## C. Results (same frame)
| | sticker mode (worked) | image mode, best candidate |
|---|---|---|
| TCP (mm) | (256.6, -28.9, -0.4) | (256.7, -29.4, -1.2) |
| jaw axis | (-0.21, 0.98) | (-0.16, 0.99), 2.9 deg apart |
| width / opening | 13.4 / 37.4 mm | 12.5 / 36.5 mm |
Head cloud: 1,094 points, xy extent 22.7 x 16.0 mm; 1 candidate survives the flatness filter (the +-3 / +-6 mm positions sit where the surface rises 8-11 mm).

## D. Evidence
`plots/01_head_image_mode_grasp_topview.png`; the 3D plot of the head mask (sticker and dark body lifted with their own depth) is in the conversation only, regenerate with the code above.

## E. Observations / conclusions
- First attempt FAILED offline and was fixed: a height band of -40/+6 mm around the sticker (meant to remove depth streaks) cut away the black body, leaving only the sticker, whose long side runs across the head: PCA gave the jaw along the cable, 72 deg wrong. The dark body is real data on this head: it forms one continuous steep surface with the sticker (z -5 .. +40 mm over ~20 mm in x), so the band was widened and the grasp height taken from the sticker points.
- Correction of an earlier statement: the "39 mm tall housing" in the depth planner runs was attributed to a depth streak; this data shows real surface there (cable / tilted head), the cause is still not isolated.
- Not proven: execution on the arm (camera and driver were down), other heads, the effect of the 8 mm drop. Width is ~1 mm short vs the colour footprint (percentile trimming).
- Next: run `run_green_pipeline.py --mode head --grasp` once the arm and camera are back.

## Addendum 1 (2026-11-07, later): CLOUD MODE (green region only, candidate search), same frame
`/grasp/cloud_grasp` (`--mode cloud`): sticker axes/size from the green blob's min-area rectangle in the IMAGE (12.5 x 9.5 mm, fill 0.87), position/height from the median of the green 3D points; 30 candidates (2 jaw edges x 3 yaw offsets x 5 pad positions), ranked. Best vs the grasp that worked: TCP (256.0, -29.3, -0.4) vs (256.6, -28.9, -0.4) mm = (-0.6, -0.4, 0.0) mm; jaw axis 10.7 deg apart (the sticker edge is not exactly aligned with the head footprint; the +-10 deg yaw candidates cover it); opening 36.5 vs 37.4 mm. Plot: `plots/01_green_cloud_candidates_topview.png`. Needs `green.jaw_edge: long` (a convention, see the main README); on this frame the short-edge candidates are ranked lower and would put the jaws along the cable. Offline only, never executed on the arm.

## Addendum 2: corrections to numbers quoted earlier today
- "The 3D sticker cloud has a 39 mm PCA extent": WRONG cause. 9 stray green points on the panel at about (277, -5) mm stretched it; without them the extents are 15.4 x 11.4 mm, consistent with the image.
- Still true: the sticker points lie on a plane tilted 73 deg from horizontal (449 of 449 inliers, z -5.7 .. +21 mm over about 11 mm in xy) although the image shows a flat 12.5 x 9.5 mm rectangle facing the camera. Both cannot be right; this favours a depth-ramp artefact over the head (my first explanation) and weakens the "real surface" retraction written above. NOT proven: a tilted head or a curved sticker would also explain the depth. The image-based axes avoid relying on it.

## Addendum 3: face rule and the candidate-search switch (same frame, offline)
| search | faces rule | extents from | candidates | jaw vs the grasp that worked | pads touch faces |
|---|---|---|---|---|---|
| ON | edge_prior (long) | sticker | 30 | 11 deg | 9.5 mm wide |
| ON | larger | sticker | 30 | 79 deg | 12.5 mm |
| ON | larger | head outline | 30 | 0 deg | 24.7 mm |
| ON | smaller | head outline | 30 | 90 deg | 13.4 mm |
| OFF | (same four) | | 1 | identical to the best of the ON list | |
| head mode | ON / OFF | | 1 / 1 | 3 deg | |
TCP differs from the grasp that worked by (-0.6, -0.4, 0.0) mm in every cloud-mode row. The grasp that worked touched the two long side faces (24.7 mm wide), the larger-area pair reachable with a top-down approach. The sticker's proportions are opposite to the head's, so a "larger faces" rule computed from the sticker alone points the jaw along the cable.

## Addendum 4: the image mode keeps a real candidate search (same frame, offline)
Earlier the head mode's hard flatness filter left ONE candidate (search ON = OFF). Now: yaw offsets are searched too and height changes are a ranking penalty (`head_height_rule: penalize`), `drop` keeps the old behaviour.
| mode | search | candidates | best (yaw / offset) | jaw vs the grasp that worked | TCP diff (mm) |
|---|---|---|---|---|---|
| image (head), penalize | ON | 9 | 0 deg / 0 mm | 3 deg | (0.1, -0.5, -0.7) |
| image (head), penalize | OFF | 1 | 0 / 0 | 3 deg | same |
| image (head), drop | ON | 3 | 0 / 0 | 3 deg | same |
| cloud | ON | 30 | 0 / 0 | 11 deg | (-0.6, -0.4, 0.0) |
| cloud | OFF | 1 | 0 / 0 | 11 deg | same |
The first run with `yaw_penalty` 0.1 ranked a -10 deg yaw first (13 deg from the grasp that worked) on a 0.015 difference in width evenness, i.e. noise; `yaw_penalty` is now 0.3 so a yaw offset must win by clear evidence. Offline only; never executed on the arm.

## Addendum 5 (2026-11-09): grid search in the image mode, no on/off switch (same frame, offline)
The head mode now generates its candidates from the head cloud (`grid_grasp_search`, `PIPELINE.md` 5.2): 268 grid points (1 mm) x 36 yaws (5 deg) = 9648 candidates, 2118 valid. The jaw-centre filter removed 7529; the opening, contact and finger-collision filters removed none on this frame, so those three are untested on real data (the collision filter is covered by a synthetic test only).
| | grasp that worked | grid search, best |
|---|---|---|
| TCP (mm) | (256.6, -28.9, -0.4) | (256.7, -29.1, -0.4) |
| jaw yaw | 102 deg | 105 deg |
| gap / opening | 13.4 / 37.4 mm | 12.0 / 36.0 mm |
Figures in `plots_grid_search/`: 3D normals, outline normals, all candidates, grasp points in 3D, candidate counts, best cost per yaw.
Found while building it: scoring the contacts with the 3D PCA normals put the best yaw at 130 deg, 28 deg off the working grasp (antipodal score 0.41 for every yaw), because the head cloud's depth ramps from -5 to +39 mm and the normals follow the ramp. Horizontal outline normals fix that (best 105 deg, antipodal 0.77). A per-window sticker height moved the TCP z to +8.8 mm, 9 mm above the working grasp, so the sticker median height is used. A narrow-gap term (`w_width`) was added because nothing else prefers the 12 mm direction over the 25 mm direction on a clean box. Offline only; never executed on the arm.
