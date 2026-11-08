# Failure analysis

## 1. Expected behaviour
A collision-free antipodal grasp across the connector head, planned from one close frame whose ROI comes from the green sticker.

## 2. Observed failure
- Plan 1 (ROI 52x40 px = green box x1.6): `no grasp survives scoring`; rejections: 20 / 8 / 6 / 14 obstacle points in the finger volume (offsets 6-7 mm), then `fails in 40% of pose-error samples`.
- Plan 2 (ROI 132x96 px = green box x4): `all grasp candidates rejected`; 66-86 obstacle points in the finger volume for every candidate.
- Both measured the same housing: 14.2 x 8.2-8.3 x 38.6-39.1 mm (L x W x H), `tip_known: true`.

## 3. Supporting evidence
`plan/` and `retry_roi_x4/` (plan.json, metrics_pipeline.json, plots/02_segmentation_and_grasp_topview.png, color_with_roi.png), `plots/close_frame_profile_retry.png` (side views of the single close frame), `controller.log`.

## 4. Root cause
- **Verified:** the first ROI was too small (the planner needs the object in the central half of the ROI, 10x8 mm here). Growing it 4x did NOT change the measured housing, so the ROI size is not the main cause.
- **Verified (from the close-frame profile):** the head is not on the bare table. Two layers are visible, wood table at z ~ -48..-40 mm and the white panel top at z ~ -15..-10 mm (about 30 mm higher). The green sticker points span z -5..+20 mm although the sticker is flat (~13x10 mm), and a slanted streak of points rises at ~45 deg from the head to z +50 mm.
- **Suspected, not isolated:** (a) the panel, bolts, wooden post and cable put obstacle points inside the finger volume, because the head sits against the panel; (b) noisy depth on the glossy black / teal head (the z-spread of a flat sticker suggests it); (c) the 38.6 mm "height" mixes the panel height with the head height (support plane choice). The merged 15-view cloud cannot be used to check this (registration smear, 5-15 mm).
- **Unknown:** which of (a)-(c) dominates. Also unknown: the intended geometry of the "two green planes" (only ONE green sticker, on the top face, is visible in all views).

## 5. Corrective actions / next experiments
1. Put the head on the open table (no panel next to it) and rerun: isolates (a).
2. Inspect the raw depth image of the head at 18 cm and at 25-30 cm (D405 on glossy black plastic): isolates (b).
3. Report the housing height relative to the panel top, not the wood table.
4. Clarify the two green planes (see conversation): if the grasp must use two green faces, put green on BOTH side faces and grasp from tilted views.
Not done on purpose: relaxing the finger-volume check; that is the collision safety check.

## 6. Resolution status
UNDER_INVESTIGATION

## Addendum (2026-11-07, later the same day; the text above is unchanged)
The image-mode validation (`Media/Grasp/run_20261107_image_mode_offline_validation/`) looked at the same close frame with the head mask lifted using each pixel's own depth. The dark body has valid depth (95% of its pixels) and forms ONE continuous steep surface with the sticker (z -5 .. +40 mm over about 20 mm in x). So the "39 mm tall housing" and the rising points are real surface in the data (cable rising from the head and/or a tilted head), not clearly a depth streak as suspected above. Still NOT isolated: whether the head is tilted in its slot or the cable lifts off it. The conclusion that the depth-measured housing was unreliable for this head stands; the explanation "depth streak" is now doubtful.

## Addendum 2 (2026-11-07, later still)
Further check on the same close frame: the green sticker points lie on a plane tilted 73 deg from horizontal (449/449 inliers) while the image shows a flat 12.5 x 9.5 mm sticker facing the camera. That favours a depth-ramp artefact over the head, i.e. it weakens the retraction in Addendum 1 (real surface) and supports the original suspicion (depth unreliable on this glossy head). Still NOT isolated (a tilted head or curved sticker would also fit). See `Media/Grasp/run_20261107_image_mode_offline_validation/README.md`.
