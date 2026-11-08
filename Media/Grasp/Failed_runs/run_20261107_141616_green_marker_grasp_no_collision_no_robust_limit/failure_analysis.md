# Failure analysis
1. Expected: the optimizer picks the best candidate once its hard limits are off.
2. Observed: the housing measurement itself failed (6.0 mm section); across the 4 plans from the same view pose the "housing" was 14.2 x 8.2 x 39.1, 14.2 x 8.3 x 38.6, 14.5 x 8.2 x 39.0 mm and then 6.0 mm long: unstable.
3. Evidence: `plan/*`, `Failed_runs/run_20261107_140835*/plots/close_frame_profile_retry.png`.
4. Cause: **Verified:** the measured shape (8 mm wide, 39 mm tall) does not match the visible head (black body ~18 x 24 mm top face, ~20 mm above the panel by the green top at z=+8 mm vs panel top ~-12 mm). **Suspected:** the 39 mm height is a depth streak (points rising 45 deg to z +50 mm in the single close frame) on the glossy black/teal head, not the head. **Unknown:** the streak's origin (specular depth error, D405 min range, motion).
5. Corrective action: NOT lowering more limits: grasping a mis-measured housing would use a wrong jaw opening / height. Next: inspect the raw depth image of the head (and at 25-30 cm), or define the grasp from the green sticker + a user-given head size.
6. Status: UNDER_INVESTIGATION

## Addendum (2026-11-07, later the same day; the text above is unchanged)
The image-mode validation (`Media/Grasp/run_20261107_image_mode_offline_validation/`) looked at the same close frame with the head mask lifted using each pixel's own depth. The dark body has valid depth (95% of its pixels) and forms ONE continuous steep surface with the sticker (z -5 .. +40 mm over about 20 mm in x). So the "39 mm tall housing" and the rising points are real surface in the data (cable rising from the head and/or a tilted head), not clearly a depth streak as suspected above. Still NOT isolated: whether the head is tilted in its slot or the cable lifts off it. The conclusion that the depth-measured housing was unreliable for this head stands; the explanation "depth streak" is now doubtful.

## Addendum 2 (2026-11-07, later still)
Further check on the same close frame: the green sticker points lie on a plane tilted 73 deg from horizontal (449/449 inliers) while the image shows a flat 12.5 x 9.5 mm sticker facing the camera. That favours a depth-ramp artefact over the head, i.e. it weakens the retraction in Addendum 1 (real surface) and supports the original suspicion (depth unreliable on this glossy head). Still NOT isolated (a tilted head or curved sticker would also fit). See `Media/Grasp/run_20261107_image_mode_offline_validation/README.md`.
