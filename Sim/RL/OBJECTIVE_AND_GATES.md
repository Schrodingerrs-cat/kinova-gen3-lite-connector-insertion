# Objective, gates, and iteration budget (Task B)

## Objective (what "done" means)
Deploy on the real Gen3 Lite: pick USB-A, jack, AC from the cradle (plug standing in its hole, cable loose) and insert each into its wall socket, with two learned policies (grasp, insertion), each = frozen base + RLT residual. Sim decides whether deployment is allowed (>= 90% verified success per connector on held-out episodes of the final stage); the real arm decides any sim-to-real claim. See `plan/Task_B_Isaac_Sim_RL_PLAN.md`, `DESIGN.md`.

## Scope change (user, 2026-11-20): insertion policy only, camera-only search + servo + insert
Grasp learning is dropped for now. Insertion episodes start with the plug held (weld allowed, user decision) at the CAD pose: plug axis along the tool approach axis, cable trailing back and drooping, grasp offset randomised around it. Only the eye-in-hand camera is available: the socket position is **not given**; the policy must find the board, servo to it and insert. Gates G3 (grasp) and the grasp-related part of G4 are deferred; new gates G3' (camera + markers render/detect, board pose randomised) and G4' (privileged scripted search/servo/insert baseline) replace them.

## Rule: no policy training until G0-G5 pass
Training is the last step, not the debugging tool. Every gate has a scripted, repeatable check and a written pass criterion fixed here before the check is run. A failed gate is fixed and re-run; the criterion is not relaxed to pass.

| Gate | Check | Pass criterion | Status |
|---|---|---|---|
| G0 geometry | MuJoCo plug/socket/wall dims vs CAD sections; overlay render | tip cross-sections and socket axes within 0.3 mm of CAD; clearance marked UNCALIBRATED until measured | partial: dims from CAD sections, clearance 0.5 mm per side is a guess |
| G1 contact | 3 adversarial scenes x 3 connectors (`tests/test_contact_scenes.py`) | aligned seats to within tolerance; 4 mm misaligned and blocked never enter; no tunneling; penetration explained | **open**: scenes discriminate, ~0.86 mm penetration unexplained, seat tolerance not fixed, insertion resistance not modelled (USB datasheets: <= 35 N to insert, >= 10 N to extract) |
| G2 robot | Gen3 Lite model vs CAD pose (`tools/fit_cad_pose.py`); IK/controller reaches standoffs | link meshes fit CAD pose with mean error < 2 mm; tool reaches all 3 socket standoffs and cradle with joint limits respected | **partial, not passed**: base/shoulder/arm/forearm 0.6-1.2 mm (base placement confirmed), gripper 2.1 mm, lower wrist 3.3 mm, upper wrist 8.4 mm. Cause of wrist misfit unresolved (CAD wrist simplified or different from URDF mesh, or CAD not an exact kinematic pose). IK reach not yet tested |
| G3 grasp | finger-contact grasp of standing plug, no weld/attach ever | scripted grasp holds through lift + rotate + 35 N push + cable drag over friction DR; >= 95% hold on 100 DR episodes | not started; gripper force/stroke TBD (not found online, measure or read Kinova docs) |
| G4 planner (B0) | scripted grasp->insert chain, per connector, DR on | >= 70% on 100 held-out DR episodes (shows task is feasible and gives RL a signal); if lower, fix sim or planner, not the policy | not started |
| G5 observations | wrist camera render + the same feature code as the real robot | features valid when the real ones are; D405 intrinsics and T_EC from calibration (TBD) | not started |

## Policy iteration budget (hard limit)
- Base policy: DAgger against the planner. Must reach >= 5-10% success per phase before RL starts (RLT needs a signal); otherwise return to G3/G4.
- RLT residual: at most **2 iterations** per policy. Each iteration starts with a written hypothesis, the exact reward/hyperparameters, and the predicted curve; ends with held-out evaluation and a failure-cause table. Iteration 2 changes only what the failure table points at. If it still misses, stop and report evidence (CLAUDE.md rule 6); no open-ended tuning.
- Reward stays sparse terminal (DESIGN.md Sec. 5). No dense reward unless explicitly labelled as an ablation.

## Path to the real robot (after sim >= 90%)
Offline replay of logged real observations through the policy, shadow mode (policy proposes, scripted controller acts), then supervised trials with e-stop and an independent seating sensor (plan gates B-G5/B-G6). Hardware constants (gripper force, intrinsics, hand-eye, cable stiffness, true clearances) must be measured before G3-G5 can be called calibrated.
