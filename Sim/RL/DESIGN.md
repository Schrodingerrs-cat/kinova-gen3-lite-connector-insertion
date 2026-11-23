# RL design: grasp policy + insertion policy, 3 connectors, RLT on each

Governing spec: `plan/Task_B_Isaac_Sim_RL_PLAN.md` (Amendment 2026-11-20). World z-up, metres, base at origin, wall front face y = 0.550, insertion direction +y. CAD (x,y,z) maps to world (x,-z,y). Everything marked UNCALIBRATED is a placeholder until measured.

## 1. What the literature says, and what it changes

| Source | Finding | Consequence here |
|---|---|---|
| RLT (Physical Intelligence, [arXiv 2604.23073](https://arxiv.org/html/2604.23073v1)) | Freeze the VLA; add an "RL token" (encoder-decoder bottleneck on its final embeddings, trained by reconstruction); train a tiny actor/critic on token + proprio. Chunked TD3-style, chunk C=10 vs VLA chunk H=50, sparse binary reward, actor gets the VLA reference chunk as input, loss `-Q + beta*||a - a_ref||^2`, reference dropout, warm-up rollouts of the base policy, 2 critic steps per actor step, high UTD. Needs ~400-1000 episodes (15 min to 5 h of robot time). Removing the BC term gave the biggest drop; single-step actions could not match the base. | Use an **off-policy chunked TD3 with BC regulariser**, not PPO, for B2. Sample budget is hundreds of episodes, so a single-env MuJoCo with rendering is enough for the first results. Residual starts at zero so it can never start worse than the base. |
| RLT scope | RL controls only the **critical phase** (5-20 s); a human picks the handover and gives the success label. | We have no VLA and no human: the **sim gives exact success labels and exact phase boundaries**, which is what lets us extend RLT to the whole task. |
| RouteRLT ([arXiv 2609.26467](https://arxiv.org/html/2609.26467v1)) | Frozen base + phase-specific RL specialists + a router (phase classifier on the base's latent, hysteresis stabiliser). Specialists trained per phase on sparse success. LIBERO: routed specialists 92.2% vs 85.0% base; **one full-trajectory RL specialist was worse (88.9%)**. Real cable pickup + 1 mm port insertion: 6.7% -> 35.0% (n=20-30, small). Isolated insertion specialist 62.1% vs 8.0% base. | End-to-end here means **a chain of phase specialists (grasp, transport, insert) behind a router**, not one monolithic RL policy. This promotes the router from optional to core for the end-to-end task. |
| Residual RL for tight insertion ([arXiv 2505.11858](https://arxiv.org/html/2505.11858v1)) | Sparse goal reward only, plus an interpenetration penalty because the simulator can be exploited; curriculum on pose-noise (up while success > 75%, down below 50%) and on residual magnitude; zero-shot sim-to-real. | Keep sparse reward. Interpenetration is a **termination** here (plan rule: log safety terminations, do not shape). Curriculum thresholds adopted (promote at >=70-75%). |
| IndustReal ([arXiv 2305.17110](https://arxiv.org/pdf/2305.17110.pdf)) | Dense SDF-based rewards, sampling-based curriculum, policy-level action integrator; 83-99% real over 600 trials. | Reference for the dense-reward ablation and for the action integrator (our delta-pose action with accumulation). Not the main reward. |

Caveats: RLT and RouteRLT numbers above come from page extracts, some from a model summary, and RouteRLT's hardware sample sizes are small with no significance test. Their base is a pretrained VLA; ours is a small policy trained in sim, so the transfer of their results is an assumption to test, not a given.

## 2. Task and phases
Episode: plug of connector `c` stands vertically in a hole of the cradle holder; the cable is NOT attached to anything (user, 2026-11-20): it hangs/lies loose from the plug's rear and is dragged by friction. Tip-up orientation is an assumption to confirm. Order USB-A, jack, AC; connector id is an input. Phases (labels come from sim, used for the router and diagnostics, never as reward):
`P1 approach+grasp` -> `P2 lift+reorient (roll to socket orientation) + transport` -> `P3 align at standoff + insert`.

## 2b. Insertion-only, camera-only (user, 2026-11-20; supersedes the two-policy split for now)
No predefined socket pose. Board/wall pose is randomised over a reachable region and the arm starts where the sockets may be out of view. Stages inside one policy: search (scan until markers seen) -> servo (centre and approach to standoff) -> insert (contact phase; plug occludes the socket, so history is needed). Plug held by weld at the CAD pose (plug axis = tool approach axis, cable trailing back and drooping), grasp offset randomised. Pipeline: privileged teacher (knows socket pose; scripted scan + servo + guarded insert) -> DAgger student on marker features from the same detector as the real robot -> frozen -> RLT residual on the contact phase, sparse reward, max 2 iterations.

## 3. Observations and actions
- Actor (deployable): wrist-camera features (socket markers + validity, plug-in-view features, depth summary + validity), q, dq, gripper opening, previous action, connector one-hot. No phase bit from the sim.
- Critic (privileged, training only): plug pose, socket pose, grasp offset, contact forces, cable state. Faithful-RLT critic (token + proprio only) is an ablation.
- Action: end-effector delta pose `[dv(3), dw(3)]` + gripper command, clipped, integrated, differential IK to joint targets, 500 Hz PD. Control 20 Hz; base chunk H=20; RL chunk C=5-10.

## 4. Policy stack (user decision 2026-11-20: two separate policies, trained side by side, not one)
Two independent policies, each with its own planner demos, base policy, RL token and RLT residual. They share the scene, the cable/contact model and the domain randomisation, nothing else.

| | Grasp policy | Insertion policy |
|---|---|---|
| Start | plug standing in the cradle hole, gripper open at standoff | **plug already held in the gripper, cable hanging free**, pose in hand drawn from the measured grasp-offset distribution, near the wall standoff; later widened to start from the grasp policy's own end states |
| Ends | plug lifted out, rotated to horizontal, held in an insertable pose | verified insertion |
| Reward (sparse) | see Sec. 5 | see Sec. 5 |
| Cable | hangs/lies loose, drags on lift | hangs from the held plug, drags during insertion |

Handoff is a fixed rule, not a learned router: when the grasp policy's success condition holds (lifted, held, in the insertable envelope) the insertion policy takes over. This replaces the router of the earlier draft (simpler; one fewer thing to fail). A learned router can return later if the fixed rule proves brittle.

Per policy:
1. **Planner** (privileged, sim only): grasp candidates from `Code/Grasp` for the grasp policy; aligned-approach + guarded insert for the insertion policy. Provides demos and baseline B0.
2. **Base policy**: small vision/proprio chunked policy trained by **DAgger against the planner** with noise injection and perturbed starts.
3. **RL token**: bottleneck on the base's final embeddings, reconstruction loss, frozen with the base (RLT Alg. 1).
4. **RLT residual**: `u = a_ref + delta*tanh(pi(token, proprio, a_ref))`, zero-initialised, chunked TD3, `-Q + beta*||a - a_ref||^2`, reference dropout 50%, warm-up with base rollouts, 2 critic updates per actor update.
5. **Chain evaluation:** run grasp then insertion with the fixed handoff on held-out episodes; this is the end-to-end number that must clear the >= 90% gate per connector. Optional last step: joint fine-tune of the pair on the full-task terminal reward.

Variants on the same held-out set: B0 planner chain, B1 TD3/SAC from scratch per policy (same chunking and budget; PPO at scale in Isaac as an extra), B2 base + RLT per policy (main result).

## Domain randomisation (user: only cable drag and contact physics, ranges from 0 up to a max)
Implemented in `Sim/RL/scene.py` (`DR`, `sample_dr`); upper bounds are UNCALIBRATED placeholders until the cable is measured.
| Parameter | Range |
|---|---|
| cable bending stiffness | 0 to 0.05 |
| cable joint damping | 0 to 0.01 |
| cable mass (total, kg) | 0.0005 to 0.06 (kept above 0 so the chain stays well-posed) |
| cable friction | 0 to 1.0 |
| plug/socket contact friction | 0.1 to 0.8 |
Not randomised here: clearance (curriculum stage), grasp offset (its own distribution), camera, lighting, delays.

## 5. Reward
All rewards sparse terminal, computed from privileged sim geometry, never from the actor's observations.
- **Insert specialist / full task:** `r=1` iff tip depth >= target, lateral and angular error within tolerance, held for a dwell, no forbidden penetration. Else 0.
- **Grasp specialist:** `r=1` iff, after lift to a clear height and a dwell, the plug is held with the **plug-in-hand pose inside the insertable envelope** (tip offset and tilt within what the insert specialist can handle, cable not pinched, no slip). This bridges your concern that a grasp is only good if insertion works: the envelope is defined from insertion feasibility, then checked again by the end-to-end reward in step 6.
- **Terminations (logged by cause, reward 0):** bounds, jam, drop, cable overstretch, interpenetration beyond threshold, timeout.
- Diagnostics only: grasp held, lifted clear, aligned at standoff, tip in cavity mouth.
- Dense-shaped (e.g. SDF) version only as a labelled ablation.

## 6. Gating (what must be true before RL starts and before a claim)
- Per phase, **base success must be non-trivial** (target >= 5-10%) before RL, otherwise sparse RL has no signal. If not, fix the base (more DAgger, perturbations); do not add dense reward silently.
- Physics gates from the plan: three adversarial contact scenes pass, no tunneling, cavity open.
- Curriculum promote at >= 70% on 100 held-out seeds, same threshold every stage: C0 grasped near standoff, wide clearance -> C1 nominal clearance -> C2 plug on cradle, short cable -> C3 full cable drag + grasp randomisation -> C4 three connectors mixed. Hardware gate stays >= 90% held-out on C4 per connector.
- Report n, intervals, seeds (>= 3), every failure by cause.

## 7. Risks
- Long horizon with one terminal reward: mitigated by phase specialists, zero-init bounded residual, curriculum; not eliminated.
- Router errors near phase boundaries (RouteRLT's own failure analysis): evaluate against oracle routing.
- Vision sim-to-real gap for the RL token: validated in plan stage C4; real-robot fine-tuning, as RLT does, would need supervised, e-stop-gated trials and is not assumed.
- Cable stiffness, mass and how it hangs through the hole are guesses until measured.
