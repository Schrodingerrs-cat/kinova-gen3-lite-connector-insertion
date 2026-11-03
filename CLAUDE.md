# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.



## Success Criteria — NON-NEGOTIABLE

**A controller is NOT considered successful merely because numerical metrics look good. It MUST demonstrably work in the actual system it is claimed for.** Which system is the final authority depends on the task:

| Task | Final authority | Proof of success |
|---|---|---|
| **Task A (IBVS + search)** | **The real Kinova arm.** A simulation implementation is optional; if built, it supports development and never substitutes for hardware results. | Video and logs of closed-loop runs on the arm, with the independent seating sensor reporting success (never robot pose or depth alone), plus metrics from the same runs. |
| **Task B (Isaac Sim/Lab RL)** | **Simulation first**, then hardware. Simulation decides whether the policy may be deployed (≥90% verified success on held-out episodes of the final curriculum stage, not the initial/easy stages). Hardware decides whether any sim-to-real claim can be made. | Sim: reproducible rollouts with screen recordings, metrics, and held-out evaluation. Hardware: supervised trials with independent seating verification. |

Numerical metrics are required to support the demonstration, not replace it.

### Mandatory Requirements

1. **The task's final authority decides (see table).** Every controller implementation must be validated in its authoritative system. Show the controller operating, the system responding, and the intended behavior being achieved. Provide a reproducible run and visual evidence (for simulation, a screen recording whenever rendering is available; for hardware, video of the arm). Hardware runs must also follow the plans' safety gates (e-stop, supervisor, independent seating sensor). A simulation result never counts as hardware success, and a hardware result never replaces simulation validation for Task B.

2. **No shortcuts. No cheating. No manufactured success.** Never manipulate simulation physics, initial conditions, contact parameters, evaluation metrics, reference trajectories, or success thresholds simply to make results look better. Never substitute scripted behavior, hardcoded actions, or artificial demonstrations for actual closed-loop controller performance.

3. **Failure is not proof of impossibility.** A failed experiment does NOT justify declaring an approach infeasible, fundamentally limited, or outside scope. Investigate the failure, identify its cause, form a hypothesis, implement a justified correction, and test again. Distinguish implementation bugs, tuning problems, numerical issues, and actual methodological limitations.

4. **Do not silently change the research plan.** Preserve the original objectives, methodology, and evaluation criteria. Implementation-level corrections and justified tuning are allowed. Any substantive deviation from the agreed approach MUST be explained and approved before proceeding. Never quietly simplify the problem to achieve success.

5. **Be persistent, but scientific.** Do not give up after one or several failed attempts. Explore reasonable solutions systematically, record meaningful findings, and use evidence to determine the next step. Do not repeat unsuccessful experiments without a clear reason.

6. **Report limitations honestly.** If an approach genuinely cannot achieve the intended behavior after meaningful investigation, report the evidence, attempted remedies, likely root causes, and remaining possibilities. Clearly distinguish confirmed limitations from unresolved failures. Never claim success without satisfying the original criteria.

7. **Make all results auditable.** Preserve the relevant configurations, commands, logs, metrics, and simulation artifacts needed to reproduce important results. Clearly distinguish successful runs from failed or incomplete experiments.

### Definition of Done

An implementation is complete ONLY when:

- The intended controller is actually implemented and running in closed loop.
- The authoritative system visibly demonstrates the intended behavior: **Task A**, the real arm with independently verified seating; **Task B**, the simulation (and, for any sim-to-real claim, the real arm).
- Quantitative results support the observed behavior.
- The result is reproducible without hidden interventions.
- The original objectives and evaluation criteria are satisfied.
- There are no concealed failures, artificial demonstrations, or unapproved shortcuts.

Task-specific completion:
- **Task A done:** the full pickup → align → guarded insert → verify → retract cycle runs on the real arm, with B0/A1/A2 trials, logs, video and failure analysis per the plan. If a hardware gate is not cleared, it is a partial deliverable, not a full insertion demonstration.
- **Task B done (simulation):** trained policy evaluated on held-out episodes against the scripted baseline, with the screen-recorded rollouts. **Ready for hardware** only at ≥90% verified simulated success on held-out episodes of the final curriculum stage (measured, exact clearance and perturbations; success on the initial/easy stages such as C0/C1 does not count) and after the offline-replay and shadow-mode gates. **Done on hardware** only after supervised physical trials with independent seating verification; otherwise label it `SIM_VALIDATED_ONLY`.

**If the authoritative system does not demonstrate the intended behavior (the real arm for Task A, the simulation for Task B), the controller is NOT DONE, regardless of how promising the numerical results appear.**

---

## Code Quality — KEEP IT SIMPLE

Write clean, minimal, readable, and auditable code.

1. **Simplicity first.** Prefer straightforward implementations over clever abstractions or unnecessarily complex architectures.
2. **Minimal comments.** Add only short, meaningful comments explaining non-obvious logic, important mathematical reasoning, or critical implementation decisions. Do not narrate obvious code.
3. **No unnecessary complexity.** Avoid excessive helper functions, classes, abstraction layers, defensive wrappers, or dependencies when simpler solutions are sufficient.
4. **Readable structure.** Use descriptive variable names, small functions where appropriate, and clear execution flow.
5. **No premature engineering.** Do not introduce frameworks, extensibility mechanisms, or elaborate design patterns for hypothetical future requirements.
6. **Scientific transparency.** Keep the implementation close to the underlying mathematical formulation so that the controller logic is easy to inspect and verify.
7. **No unnecessary code.** Remove abandoned experiments, dead code, redundant logic, and debugging clutter from the final implementation, while preserving relevant experimental evidence separately.
8. **Comments only where needed.** Keep comments short and understandable in caveman language, so the code is not messy and has mroe comments than teh code itself.

**The objective is code that I can open, understand, verify, and audit without having to reverse-engineer unnecessary complexity.**

### Guiding Principle

**Correctness over convenience. Evidence over claims. Simplicity over cleverness. Scientific integrity over impressive-looking results.**


## Graph Generation and Visualization Standards — Mandatory Instructions for Claude

Whenever generating graphs, plots, charts, or statistical visualizations, **prioritize clarity, readability, and interpretability over information density.**

Every graph must be clean, professionally formatted, properly labeled, and easy to understand without requiring extensive explanation.

### 1. Never Overcrowd Graphs

- Do not attempt to fit every metric, variable, or experimental result into a single graph.
- **Prefer multiple clean, focused graphs over one overcrowded visualization.**
- Each graph should communicate one primary finding or answer one specific experimental question.
- Separate unrelated metrics into individual plots.
- Avoid excessive curves, annotations, legends, or overlapping elements.
- Use subplots only when they genuinely improve comparison without sacrificing readability.

### 2. Mandatory Formatting Requirements

Every graph must have:

- **Descriptive title:** Clearly communicate what is being measured or compared.
- **Labeled axes:** Include variable names and appropriate measurement units.
- **Readable legends:** Clearly distinguish controllers, experimental conditions, or datasets.
- **Appropriate axis ranges:** Avoid misleading scales or unnecessary empty space.
- **Readable typography:** Titles, tick labels, legends, and annotations must remain legible at normal viewing size.
- **Consistent visual styling:** Use consistent colors, line styles, and terminology across related experiments.
- **Appropriate visual encodings:** Use distinguishable lines, markers, or colors without relying on color alone.

### 3. Zero Overlapping or Clipped Elements

Before saving any graph, verify that:

- Titles, axis labels, tick labels, and legends do not overlap.
- Text and annotations do not obscure important data.
- Legends do not cover significant portions of plotted curves.
- No labels or plot elements are clipped at the figure boundaries.
- Dense data remains visually distinguishable.
- The figure is readable when included in a technical report or presentation.

Use appropriate figure dimensions, margins, font sizes, legend positioning, and layout adjustments.

**If the graph is crowded, redesign it or split it into multiple graphs rather than shrinking everything to fit.**

### 4. Make Experimental Conclusions Immediately Understandable

A reader should be able to inspect a graph and quickly determine:

1. What is being measured?
2. Which experiment, controller, or configuration does it represent?
3. What are the important trends or differences?
4. What conclusion is supported by the data?

For example, rather than plotting IBVS position error, orientation error, controller commands, search trajectory, and insertion depth on the same axes, generate separate focused graphs for each relevant metric.

For RL, avoid combining reward curves, episode lengths, success rates, and training losses into a single cluttered plot.

### 5. Preserve Scientific Integrity

- Never manipulate axis scales or selectively omit data to exaggerate performance.
- Clearly identify units, experimental conditions, sample sizes, and aggregation methods.
- Distinguish raw measurements from smoothed or averaged curves.
- Show uncertainty or variability when relevant.
- Identify failed runs and outliers rather than silently excluding them.
- Do not imply causal relationships or statistical significance without supporting evidence.

### 6. Mandatory Visual Inspection

**Generating a graph successfully in code does not mean the graph is visually acceptable.**

After generating a plot:

1. Save it to the appropriate experiment's `plots/` directory.
2. Open or render the saved graph for visual inspection whenever the environment supports it.
3. Check for overlapping labels, clipped content, unreadable legends, excessive information density, and misleading formatting.
4. Revise and regenerate any graph that fails these checks.
5. Ensure the final graph is understandable independently, with a clear title, labels, and sufficient context.

Do not claim to have visually inspected a graph unless it was actually rendered and examined.

### 7. Final Quality Standard

**Every graph should tell one clear, technically defensible story.**

A graph is acceptable only if it is:

- **Clean:** No clutter, overlapping elements, or unnecessary decoration.
- **Readable:** Every relevant label, curve, and measurement is distinguishable.
- **Self-explanatory:** Its purpose and main finding can be understood without reading the implementation code.
- **Accurate:** It faithfully represents the underlying experimental data.
- **Presentation-ready:** It can be used directly in a technical report, research presentation, or experiment review.

**Non-negotiable rule: Never sacrifice readability to fit more information into one figure. If separating information into three or four graphs makes the results easier to understand, generate three or four graphs.**

## Status

Planning stage: **no code, build, lint or test setup yet.** `Code/Grasp/` and `Code/IBVS/` hold only placeholder READMEs (grasp synthesis code goes in `Code/Grasp/`). Do not invent commands before they exist; update this file as code lands.

Layout:
- `source/proposal.pdf`: RBE 595 Final Project Report, Part 1 (intro, literature review).
- `plan/`: the governing specs. `Task_A_IBVS_Search_PLAN.md` (IBVS + bounded search), `Task_B_Isaac_Sim_RL_PLAN.md` (Isaac Sim/Lab RL), `SHARED_PRIMITIVES_CHECKLIST.md` (measurements, frames, motion primitives, safety gates), `setup_parameters.template.yaml` (schema only, `status: UNCALIBRATED`). Read these before implementing anything.
- `research/insertion_papers_master_review.md`: curated literature map.
- `Media/`: auditable experiment evidence. `Media/RULES.md` defines the mandatory run/failure documentation and directory layout (read it before running or recording any experiment). `Media/README.md` (currently empty) and `Media/IBVS/`, `Media/RL/` (empty) are for the chronological experiment history and runs.

## Project

Vision-guided cable connector insertion with a Kinova Gen3 Lite (6-DoF, two-finger parallel gripper) and a wrist-mounted Intel RealSense D405 (eye-in-hand). Connectors sit in a known 3D-printed cradle; sockets are real sockets on a fixed board inspired by the NIST Assembly Task Board 1. Three connectors of increasing difficulty: 6.35 mm audio jack, IEC power connector, USB-A.

Two tracks, evaluated on matched conditions, with separate code, results and histories (never merged):
- **Task A, IBVS + bounded search** (ROS 2, MoveIt, Kortex `twist_controller`): pickup → image-based servo alignment → guarded axial insertion → independent seating check → retract; on failure, retract and try the next bounded candidate offset. Compared as B0 (taught pose), A1 (IBVS), A2 (IBVS + search). **Hardware-focused**: the real-arm result is the primary deliverable. The controller may also be implemented in simulation (undecided, not required yet); if it is, simulation supports the hardware work (controller development, safe pre-contact testing, dry runs) and should share the same controller code and logging schema. Do not build Task A simulation infrastructure unless the user asks.
- **Task B, Isaac Sim/Lab RL**: measured digital twin; learns only pre-contact fine alignment and insertion (no grasp learning). Compared as B0 (scripted), B1 (standalone PPO/SAC), B2 (demo-augmented/residual). The actor may use only observations available on the real robot.

Priorities: Task A is hardware-first, with an optional simulation implementation still to be decided; Task B is sim-first: the policy is developed and validated in simulation, and it is deployed on the hardware only after it reaches ≥90% verified success in simulation on held-out episodes of the final curriculum stage, never the initial/easy stages (the user's deployment threshold; this is stricter than the plan's ≥80% nominal / ≥70% perturbed targets, so use 90% as the go/no-go for hardware deployment, on held-out episodes with the pre-registered evaluation set). Deployment still has to clear the plans' B-G5/B-G6 safety gates. Sim results must be demonstrated visually per the Success Criteria and `Media/` rules. Real-arm runs must clear the plans' hardware gates (independent seating sensor, e-stop, supervisor, G0-G5) and are logged to the same evidence standard (videos, logs, configs, failures kept); a sim result never substitutes for a hardware result, and vice versa. If a hardware gate is not cleared, report the partial deliverable (e.g. `SIM_VALIDATED_ONLY`) rather than claiming physical insertion.

## Design decisions and constraints that shape the code

- Grasp the rigid housing, never the cable. The cradle gives a near-identical grasp every time (fixed grasp pose).
- Markers (cradle and board) give a 6-DoF pose from one camera; socket positions are known relative to the board markers. Board pose is the unknown that vision supplies; connector CAD/NIST dimensions are known in advance.
- Two-phase motion: MoveIt position control to a standoff a few cm above the socket, then slow velocity control along the insertion axis.
- **No wrist force/torque sensor**: jam/stall detection must come from commanded vs. actual motion and insertion depth. Depth alone cannot prove seating; success is checked independently (continuity/switch/LED). No force-guided search or imagined force feedback.
- **No unmeasured constants**: any dimension, transform, limit or depth not measured stays `TBD`/`null` and blocks its gate (see `plan/SHARED_PRIMITIVES_CHECKLIST.md`). Alignment success ≠ insertion success.
- IBVS controls image-space error only for the DoFs it actually observes (lateral x/y + insertion-axis yaw in v1); anything using estimated depth/pose is labelled hybrid.
- Single camera, and the plug can occlude the socket near insertion; markers must stay visible until the final approach. PBVS (pose error) and IBVS (image-feature error) are separate options, not interchangeable.
- Parallel gripper cannot reorient the plug in its fingers, so any grasp shift changes alignment.

## Evaluation

Two layers, kept distinct:
- **Task A/B plans** (authoritative): matched B0/A1/A2 physical trials (30 nominal + 30 perturbed each, A2 target ≥24/30 verified; hardware is Task A's priority; matched simulation runs only if a Task A simulation is built) and B0/B1/B2 simulation trials (≥100 held-out episodes per distribution, targets ≥80% nominal / ≥70% perturbed, ≥3 training seeds where compute permits). Targets are goals, not pass/fail claims; report exact `n`, confidence intervals and every failure.
- **Proposal Table I** (final-report metrics, 20 trials per connector): insertion success (audio ≥90%, IEC ≥80%, USB-A ≥70%, 95% Wilson interval), first-attempt success, grasp success ≥95%, socket localization error (mean <2 mm, <3°), cycle time <60 s, retries ≤1 per success, failure causes (perception / grasp / insertion). The plans' 30-trial counts differ from the proposal's 20; reconcile with the user before running physical trials.

Define success criteria before evaluating and never alter them afterwards.

## Environment

Ubuntu 22.04, ROS 2 Humble, MoveIt, `ros2_kortex` driver; arm at 192.168.1.10 (host 192.168.1.11 over the arm's RNDIS USB interface). `../start_arm.sh` configures the link and launches `kinova_gen3_lite_moveit_config robot.launch.py` on real hardware (keep the e-stop within reach). `../drivers.md` has driver install notes; `../Kinova-kortex2_Gen3_G3L*` are copies of the Kinova Kortex API docs/repo.

Hardware gotchas seen on this arm/camera (from earlier coursework): every terminal needs the same `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; the D405 publishes aligned depth only while raw depth is also subscribed; the arm's USB link dropping crashes the driver. `../HW7/` is an unrelated earlier grasping project (different grasp pipeline and controller); do not treat it as a code base to build on.

Isaac Sim/Lab needs an RTX-class GPU workstation (plan cites ~16 GB VRAM / 32 GB RAM); check compatibility before budgeting training.
