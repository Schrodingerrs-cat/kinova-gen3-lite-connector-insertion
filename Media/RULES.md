# Simulation Results & Experimental Evidence (for claude to read)

**Purpose:** This directory serves as the central, auditable record of all simulation experiments, controller evaluations, proof-of-concept demonstrations, and experimental findings for both **Image-Based Visual Servoing (IBVS)** and **Reinforcement Learning (RL)**.

This document also defines mandatory instructions for Claude when implementing, testing, validating, and documenting simulation results.

**Core principle: Every meaningful claim about controller performance must be supported by reproducible experiments, numerical measurements, and authentic visual evidence from the simulation.**

Successful experiments, partial successes, and failures are equally valuable. The objective is to establish an accurate, transparent, and technically defensible experimental history—not merely to demonstrate success.

---

## 1. Directory Organization

Maintain separate experimental histories for IBVS and RL. Never mix their results, configurations, checkpoints, or evidence.

### Required Directory Structure

```text
Media/
│
├── README.md
│
├── IBVS/
│   ├── README.md
│   ├── run_YYYYMMDD_HHMMSS_<description>/
│   │   ├── README.md
│   │   ├── config.json
│   │   ├── metrics.json
│   │   ├── trajectory.csv
│   │   ├── controller.log
│   │   ├── screenshots/
│   │   ├── videos/
│   │   └── plots/
│   │
│   ├── Failed_runs/
│   │   └── run_YYYYMMDD_HHMMSS_<description>/
│   │       ├── README.md
│   │       ├── failure_analysis.md
│   │       ├── config.json
│   │       ├── metrics.json
│   │       ├── controller.log
│   │       ├── screenshots/
│   │       ├── videos/
│   │       └── plots/
│   │
│   └── archive/
│       └── <older_successful_runs>/
│
└── RL/
    ├── README.md
    ├── run_YYYYMMDD_HHMMSS_<description>/
    │   ├── README.md
    │   ├── config.json
    │   ├── metrics.json
    │   ├── trajectory.csv
    │   ├── controller.log
    │   ├── screenshots/
    │   ├── videos/
    │   ├── plots/
    │   └── checkpoints/
    │
    ├── Failed_runs/
    │   └── <failed_runs>/
    │
    └── archive/
        └── <older_successful_runs>/
```

### Organization Rules

1. **Active successful or partially successful experiments:** Store in their respective `IBVS/` or `RL/` directories.
2. **Failed experiments:** Move to `Failed_runs/` under the appropriate controller category.
3. **Older successful experiments:** Move to `archive/` once superseded by newer validated results.
4. **Unresolved or partially successful experiments:** Retain in the active directory, with their incomplete status clearly documented.
5. **Never delete experimental evidence merely because an experiment failed or a newer implementation performs better.**
6. Every experiment must have a unique run identifier.
7. Update references when moving runs between active, failed, and archived directories.
8. Never combine outputs from different runs without explicitly identifying each source.

Only preserve artifacts relevant to simulation execution, performance analysis, reproducibility, debugging, and experimental validation.

---

## 2. Mandatory Documentation for Every Experiment

Every meaningful simulation milestone, controller test, experiment, or proof of concept (POC) must be documented.

This includes early tests demonstrating partial functionality, such as successful visual alignment without successful insertion.

Each experiment must contain a `README.md` documenting the following information.

### A. Experiment Identification

- Unique experiment/run ID.
- Date and time.
- Controller type: IBVS or RL.
- Experiment objective.
- Hypothesis being tested.
- Current status: `SUCCESS`, `PARTIAL_SUCCESS`, or `FAILED`.
- Associated code commit/version, if available.

### B. Implementation Details

Record everything reasonably necessary to reproduce the experiment.

For IBVS, include relevant information such as:

- Visual features and desired feature configuration.
- Camera calibration and camera-to-robot transformation.
- Controller gains and control frequency.
- Alignment tolerances and stopping criteria.
- Search strategy, if implemented (spiral, grid, random, etc.).
- Insertion strategy and motion constraints.
- Robot initial pose and target pose.

For RL, include relevant information such as:

- Algorithm (PPO, SAC, etc.).
- Policy architecture and observation space.
- Action space, action scaling, and control frequency.
- Reward function and individual reward components.
- Curriculum stage, if applicable.
- Training hyperparameters.
- Number of training steps and environment instances.
- Randomization settings.
- Policy checkpoint used for evaluation.
- Training versus evaluation environment configuration.

For both approaches, include:

- Simulation engine and version.
- Robot model and physical parameters.
- Collision geometries and contact settings.
- Connector and socket geometry.
- Initial conditions and object offsets.
- Relevant random seeds.
- Software dependencies and configuration files.

If a value is unavailable, mark it as `UNKNOWN` rather than inventing it.

### C. Results

Provide quantitative measurements wherever applicable.

Examples include:

- Position and orientation errors.
- Visual feature alignment error.
- Insertion success rate.
- Completion time.
- Number of insertion attempts.
- Controller convergence time.
- Contact and collision statistics.
- Peak and sustained contact forces, if available.
- Controller stability and oscillations.
- Joint-limit or workspace violations.
- Episode reward and reward components for RL.
- Failure frequency and failure categories.

Do not report metrics that were not actually measured.

### D. Evidence

Provide exact relative paths to:

- Simulation screenshots.
- Screen recordings or videos.
- Numerical results (JSON/CSV).
- Trajectory recordings.
- Performance plots.
- Relevant logs.
- Saved controller configurations or policy checkpoints.

Every artifact must correspond to the experiment being documented.

### E. Observations and Conclusions

Document:

- What behavior was visibly observed.
- Whether the controller achieved the intended task.
- Whether the numerical metrics agree with the visual evidence.
- What the experiment actually proves.
- What remains uncertain or unresolved.
- What should be tested next.

**Never infer complete task success from partial controller functionality.**

For example, successful IBVS alignment does not establish successful insertion.

---

## 3. Visual Evidence Is Mandatory for Claims of Success

**Numerical metrics alone are not sufficient to declare a controller successful.**

A controller must visibly execute the intended task inside the actual simulation environment.

A valid success claim requires all of the following:

1. **Actual controller execution:** A reproducible simulation run using the implemented controller or trained policy.
2. **Visual proof:** Screenshots or videos showing the relevant robot motion and task outcome.
3. **Numerical validation:** Recorded measurements supporting the observed behavior.
4. **Traceability:** Evidence that the visual recording and numerical results originated from the same run.
5. **Task completion:** Verification that the predefined task success conditions were actually satisfied.

For insertion tasks, evidence should demonstrate the relevant phases:

- Initial robot and target configuration.
- Approach and alignment.
- Search or correction behavior, if applicable.
- Insertion execution.
- Final insertion state.
- Any contact violations, collisions, or failures.

A favorable error metric without actual task completion is **not proof of successful insertion**.

### Prohibited Practices

Never:

- Substitute manually scripted motions for controller behavior while presenting them as controller execution.
- Use hardcoded successful trajectories as proof that a controller works.
- Alter evaluation conditions after observing results without documenting the change.
- Relax success thresholds silently.
- Present hand-selected screenshots as representative statistical performance.
- Present a successful training rollout as independent policy evaluation.
- Fabricate, modify, or misrepresent numerical results.
- Claim an insertion succeeded simply because the robot reached a desired pose.
- Hide unsuccessful episodes or regressions.

Scripted motions may be used legitimately for calibration, baseline comparisons, or debugging, but **must be explicitly labeled as scripted**.

---

## 4. Failure Documentation Is Equally Important

**Every meaningful failure must be preserved, analyzed, and traceable.**

Failed experiments are not disposable outputs. They provide evidence of controller limitations, modeling errors, unsuccessful hypotheses, and engineering decisions.

Store failed experiments under:

```text
Media/IBVS/Failed_runs/
Media/RL/Failed_runs/
```

Each failed experiment must contain a `failure_analysis.md`.

### Required Failure Analysis

#### 1. Expected Behavior

Describe what the controller was supposed to accomplish.

#### 2. Observed Failure

Explain what actually happened.

Examples:

- Controller failed to converge.
- Robot oscillated around the target.
- Visual alignment succeeded, but insertion failed.
- Connector collided with the socket.
- Robot exceeded a defined motion constraint.
- RL policy failed to generalize to a changed offset.
- Policy repeatedly selected ineffective actions.
- Simulation became unstable or terminated unexpectedly.

#### 3. Supporting Evidence

Preserve relevant:

- Videos or screenshots.
- Trajectory data.
- Error plots.
- Controller logs.
- Simulation state information.
- Metrics captured before the failure.

#### 4. Root-Cause Analysis

Investigate the cause using the available observations and recorded data.

Clearly distinguish between:

- **Verified cause:** Supported by sufficient diagnostic evidence.
- **Suspected cause:** Plausible explanation requiring additional testing.
- **Unknown cause:** Insufficient evidence to make a reliable determination.

If the root cause cannot be established, provide an evidence-based hypothesis and explain what additional experiment could verify it.

Never present an educated guess as a confirmed explanation.

#### 5. Corrective Actions

Document:

- Proposed corrections.
- Parameters or implementation changes attempted.
- Why those changes were chosen.
- Subsequent experiments used to test the changes.

#### 6. Resolution Status

Use one of:

- `UNRESOLVED`
- `UNDER_INVESTIGATION`
- `FIX_ATTEMPTED`
- `RESOLVED_AND_VALIDATED`

A failure can only be marked resolved after a subsequent simulation experiment demonstrates that the correction addresses the original issue.

Link the validating run explicitly.

Do not overwrite the failed run with the corrected result.

---

## 5. Milestone Tracking and Experimental History

Maintain a chronological experimental history in:

```text
Media/README.md
Media/IBVS/README.md
Media/RL/README.md
```

The main README should summarize major developments across both methods.

Controller-specific READMEs should contain their complete experimental histories.

### Milestone Record

For every milestone, document:

| Field | Description |
|---|---|
| Date | Experiment date |
| Run ID | Unique experiment identifier |
| Controller | IBVS or RL |
| Objective | What was tested |
| Change | What changed from the previous experiment |
| Outcome | Success, partial success, or failure |
| Key metrics | Relevant numerical results |
| Evidence | Relative path to run directory |
| Next action | Follow-up experiment or unresolved issue |

Include successful experiments, failures, improvements, regressions, and controller changes.

**Never silently replace or rewrite historical results.**

If a previously successful controller fails under new conditions, document that regression explicitly.

When archiving a successful experiment, retain its original evidence and update the history to indicate its archived location.

---

## 6. Videos, Images, Plots, and Statistical Evidence

Visual and statistical evidence should support a technically accurate narrative of the controller's behavior.

Useful artifacts include:

### Videos

- Complete simulation runs.
- Successful insertion demonstrations.
- Failed insertion attempts.
- Controller convergence behavior.
- Search-strategy execution.
- Side-by-side controller comparisons.
- Policy behavior under different initial conditions.
- Representative RL evaluation episodes.

Whenever practical, retain full unedited recordings. Short clips may be created for presentations, but must link back to the corresponding full recording.

### Screenshots

- Initial and final simulation states.
- Intermediate alignment stages.
- Connector/socket contact states.
- Controller failure states.
- Simulation configurations.
- Relevant debugging visualizations.

### Graphs and Plots

- Position and orientation errors over time.
- Visual feature error over time.
- End-effector trajectories.
- Search paths and insertion trajectories.
- Joint positions and velocities.
- Contact forces and penetration depth, where available.
- RL training reward curves.
- RL evaluation success rates.
- Episode length distributions.
- Success rate versus initial offset.
- Performance comparisons between controller variants.

### Statistical Analysis

For experiments involving repeated trials, report:

- Number of trials.
- Number of successes and failures.
- Success rate.
- Mean and standard deviation where appropriate.
- Median and range or percentiles where useful.
- Variation across initial configurations.
- Variation across random seeds, when relevant.
- Failure-category distributions.

Clearly distinguish between individual demonstrations and aggregated results.

For example:

- `1/1 successful run` demonstrates one successful execution.
- `85/100 successful trials` supports an estimated 85% success rate under the tested conditions.

These are not equivalent claims.

Use repeated trials where necessary to establish reliability, robustness, and generalization.

---

## 7. Reproducibility and Experiment Integrity

Every meaningful experiment should be reproducible with the available code, configuration, initial conditions, and environment information.

### Required Practices

1. Save the exact configuration used during execution.
2. Record the controller implementation version or commit hash when available.
3. Preserve random seeds for stochastic experiments.
4. Record the actual initial conditions rather than only their intended values.
5. Preserve the evaluated RL checkpoint and its training configuration.
6. Document changes to robot geometry, socket geometry, physical parameters, or contact models.
7. Record simulator crashes or incomplete runs separately from legitimate task failures.
8. Ensure plots, metrics, and videos reference the same experiment ID.
9. Avoid overwriting existing run directories or files.
10. Preserve raw output whenever possible; clearly label any processed data.

If a run cannot be fully reproduced, explicitly document which information is missing.

### Simulation Fidelity

Since the ultimate objective is real-world insertion, simulation assumptions must be documented carefully.

Relevant parameters include:

- Robot kinematics and joint limits.
- End-effector and connector dimensions.
- Socket dimensions and tolerances.
- Camera pose, calibration, and field of view.
- Collision geometries.
- Contact friction and compliance.
- Actuator limits.
- Controller latency and update rates.
- Sensor noise and uncertainty.
- Object pose errors and initial offsets.

Distinguish measured real-world parameters from estimated or assumed parameters.

**Never describe a simulation as physically validated or representative of real-world performance unless the relevant properties have actually been verified.**

---

## 8. Success Criteria and Evaluation Integrity

Define success conditions before evaluating each experiment.

For insertion experiments, the evaluation may include:

- Alignment error within a predefined tolerance.
- Correct connector orientation.
- Insertion depth reaching a specified threshold.
- No prohibited collisions or joint-limit violations.
- Completion within the permitted number of attempts or time limit.
- Stability of the final inserted state.

The applicable criteria must be documented in the experiment configuration.

If only alignment succeeds, report `ALIGNMENT_SUCCESS`, not insertion success.

If the connector partially enters the socket but fails to reach the required depth, report `PARTIAL_SUCCESS` or `FAILED` according to the predefined criteria.

If a controller works only under a narrowly defined initial condition, document that limitation.

Do not extrapolate success to untested conditions.

### IBVS and RL Comparison

When comparing the two approaches, use equivalent evaluation conditions wherever practical:

- Same robot and insertion geometry.
- Same initial offset distributions.
- Same task completion criteria.
- Same workspace and safety constraints.
- Comparable trial counts.
- Clearly documented differences in observation or action capabilities.

Keep both raw result sets separate. Any combined comparison must reference the original run IDs.

---

## 9. Required Workflow for Claude

Claude must follow this workflow whenever conducting a meaningful simulation experiment.

### Before Running

1. Identify the controller and experimental objective.
2. Define measurable success and failure criteria.
3. Record the current implementation and configuration.
4. Create a unique run ID.
5. Prepare a dedicated experiment directory.

### During Execution

1. Run the actual implemented controller or policy.
2. Capture relevant visual evidence whenever supported.
3. Record numerical metrics and trajectories.
4. Preserve logs and important simulation events.
5. Avoid modifying success criteria during evaluation.

### After Execution

1. Inspect the actual simulation behavior.
2. Verify that numerical results agree with visual observations.
3. Classify the outcome honestly.
4. Save all available supporting artifacts.
5. Document conclusions and limitations.
6. If failed, create `failure_analysis.md` and place the run in `Failed_runs/`.
7. If successful, retain the run in the active controller directory.
8. Update the chronological experiment history.
9. Link any related earlier or subsequent experiments.

When a prior successful run is superseded, move it to `archive/` without discarding its evidence.

Do not create placeholder media, fabricated logs, or synthetic results simply to satisfy directory requirements. If an artifact could not be captured, record that limitation explicitly.

---

## 10. Evidence-Driven Reporting

The contents of this directory should make it possible to reconstruct the technical development of both approaches, including:

- What was attempted.
- Why it was attempted.
- Which assumptions were made.
- Which hypotheses were supported or rejected.
- What worked and under which conditions.
- What failed and why.
- Which corrections were attempted.
- Whether those corrections actually worked.
- How performance evolved across experiments.
- What can and cannot be concluded from the collected evidence.

Evidence can include videos, images, logs, graphs, controller trajectories, statistical analyses, configurations, numerical measurements, and documented experimental observations.

All evidence must be **authentic, correctly attributed, reproducible where feasible, and interpreted fairly**.

Do not select only favorable evidence to construct an artificially positive narrative.

The objective is not to make either IBVS or RL appear better. It is to determine experimentally what each approach can achieve, under which conditions, and with what limitations.

---

## 11. Non-Negotiable Rules

1. **No undocumented meaningful experiments.**
2. **No unsupported success claims.**
3. **No deletion or concealment of meaningful failures.**
4. **No mixing of evidence from different runs.**
5. **No replacing actual controller execution with scripted demonstrations.**
6. **No fabricated metrics, screenshots, logs, or videos.**
7. **No claiming that a fix works without validating it.**
8. **No claiming full insertion success from alignment alone.**
9. **No merging IBVS and RL experimental histories.**
10. **No modifying past results to match a desired narrative.**
11. **No claiming robustness or generalization without appropriate testing.**
12. **No ignoring regressions or unfavorable results.**

### Final Requirement

**The simulation itself must demonstrate that the controller works. The numerical results must support that demonstration. The recorded evidence must make the conclusion independently verifiable.**

A failed experiment with complete evidence and a well-supported diagnosis is more useful than an apparent success with missing or misleading evidence.

Maintain this directory as a reliable engineering and research record that can support debugging, future development, technical reports, presentations, and eventual real-world validation.
