# Literature review and candidate research contributions

**Status:** Living research document  
**Last reviewed:** 2026-08-20  
**Purpose:** Preserve the literature comparison, novelty boundary, and proposed experiments for later paper drafting.

## 1. Scope and review method

This review positions StarfishBot relative to work on:

- Online-reconfigurable and morphology-adaptive legged robots
- Robots that add, remove, disable, or repurpose legs or modules
- Morphology-aware and morphology-agnostic locomotion control
- Hexapod free-gait and fault-tolerant planning as supporting background
- Support-polygon and force-based stability measures
- Central pattern generator (CPG) locomotion control
- Symmetry and asymmetry as properties of reconfigurable morphologies

The review was produced through a targeted search of journal articles, conference papers, institutional repositories, and recent preprints. It is not yet a systematic or PRISMA-style review. Before submitting a paper, repeat the search in Scopus, Web of Science, IEEE Xplore, and Google Scholar; record the exact search strings and inclusion criteria; and check forward and backward citations from the closest papers.

## 2. Current project scope

StarfishBot models a six-legged robot whose legs can be attached at continuously
variable radial angles rather than only at a fixed set of chassis locations. The
research platform is intended to cover both continuous geometric reconfiguration
(changing an attachment angle) and discrete topology change (detaching or
reattaching a leg). A morphology state can be represented by:

- an active-leg mask indicating which legs are attached; and
- an angle vector indicating the radial attachment angle of each active leg.

This representation also makes it possible to study uncertainty in the robot's
knowledge of its own morphology. The current controller:

1. Samples or schedules leg mounting angles.
2. Predicts a geometric support-margin proxy for each candidate leg lift.
3. Selects the candidate with the largest immediate margin.
4. Sends either binary or experimental CPG-scaled joint offsets.
5. Records the commanded lifted state, geometric margin, and body orientation in CoppeliaSim.

The current margin assumes idealized contacts on a circle and a center-of-mass projection fixed at the origin. It is therefore a geometric proxy rather than a validated physical or dynamic stability measurement.

## 3. Literature map

| Area | Established result | Consequence for StarfishBot |
| --- | --- | --- |
| Static support margin | Static stability based on the projected center of mass and support polygon is a classical result. Force-, moment-, and dynamic alternatives have also been developed. Agheli and Nestinger proposed and physically evaluated a foot-force stability margin for online use. | A support-polygon distance alone is not novel. StarfishBot could contribute a validation study showing when its inexpensive angle-based proxy agrees with contact-, CoM-, or force-based measurements. |
| Arbitrary and free gait | Porta and Celaya designed a reactive controller that handles arbitrary leg configurations. Valouch and Faigl presented gait-free planning with arbitrary swing sequences and graph-searched stances. | “Works with arbitrary leg configurations” or “chooses legs without a fixed tripod sequence” is insufficient as a novelty claim. The distinction must be online change of the robot's mounting morphology and safe transitions between morphologies. |
| Fault-tolerant gait | Hexapod leg-failure recovery using stability-aware gait sequences, altered footholds, and learned compensatory behaviors is established. | A failure study must compare morphology reconfiguration against strong gait-only and learning-based recovery baselines. |
| CPG control | CPG locomotion, sensory feedback, reinforcement-learning adaptation, and smooth gait transitions have extensive prior work. | A basic two-neuron CPG is not a research contribution. A possible contribution is morphology-conditioned coupling, phase adaptation, or a controlled comparison of CPG and non-CPG scheduling across unseen morphologies. |
| Morphological adaptation | Nygaard et al. demonstrated a quadruped that changes leg length in response to terrain and energy efficiency. Recent morphology-generalized policies also target control across different embodiments. | StarfishBot should focus specifically on angular or radial redistribution of leg mounts and the stability of the morphology transition, not claim morphological adaptation in general. |
| Online module removal | CONRO continued walking when legs were dynamically removed and selected new group behaviours when its topology changed. Yu et al. later demonstrated a pretrained amputation-agnostic policy that retained locomotion after severe module removal without receiving explicit connectivity information. | Continuing after leg loss and doing so without explicit morphology input are established. StarfishBot should compare adaptation mechanisms and quantify recovery rather than claim either capability alone as new. |
| Morphology recognition and controller selection | Snapbot detects connected legs at fixed ports and selects a hard-coded locomotion mode. Snapbot V2 uses camera-based self-recognition. WalkingBot reconstructs its topology and generates an MPC policy, but changes after startup require a rescan and policy generation/upload. | Explicit configuration knowledge is established and is not always necessary. The open question is when exact continuous attachment geometry improves recovery, robustness, or efficiency over active-leg-only and morphology-agnostic control. |
| Distributed arbitrary-configuration gait | KARAKASA modules use local communication to generate static walking for varied 2-D clusters without a global description of the complete leg configuration; later work recruits modules to adapt the morphology to slopes and gaps. | Arbitrary layouts and distributed gait generation are not new by themselves. StarfishBot's distinction should be the controlled study of continuous circumferential attachment angles, symmetry, and controller behaviour during well-defined reconfiguration events. |
| Morphology-conditioned generalized policies | Graph-structured modular policies can control many unseen assembled designs without additional learning by conditioning the policy structure on the robot design graph. | A geometry-aware learned policy is a relevant controller class, but generalization must be tested on held-out angle regions and sudden online changes, not only preassembled designs. |
| Asymmetric structure | Fixed asymmetric hexapod structures and model-based structural optimization have already been studied. | The contribution should be continuously variable asymmetry, online adaptation, or a morphology-performance model rather than one optimized fixed chassis. |
| Asymmetric locomotion | A recent preprint uses asymmetric contact planning and hardware symmetry breaking to improve hexapod speed. | “Asymmetry can improve locomotion” is not a safe novelty claim. StarfishBot should focus on reconfiguration, transition safety, fault recovery, and generalization. |
| Reproducible platforms | Recent open hexapod platforms provide CPG control, inverse kinematics, contact sensing, terrain trials, and sim-to-real evaluation. | Open source is valuable but should be presented as a supporting contribution. A StarfishBot benchmark should uniquely cover continuously varying radial morphology. |

### 3.1 Closest online-reconfiguration comparisons

The primary comparison set should be robots whose morphology changes during
operation or whose controller is explicitly designed for changing modular
topology. Fixed asymmetric robots remain useful background for RQ1, but they are
not the main comparison group for the controller study.

| System | Reconfiguration and geometry | Control response | Evidence about reset or interruption |
| --- | --- | --- | --- |
| CONRO (Salemi and Shen, 2004) | Chain-type modules with discrete labelled connectors; legs and spine modules were removed and reconnected dynamically. | Distributed topology discovery maps a global task and local topological role to stored local behaviours. | Walking continued after one or more leg removals when the remaining structure could support the body. Reconnecting two separated robots caused the four-legged gait to be *re-initiated*, indicating a gait restart rather than online learning or a full robot reboot. |
| Snapbot (Kim et al., 2017) and Snapbot V2 (Gim and Kim, 2020) | Removable legs occupy fixed discrete ports around a central body. | The robot recognizes the connected-leg configuration and selects an associated preprogrammed locomotion style. | No retraining or robot reboot is reported after recognition. The publications do not establish whether attachment or detachment occurs during uninterrupted walking, whether the robot first assumes a neutral stance, or whether gait phase is preserved. |
| WalkingBot (Wang et al., 2020) | Users assemble discrete actuator, passive, sensor, and foot blocks into varied legged structures. | The structure is reconstructed and an MPC policy is generated from predefined footfall patterns. | The topology is scanned at startup. A later physical change requires a user-triggered rescan, policy generation over several seconds, verification, and upload; this is not a seamless hot-plug transition. |
| KARAKASA (Hayakawa et al., 2020; Hayakawa and Matsuno, 2022) | Single-legged disk modules form non-biological 2-D clusters. Hardware connectors are discrete; the 2022 dynamic simulations additionally allow connection anywhere around the body. | Each module determines its foot pattern from local communication, without a global description of the complete configuration. A later system recruits an additional module when traversal risk is high. | The work supports configuration-independent distributed static walking and adaptive recruitment. It does not provide a standard measurement of stop time, gait-phase reset, or uninterrupted mid-stride attachment comparable with the proposed StarfishBot benchmark. |
| Modular graph policy (Whitman et al., 2023) | Legs, wheels, and bodies are assembled into a design represented as a graph. | One graph-structured policy is conditioned on that design and generalizes to many unseen designs without additional learning. | It avoids morphology-specific retraining for tested assembled designs, but the main experiments are not sudden mid-gait leg-loss transitions. |
| Modular legs (Yu et al., 2026) | Autonomous modules may be freely assembled into many discrete body structures and physically removed to emulate amputation. | A single amputation-agnostic policy is distilled offline from damage-specific expert policies. It receives sensor history but no connectivity input and continues to address disconnected modules. | No policy switch or online retraining is required at damage time. This is the strongest morphology-agnostic baseline, although it depends on substantial offline training over multiple damage states. |

Because the literature uses *reset* inconsistently, StarfishBot should not report
it as a single yes/no outcome. Each trial should record separately:

1. whether locomotion is interrupted;
2. whether a neutral or safe stance is entered;
3. whether gait phase or controller state is reinitialized;
4. whether a stored policy is switched or a new trajectory is generated;
5. whether online optimization or learning is required; and
6. time until stable target-speed locomotion is recovered.

## 4. Novelty boundary

### Claims that should not be made

The following are established or currently unsupported:

- “We introduce the support-polygon stability margin.”
- “We introduce CPG control for a hexapod.”
- “We introduce fault-tolerant hexapod gait planning.”
- “We are the first to generate a gait for arbitrary leg configurations.”
- “Asymmetric morphology improves locomotion.”
- “We are the first robot to continue walking after losing a leg.”
- “A reconfigurable robot must know its attachment geometry to recover.”
- “The current proxy measures physical dynamic stability.”
- “The current controller demonstrates forward locomotion,” unless complete foot trajectories, displacement, and speed are measured.
- “The method generalizes,” unless it is evaluated on held-out morphologies rather than only new random seeds.

### Potentially defensible central claim

Subject to a more exhaustive literature search and successful experiments, a defensible direction is:

> We provide a controlled statistical study of how symmetry class and degree of
> asymmetry affect locomotion across continuously parameterized circumferential
> leg configurations. We then benchmark morphology-specific, geometry-aware,
> and morphology-agnostic controllers under online leg detachment, reattachment,
> and angular repositioning, including ablations that quantify the value of
> active-leg and attachment-angle information.

This claim does not require every morphology change to occur during locomotion.
Changing configurations between trials is sufficient for the morphology atlas and
cross-controller benchmark. The stronger claim of *online reconfiguration* should
be used only if the physical or simulated robot changes an attachment angle, loses
a leg, or accepts a reattached leg during an uninterrupted trial.

The platform alone is not the complete contribution. The paper needs a controlled
sampling protocol, controller comparisons, repeated trials, and statistically
supported findings about the interaction between morphology, morphology
information, and controller performance.

## 5. Candidate contributions

### 5.1 Morphology-stability atlas

Systematically map radial mounting configurations to observed robot performance. Candidate morphology descriptors include:

- Largest circular gap between adjacent legs
- Minimum circular gap
- Variance or coefficient of variation of all six circular gaps
- Circular entropy
- Distance from regular 60-degree spacing
- Low-order Fourier descriptors of the radial contact distribution
- Area, minimum width, and isotropy of the idealized support polygon

Candidate response variables include:

- Minimum geometric support margin
- Minimum actual CoM-to-contact-polygon margin
- Roll and pitch RMS, maximum, and recovery time
- Contact loss and tipping probability
- Feasible gait-sequence count
- Speed, distance, and cost of transport after forward locomotion is implemented

The research result should identify which morphology descriptors predict performance and whether those relationships hold for held-out configurations.

### 5.2 Controller adaptation benchmark for online reconfiguration

The controller contribution should be a comparison of adaptation mechanisms, not
only a comparison of named gait patterns. Candidate strategy classes are:

1. **Unaware continuation:** continue the nominal controller after the morphology
   changes.
2. **Configuration lookup:** detect the active-leg configuration and select a
   stored morphology-specific gait, following the general Snapbot approach.
3. **Geometry-aware replanning:** update the active-leg mask and continuous
   attachment angles, then regenerate the gait, support sequence, or short-horizon
   plan.
4. **Morphology-conditioned generalized policy:** use the current morphology as
   policy context and act without morphology-specific retraining.
5. **Morphology-agnostic adaptation:** infer the consequences of the change from
   proprioceptive and contact history without receiving the attachment geometry.
6. **Online optimization or learning:** adapt after the change through additional
   trials or updates, reporting the time and experience required.

Not every class must be implemented in the first paper. The minimum useful study
is unaware continuation, configuration lookup or phase-reset control,
geometry-aware replanning, and one controller that does not receive exact angles.
All methods must receive comparable sensing and tuning or training budgets.

The key experiment is locomotion through a controlled morphology-change event.
Measure the transient from the change until recovery, including whether the robot
stops, adopts a safe stance, resets gait phase, switches policy, replans, or
relearns.

### 5.3 Configuration-aware detachment and reattachment

The fault study should distinguish the value of knowing that a leg is absent from
the additional value of knowing where every remaining or reattached leg is
located. Compare the following morphology-information conditions while holding
the controller family and tuning budget fixed:

1. **Morphology-blind:** assume the nominal six-leg configuration.
2. **Detachment-aware:** know which leg is missing, but assume nominal attachment
   geometry for the surviving legs.
3. **Exact angle-aware:** know the active-leg mask and ground-truth attachment
   angles.
4. **Noisy angle-aware:** receive attachment angles with controlled bias or noise.
5. **Self-estimating:** infer the active-leg mask and attachment angles from robot
   sensors or a calibration procedure, if such a method is implemented.
6. **Oracle:** use simulator ground truth as an upper-bound reference.

Evaluate intact asymmetric configurations, detachment before locomotion,
detachment during locomotion, angular repositioning, same-angle reattachment,
and arbitrary-angle reattachment. Live reattachment should be included only if
it is mechanically and operationally feasible; otherwise it can be evaluated
between trials.

This ablation directly tests whether morphology sensing or self-identification is
necessary for robust control. A possible result, if supported, is that knowledge
of both attachment topology and continuous radial geometry improves recovery and
generalization beyond knowing only which leg failed. Relevant outcomes are
minimum physical stability, task completion, forward progress, recovery time,
energy, attitude disturbance, and failure rate.

Leg removal also changes mass distribution and the physical centre of mass. The
fixed-origin geometric proxy is therefore not adequate ground truth for this
study; the experiments must use the actual projected centre of mass, measured
contacts, and preferably contact forces.

### 5.4 Validated low-cost stability surrogate

Compare the current angle-based proxy with:

- Actual CoppeliaSim foot-contact positions
- Projected physical center of mass
- Contact-force or foot-force stability margin
- Body attitude and angular velocity
- Observed tipping, slipping, and contact loss

A useful result would define the operating conditions and error bounds under which the inexpensive proxy preserves the same gait decision as a higher-fidelity metric. A corrected or learned calibration model may be needed.

### 5.5 Open asymmetric-morphology benchmark

Release a benchmark containing:

- A defined continuous morphology space and constraints
- Fixed training, validation, and test morphology splits
- Symmetric, mildly asymmetric, severely asymmetric, transition, and failure scenarios
- Detachment, angular repositioning, same-angle reattachment, and arbitrary-angle
  reattachment scenarios
- Morphology-information ablations with exact, missing, and noisy angle inputs
- Versioned CoppeliaSim scenes
- Controller baselines
- Raw contact, CoM, force, orientation, energy, and trajectory data
- Configurations, seeds, checksums, and analysis scripts

This should be treated as a supporting contribution unless the benchmark fills a clearly documented gap and is adopted beyond this project.

## 6. Proposed research questions and hypotheses

### Primary research questions

> **RQ1 — Symmetry and performance:** Across continuously variable
> circumferential leg-attachment configurations, how do symmetry class and
> degree of asymmetry affect locomotion performance, stability, efficiency, and
> robustness?

> **RQ2 — Control after online reconfiguration:** Which control strategies are
> most suitable for a free-radial reconfigurable legged robot when legs are
> detached, reattached, or repositioned during operation?

> **RQ3 — Value of morphology information:** How much does a controller benefit
> from knowing which legs are active and their circumferential attachment angles
> when adapting to an online morphology change?

### Supporting research questions

- **RQ4:** Which morphology descriptors best predict performance on held-out
  configurations when different angle arrangements can have the same scalar
  asymmetry score?
- **RQ5:** Which controller families remain robust across symmetric, asymmetric,
  detached-leg, angular-repositioning, and arbitrary-angle reattachment
  conditions?
- **RQ6:** What is the performance difference between morphology-blind,
  detachment-aware, exact angle-aware, noisy angle-aware, and self-estimating
  controllers?
- **RQ7:** How sensitive is controller performance to attachment-angle measurement
  error, and what sensing accuracy is required to approach oracle performance?
- **RQ8:** Which adaptation strategies require locomotion interruption, a safe
  stance, gait-phase reset, policy switching, replanning, or online learning, and
  how long does each take to recover?
- **RQ9:** Can the robot recover from leg detachment or angular repositioning
  without offline retraining or morphology-specific manual retuning?
- **RQ10:** How accurately does the geometric proxy predict contact-based
  stability, attitude disturbance, and failure?

### Candidate hypotheses

- **H1:** The largest circular gap predicts minimum physical support margin better
  than a general scalar asymmetry index.
- **H2:** Controller performance degrades as radial asymmetry increases, but the
  rate of degradation differs by controller family.
- **H3:** Geometry-aware control improves task success, recovery time, and body
  stability after detachment, angular repositioning, or arbitrary-angle
  reattachment relative to morphology-blind and active-leg-only control.
- **H4:** A morphology-agnostic adaptive controller can recover without explicit
  angle input, but geometry-aware control reaches stable target-speed locomotion
  faster or with less offline training across unseen attachment angles.
- **H5:** The advantage of angle-aware control increases with morphology asymmetry
  and with the displacement of a reattached leg from its nominal angle.
- **H6:** Controlled attachment-angle noise produces a measurable performance
  degradation, enabling estimation of the sensing accuracy required for robust
  control.
- **H7:** A self-estimated morphology, if implemented, approaches oracle
  performance without direct access to simulator ground truth.
- **H8:** A calibrated geometric proxy preserves most controller decisions at
  lower computational cost than force-based evaluation.
- **H9:** A safe transition controller that explicitly manages contact and gait
  phase produces smaller attitude disturbances and fewer falls than immediate
  policy switching or unaware continuation.

## 7. Recommended experimental design

### 7.1 Independent variables

- Leg mounting morphology
- Morphology asymmetry level
- Static versus online-changing morphology
- Controller or gait strategy
- Motor mode
- Terrain type and slope
- Leg-failure condition
- Reattachment condition: absent, original angle, or new arbitrary angle
- Morphology information: none, active-leg mask only, exact angles, noisy angles,
  self-estimated angles, or oracle state
- Attachment-angle error magnitude
- Random seed and initial state

### 7.2 Controller baselines

At minimum, compare:

1. Nominal fixed gait with unaware continuation after a morphology change
2. Configuration lookup or gait switching using the active-leg mask
3. Current one-step greedy margin planner
4. Geometry-aware planner using the active-leg mask and exact attachment angles
5. The same planner without exact angles, or a morphology-agnostic adaptive
   controller
6. A look-ahead transition-aware planner, if implemented

A fault study should also include a published-style fault-tolerant gait-only baseline.
For each suitable controller family, include morphology-blind and morphology-aware
variants where possible. This avoids concluding that morphology information is
valuable merely because a baseline implementation is hard-coded for six nominally
positioned legs. Give all methods the same training or tuning budget.

Fixed tripod, wave, and random eligible-leg selection remain useful lower-level
gait baselines, but they do not by themselves represent distinct reconfiguration
adaptation mechanisms.

### 7.3 Measurements

Primary outcomes should include:

- Minimum actual support margin
- Contact- or force-based stability margin
- Roll/pitch RMS and maximum
- Angular velocity
- Contact loss, slip, and tipping rate
- Forward speed and distance
- Energy or cost of transport
- Morphology transition duration
- Locomotion interruption duration
- Whether a neutral or safe stance was entered
- Whether gait phase or controller state was reset
- Whether a stored policy was switched or a new trajectory was generated
- Online optimization or learning time and number of trials
- Recovery time after failure
- Recovery time after reattachment or morphology-state update
- Performance degradation relative to the intact nominal condition
- Morphology-estimation error and update latency, if self-estimation is implemented
- Controller computation time

The geometric proxy should be reported as a predictor or controller input, not as ground truth.

### 7.4 Sampling and replication

A reasonable staged design is:

1. Screen a large morphology set in simulation.
2. Divide morphologies into asymmetry quantiles or descriptor strata.
3. Reserve complete morphology regions as held-out tests.
4. Select representative morphologies for higher-fidelity or physical trials.
5. Repeat every condition across multiple seeds and initial states.

Avoid treating thousands of time steps from one run as independent experimental replicates. The independent unit is normally a run, morphology, or robot trial, depending on the hypothesis.

### 7.5 Statistical analysis

Possible analyses include:

- Correlation and nonlinear regression between morphology descriptors and outcomes
- Cross-validated prediction on held-out morphologies
- Mixed-effects models with controller, asymmetry, morphology information, and
  their interactions as fixed effects, and morphology and run as random effects
- Paired controller comparisons on identical morphologies and seeds
- Confidence intervals and effect sizes, not only p-values
- Survival or failure-rate analysis for tipping and incomplete trials
- Ablations for the proxy, look-ahead horizon, transition cost, and CPG mode

Pre-register primary hypotheses and exclusion criteria before collecting the final dataset.

## 8. Recommended paper direction

### Working title

**Symmetry and Controller Adaptation in a Free-Radial Reconfigurable Legged Robot**

### Draft contribution statement

Subject to experimental validation, the paper could claim:

1. A controlled statistical characterization of how symmetry class and degree of
   asymmetry affect locomotion performance across continuously variable
   circumferential leg-attachment configurations.
2. A reproducible online-reconfiguration benchmark comparing controller
   adaptation mechanisms under leg detachment, reattachment, and angular
   repositioning.
3. A controlled morphology-information ablation comparing active-leg and exact
   angle input with missing, noisy, incorrect, or implicitly inferred morphology.
4. A quantitative analysis of transition behaviour, including interruption,
   safe-stance use, gait-phase reset, policy switching, replanning, relearning,
   and recovery time.
5. An open testbed, dataset, morphology splits, controller baselines, and analysis
   pipeline supporting the above studies.

If a morphology estimator is implemented and validated, add:

6. A self-configuration method that estimates active legs and radial attachment
   angles and is compared with morphology-blind and ground-truth-oracle control.

Items 1--5 are proposed contributions until supported by complete experiments.
The platform should not be presented as the sole novelty, and the self-estimation
claim should not be made unless an estimator is actually implemented.

### Minimum publishable version

The minimum credible study should include:

- Actual contact positions and projected CoM
- A complete forward locomotion trajectory
- At least three controller adaptation strategies
- Morphology-blind, active-leg-only, and exact-angle information ablations
- Detachment, angular repositioning, and arbitrary-angle reattachment conditions
- Held-out morphologies
- Repeated simulation trials
- Correct orientation measurements
- Proxy-versus-physical-stability analysis
- Explicit reporting of interruption, gait-phase reset, controller switching,
  and recovery time

### Stronger robotics version

A stronger paper would additionally include:

- Online morphology change during uninterrupted locomotion
- Force or contact sensing
- Fault-recovery experiments
- Energy measurement
- Physical robot validation or a careful sim-to-real study

## 9. Current evidence and limitations

The repository currently contains 42 historical CSV files, including one 30,000-row experiment. These files are useful for exploratory analysis and pipeline development, but they should not be the final paper dataset because they predate corrections to:

- CoppeliaSim alpha-beta-gamma versus yaw-pitch-roll labeling
- The one-step mismatch in reported lifted-leg state
- Reproducible seeds and run metadata

Additional limitations are:

- No actual foot-contact or force measurement
- Fixed assumed CoM projection
- No validated forward locomotion or foot trajectory
- Experimental rather than validated CPG mapping
- No energy measurement
- No physical robot validation

Final results must be recollected with the corrected runner and an expanded measurement model.

## 10. Primary-source reading list

1. Porta, J. M., and Celaya, E. “Reactive free-gait generation to follow arbitrary trajectories with a hexapod robot.” *Robotics and Autonomous Systems*, 47(4), 187–201, 2004. [DOI: 10.1016/j.robot.2004.04.001](https://doi.org/10.1016/j.robot.2004.04.001)
2. Valouch, D., and Faigl, J. “Gait-Free Planning for Hexapod Walking Robot.” ECMR, 2021. [DOI: 10.1109/ECMR50962.2021.9568834](https://doi.org/10.1109/ECMR50962.2021.9568834)
3. Liu, Y., Gao, H., Fan, X., Ding, L., Wang, J., and Liu, T. “Fault-Tolerant Tripod Gait Planning and Verification of a Hexapod Robot.” *Applied Sciences*, 10(8), 2959, 2020. [DOI: 10.3390/app10082959](https://doi.org/10.3390/app10082959)
4. Cully, A., Clune, J., Tarapore, D., and Mouret, J.-B. “Robots that can adapt like animals.” *Nature*, 521, 503–507, 2015. [DOI: 10.1038/nature14422](https://doi.org/10.1038/nature14422)
5. Ouyang, W., Chi, H., Pang, J., Liang, W., and Ren, Q. “Adaptive Locomotion Control of a Hexapod Robot via Bio-Inspired Learning.” *Frontiers in Neurorobotics*, 15, 627157, 2021. [DOI: 10.3389/fnbot.2021.627157](https://doi.org/10.3389/fnbot.2021.627157)
6. Nygaard, T. F., Martin, C. P., Torresen, J., Glette, K., and Howard, D. “Real-world embodied AI through a morphologically adaptive quadruped robot.” *Nature Machine Intelligence*, 3, 410–419, 2021. [DOI: 10.1038/s42256-021-00320-3](https://doi.org/10.1038/s42256-021-00320-3)
7. Burkus, E., Odry, Á., Awrejcewicz, J., Kecskés, I., and Odry, P. “Mechanical Design and a Novel Structural Optimization Approach for Hexapod Walking Robots.” *Machines*, 10(6), 466, 2022. [DOI: 10.3390/machines10060466](https://doi.org/10.3390/machines10060466)
8. Agheli, M., and Nestinger, S. S. “Force-based stability margin for multi-legged robots.” *Robotics and Autonomous Systems*, 83, 138–149, 2016. [DOI: 10.1016/j.robot.2016.05.012](https://doi.org/10.1016/j.robot.2016.05.012)
9. Habala, J., Margolis, G. B., Wang, T., Bhatt, P., He, J., Naeem, N., Xu, Z., Agrawal, P., Goldman, D. I., Luo, D., and Chong, B. “Agile asymmetric multi-legged locomotion: contact planning via geometric mechanics and spin model duality.” Preprint, 2026. [arXiv:2602.09123](https://arxiv.org/abs/2602.09123)
10. Feng, G., Zhang, H., Li, Z., et al. “GenLoco: Generalized Locomotion Controllers for Quadrupedal Robots.” Preprint, 2022. [arXiv:2209.05309](https://arxiv.org/abs/2209.05309)
11. Salemi, B., and Shen, W.-M. “Distributed Behavior Collaboration for Self-Reconfigurable Robots.” ICRA, 2004. [DOI: 10.1109/ROBOT.2004.1308927](https://doi.org/10.1109/ROBOT.2004.1308927) [Author PDF](https://robots.isi.edu/prl/salemi2004distributed-behavior-collaboration-for-self-reconfigurable.pdf)
12. Kim, J., Alspach, A., and Yamane, K. “Snapbot: A Reconfigurable Legged Robot.” IROS, 2017. [DOI: 10.1109/IROS.2017.8206477](https://doi.org/10.1109/IROS.2017.8206477)
13. Gim, S., and Kim, J. “Snapbot V2: A Reconfigurable Legged Robot with a Camera for Self-Configuration Recognition.” IROS, 2020. [DOI: 10.1109/IROS45743.2020.9341279](https://doi.org/10.1109/IROS45743.2020.9341279)
14. Wang, M., Su, Y., Liu, H., and Xu, Y. “WalkingBot: Modular Interactive Legged Robot with Automated Structure Sensing and Motion Planning.” RO-MAN, 2020. [DOI: 10.1109/RO-MAN47096.2020.9223474](https://doi.org/10.1109/RO-MAN47096.2020.9223474) [Author PDF](https://liuhx111.github.io/data/paper/ROMAN20_Walkingbot.pdf)
15. Hayakawa, T., Kamimura, T., Kaji, S., and Matsuno, F. “Autonomous Distributed System for Gait Generation for Single-Legged Modular Robots Connected in Various Configurations.” *IEEE Transactions on Robotics*, 36(5), 1491–1510, 2020. [DOI: 10.1109/TRO.2020.2992983](https://doi.org/10.1109/TRO.2020.2992983)
16. Hayakawa, T., and Matsuno, F. “Autonomous Distributed System for Single-Legged Modular Robots to Traverse Environments by Adaptive Reconfiguration.” *Robotics and Autonomous Systems*, 155, 104152, 2022. [DOI: 10.1016/j.robot.2022.104152](https://doi.org/10.1016/j.robot.2022.104152)
17. Whitman, J., Travers, M., and Choset, H. “Learning Modular Robot Control Policies.” *IEEE Transactions on Robotics*, 39(5), 4095–4113, 2023. [DOI: 10.1109/TRO.2023.3284362](https://doi.org/10.1109/TRO.2023.3284362)
18. Yu, C., Matthews, D., Wang, J., et al. “Agile Legged Locomotion in Reconfigurable Modular Robots.” *Proceedings of the National Academy of Sciences*, 123(10), e2519129123, 2026. [DOI: 10.1073/pnas.2519129123](https://doi.org/10.1073/pnas.2519129123) [Open-access record](https://pubmed.ncbi.nlm.nih.gov/41790937/)

## 11. Literature-search TODO

Before drafting the related-work section:

- Verify all bibliographic metadata in a reference manager.
- Search forward citations of CONRO, Snapbot, KARAKASA, WalkingBot, Whitman et
  al. (2023), and Yu et al. (2026), in addition to the supporting fixed-morphology
  literature.
- Search specifically for robots with continuously movable leg-base or shoulder positions.
- Search for online morphology transitions in hexapods and other multi-legged
  robots, recording whether each method stops, resets phase, switches policy,
  replans, or relearns.
- Search for morphology-conditioned CPG coupling and phase adaptation.
- Search for public benchmarks covering variable leg placement.
- Separate physical attachment or detachment from simulated damage, disabled
  actuators, and pre-operation manual assembly.
- Add exact comparison details from methods and experiment sections, not only abstracts.
- Separate peer-reviewed literature from preprints in the paper.
