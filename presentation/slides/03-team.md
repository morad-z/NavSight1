# Slide 03: Team & Collaboration

**Section:** Opening · **Slide:** 3 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **Morad Zubidat** (ID [ID REDACTED]) — IMU pre-integration, sensor fusion, sensor sync & calibration, integration testing & documentation
- **Roey Ben Harush** (ID [ID REDACTED]) — Android / Jetpack Compose UI, JNI bridge *(to confirm)*
- **Tamir Sobuh** (ID [ID REDACTED]) — C++ native VIO core / vision front-end *(to confirm)*
- **Supervisor:** Mr. Amit Dunsky
- **Collaboration:** per-member feature branches → merge to `master`; deterministic C++ replay harness + CI before every on-device walk
- **Team conventions:** "fix the root cause, don't disable" · "validate on real-walk data before tuning constants"
- [Diagram: diagrams/01-system-architecture.md — 4 tiers annotated with owners]

## Talking Points (what the presenter SAYS)
- "We are a team of three, supervised by Mr. Amit Dunsky."
- "I'm Morad — my confirmed ownership is the inertial side: IMU pre-integration, sensor fusion into the EKF, sensor synchronization and calibration, plus the integration-testing harness and documentation."
- "Roey owns the Android and Jetpack Compose UI and the JNI bridge; Tamir owns the C++ native VIO core and the vision front-end — those splits we're noting as to-confirm with the team."
- "We divided the work along the architecture's natural seams — UI, JNI bridge, native core, and validation — so each member owned a tier with a clear interface, which kept coupling low."
- "We collaborated through per-member feature branches merged into master, and we gated every on-device walk behind a deterministic C++ replay harness and CI, so regressions were caught on recorded data before they ever reached the phone."

## Why We Chose This Approach
- **Alternatives considered:** a single shared development branch, or splitting by file type. We rejected the shared branch (constant conflict on the same C++/Kotlin files) and file-type splitting (no clear ownership of behavior).
- **Tradeoffs accepted:** per-member branches require disciplined merging into `master`, and cross-tier changes (e.g. a new JNI field) need coordination across two owners.
- **Benefits gained:** ownership maps onto the 4-tier architecture, so each person owns one tier behind a stable interface — the JNI `VioData` contract decoupled Kotlin work from C++ work, letting UI and core evolve in parallel.
- **Engineering reasoning:** the replay harness made the team conventions enforceable — "validate on data before tuning" is only possible if anyone can re-run a recorded ride deterministically. This is exactly why the gravity-drift debugging episode — where a ~6-10° tilt mis-cancelled gravity into ~800 m of phantom Z drift, and the fix came from reading the residual data rather than tuning constants — became the cautionary rule we now follow.

## Potential Questions (Defense)
**Q:** Which parts did *you personally* build and own?
**A:** My confirmed ownership is the inertial half of the pipeline: the IMU pre-integration module (Madgwick attitude plus gyro/accel preintegration), the sensor-fusion integration into the 15-DOF error-state EKF, sensor synchronization and calibration (including the Allan-calibrated noise model and timestamp alignment), and the integration-testing and documentation effort — notably the deterministic replay harness used to validate every fix before walking.

**Follow-up Q:** How did you keep three people from breaking each other's code on a tightly-coupled VIO system?
**Follow-up A:** Two mechanisms. First, ownership followed the 4-tier architecture, so each member worked behind a stable interface — the Kotlin↔C++ boundary is the JNI contract — the `processCameraFrameDirect` entry point (9 arguments) and the 30-field `VioData` return type — which let the UI and the native core change independently. Second, every behavior-changing edit had to pass the deterministic C++ replay harness and CI scoring on recorded fixtures before an on-device walk — so regressions surfaced on identical recorded input, not in the field.

## Speaker Notes
- Be explicit that the non-Morad subsystem assignments are "to confirm" — say it once, clearly, so the panel knows the team will confirm exact splits.
- Have ready the architecture seam mapping: Tier 1 (Compose UI) → Roey; Tier 3 (JNI bridge) → Roey; Tier 4 (native C++ VIO core / vision) → Tamir; inertial + fusion + calibration + validation harness → Morad (confirmed).
- If asked about contribution evidence: git history corroborates active contributors across the per-member branches (`morad`/`morad-ui`, `tamir-*`, Roey's Shenkar email), but do not over-quote raw commit counts as a measure of contribution — frame it as branch topology supporting the split.
- Pitfall: don't claim ownership of subsystems outside the confirmed set. Stay precise: confirmed = inertial/fusion/calibration/integration-testing; everything else = team to confirm.
- Tie the conventions back to the deck's spine: "fix root cause, don't disable" and "validate on data before tuning" both reappear in the Engineering Challenges slide (09) — flag that this is where the team culture came from.
