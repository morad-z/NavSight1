---
name: "NavSight Implementor"
description: "Implementation discipline contract for the NavSight repo. Use BEFORE any Edit/Write/NotebookEdit under app/src/main/cpp, app/src/main/java/com/example/navsight1, tests/cpp, or scripts that adds or changes runtime behavior. Use BEFORE responding to any 'fix', 'implement', 'add', 'wire', 'refactor', 'extract', 'rename', 'replace', 'improve', 'make it do X' request. Enforces: (1) plan adherence per docs/study/post_v19_sprint_plan.md and docs/study/phase2_productization_plan.md — no jumping ahead of step order; (2) ban on workarounds, defensive clamps, magic-number tweaks, symptom-patches — root-cause fixes only; (3) mandatory debug instrumentation (LOGI + EventCounters) at every state transition and frame/convention boundary; (4) comment-out instead of delete; (5) articulate cause / change / falsifier before writing the fix; (6) build + walk validation before declaring done."
---

# NavSight Implementor

## TL;DR

This skill is the binding implementation contract for NavSight. Before touching production code, run the pre-flight checklist; while writing code, follow the anti-patterns ban and the instrumentation requirements; after writing, gate on build + verification. The point is to break the multi-week pattern of patches that hide bugs and force every change to be plan-aligned, root-cause-grounded, and post-hoc debuggable from logs alone.

**One-line invariant**: code written without a stated plan step, a stated root cause, and per-state-transition logging is not allowed to ship.

---

## When to use

Auto-invoke this skill BEFORE any of:

1. `Edit`, `Write`, or `NotebookEdit` on:
   - `app/src/main/cpp/**` (any C++ in the VIO core)
   - `app/src/main/java/com/example/navsight1/**` (any Kotlin)
   - `tests/cpp/**` (any C++ test)
   - `scripts/**` if the script affects runtime / device behavior (build, install, sim recording)
2. Any user message containing the verbs: "fix", "implement", "add", "wire", "refactor", "extract", "rename", "replace", "improve", "make it do X", "ship", "land"
3. Any commit, PR creation, or release-tag work

Do NOT skip this skill for "small" changes. The patches that ate this week's debugging time were all single-line "small" changes.

---

## Pre-flight checklist (run before writing ANY code)

Walk through every item. Skipping an item is a deviation and must be flagged to the user explicitly.

### 1. Identify the plan step

- [ ] Read the current status section of `docs/study/post_v19_sprint_plan.md` (Phase 1).
- [ ] If past Phase 1, read `docs/study/phase2_productization_plan.md`.
- [ ] State out loud: "This change implements Step N (sub-task M) of Phase X." If it doesn't fit a planned step, this is a **deviation** — pause and ask the user for explicit approval to proceed.
- [ ] Verify earlier steps in plan order are complete. Do NOT jump ahead. Example trap: building Step 6 (persistent landmark map) while Step 5 (pose-graph back-end) is still queued.

### 2. State the root cause (not the symptom)

Before writing the fix, write three statements in your response:

- [ ] **Cause** — what is mechanically wrong, at the level of math / data flow / state. Not "loop closure didn't help", but "target_R_GtoI is composed using R_bc at query time but the keyframe was stored with a different R_bc, so the residual carries a frame asymmetry of magnitude X".
- [ ] **Change** — exactly what code change addresses that cause, and why.
- [ ] **Falsifier** — what observation would prove the cause hypothesis wrong. If you can't name one, the cause statement isn't real yet — go back and dig until you have one.

If any of the three is missing, the fix isn't ready. Stop and either (a) gather more data, or (b) tell the user you don't have enough information yet.

### 3. Check existing skills before solving from scratch

- [ ] If the work touches VIO/EKF math: invoke `navsight-vio-specialist`.
- [ ] If it's UI / Kotlin / lifecycle / sensor permission: invoke `navsight-android-specialist`.
- [ ] If it requires reading sim data or logcat to diagnose: invoke `navsight-sim-debugging`.
- [ ] If a relevant external library: invoke `docs-lookup` for current API rather than guessing.

---

## Anti-patterns (BANNED — never ship these)

If implementing the right fix requires any of these, STOP and report the actual root cause to the user. Do not ship the workaround.

### 1. Defensive output clamps

```cpp
// ❌ BANNED — hides the upstream bug producing the bad value
if (delta_p_norm > 200.0) delta_p_norm = 200.0;
if (rho < 1e-9) rho = 1e-9;
```

Why: clamps make the symptom invisible while the cause continues to inject garbage into the state. The clamped value is also wrong — it's just less wrong, in a way that the next module silently consumes. Find what produced the bad value and fix it there.

Allowed exception: clamps **at trust boundaries**, with structured log line documenting frequency and magnitude of clamp activation, AND a tracking counter in `EventCounters.h`. If a clamp ever fires in production, you must be able to grep for it post-hoc.

### 2. Symptom gates ("reject if it looks bad")

```cpp
// ❌ BANNED — masks the wrong-frame bug producing the residual
if (m2_R > 16.27) return false;  // "block bad rotation corrections"
if (target_p_norm > 200.0) return false;  // "block 367km outliers"
```

If a gate is needed to make the system stable, the system isn't stable — find the upstream wrong-frame / wrong-update / wrong-state and fix it there. A gate is acceptable only as a **safety net for known-rare failure modes**, with a tracking counter and a clear comment explaining what real failure mode it catches (not "it makes things feel better").

### 3. Magic-number tweaks without data

```cpp
// ❌ BANNED — adjusting a threshold without quoting the data that demands the change
LOOP_CLOSURE_DRIFT_RATE = 0.05;  // was 0.032, "feels right"
sigma_axis_sq = 1e-3;            // was 1e-4, "less aggressive"
```

If a constant changes, the commit message and a comment in the code MUST cite the specific sim file, log line, and measurement that justifies the new value. The team has a `navsight-sim-debugging` skill specifically because "read the data before tuning" is the cardinal rule.

### 4. Silent error swallowing

```cpp
// ❌ BANNED — failure becomes invisible
if (!ekf_.getClonePose(id, R, p)) return;  // function silently no-ops
try { risky(); } catch (...) { /* ignore */ }
```

Every failure path must log (with counter) what failed and why. The two patterns we lost days to — `loop_closure_corrections_applied` saying 33 while every damp line was `ok=0`, and SLAM features going to ρ→0 — were both invisible-failure bugs.

### 5. Disabling code with `#if 0` / `if (false)`

```cpp
// ❌ BANNED — what looks like working code isn't
#if 0
applyMSCKFUpdate(...);
#endif
```

If code is wrong, fix it or comment it out with a date + reason explaining what specifically replaced it. `#if 0` and `if (false)` make CI green while production silently regresses.

### 6. "TODO: come back later" hiding broken paths

```cpp
// ❌ BANNED unless paired with an issue/plan ref
// TODO: this is wrong, fix later
return identity_pose();
```

If you must defer a fix, the TODO must reference (a) the open issue or plan step that owns the deferral, (b) the date deferred, (c) the impact while deferred. Otherwise the path is invisible to future-you reading git blame.

### 7. Wiring functions you don't understand

If you find an orphan function (defined but never called) and the temptation is "just wire it up to fix things", STOP. Read the function. Trace its math. Compare to canonical references. The `UpdaterSLAM::update_skf` wiring on 2026-05-12 was correct because the function was OpenVINS-canonical; but wiring without verification can silently activate broken code.

### 8. Celebrating metric improvements when user behavior is unchanged

Per [[feedback_no_metric_celebration]]: if close-loop drops 3.44 → 1.68 but orange dots still don't anchor and loops still don't overlay, **the system is still broken**. Report what the user can see, not the chart.

---

## Required instrumentation (every change)

Implementations without instrumentation are not done. Specifically:

### 1. LOGI per state-transition function

Every new function whose body contains non-trivial conditional logic emits at least one structured log line. Format:

```cpp
LOGI("PHASE_NAME: key1=%.3f key2=%d key3=%s state=%s",
     value1, value2, descriptor, state_descriptor);
```

Use space-separated `key=value` so it's grep-able. Example:

```cpp
LOGI("SLAM_PROMOTE: fid=%d slot=%d rms_px=%.2f anchor_clone=%d depth_m=%.2f baseline_m=%.4f",
     fid, slot, rms, anchor_clone_id, zC, baseline);
```

### 2. Counted log line at every gate / decision point

Any `if` that decides "promote/demote/accept/reject/skip" emits a log line AND increments a counter in `EventCounters.h`:

```cpp
if (rms > 1.5) {
    navsight::eventCounters().slam_promotion_rejected_rms.fetch_add(
        1, std::memory_order_relaxed);
    LOGI("SLAM_PROMOTE_REJECT: reason=rms fid=%d rms=%.2fpx thresh=1.5", fid, rms);
    continue;
}
```

Why two: the LOGI lets you grep specific rejections; the counter gives you aggregate stats in `event_summary` JSON for sim diffing. We've eaten days because one or the other was missing.

### 3. Frame / convention boundary logging

Every place where data crosses a known-fragile convention boundary logs on both sides:

- Body ↔ camera (R_bc applications)
- Z-up ↔ Y-up (JNI boundary, in `native-lib.cpp`)
- EKF state ↔ legacy `global_t_` / `global_R_` (Tracker pose mirror sites)
- World ↔ image (projection / triangulation)

Example:

```cpp
LOGD("FRAME_BOUNDARY: site=clone_storage R_GtoI=[...] R_bc=[...] R_GtoC_stored=[...]",
     ... matrix-printing macros ...);
```

These should be `LOGD` (verbose), gated by a debug flag if too noisy in production.

### 4. EventCounters entry for every new gate/decision

For every new decision point, add a `std::atomic<long long>` field to `app/src/main/cpp/EventCounters.h` and increment it in the relevant branch. Field name convention: `<phase>_<outcome>` (e.g., `slam_promotion_rejected_rms`). The simulator JSON's `event_summary` will pick it up automatically and you can diff it across walks.

### 5. Pre-fix log capture

Before fixing a bug, log the symptom IN A WAY THAT PROVES IT'S GONE after the fix. Example: if fixing ρ→0 divergence, log per-frame ρ trajectory so the next walk shows ρ stable instead of decaying. The fix isn't validated until the symptom log shows the fix worked.

---

## Verification gates (after implementation)

A change is NOT done until all of these pass:

### 1. Android build green

```bash
./gradlew.bat assembleDebug
```

Must end with `BUILD SUCCESSFUL`. Warning-mode-all is optional, but no new warnings should be introduced.

### 2. C++ unit tests (if applicable)

If the change touches `EKFState`, `IMUPreintegrator`, `Tracker`, any `Updater*`, `LoopClosureDetector`, or `FeatureManager`, the matching test files in `tests/cpp/` should still pass when the host can compile them. If a test has been broken by the change, either fix the test (the convention changed) or fix the code (the test caught a regression). Never silently delete or skip a test.

### 3. Real-walk validation (per [[feedback_follow_plan_rules]])

Code written ≠ step closed. The fix is complete only after:

- A new sim is recorded with the modified APK
- The expected behavior change is visible in the data (use the symptom log added in §5 of Required Instrumentation)
- The user has visually confirmed in the running app (per [[feedback_no_metric_celebration]] — metric deltas alone don't count)

Until those land, the work is "in flight", not "done".

### 4. Memory update

If the change discovers a non-obvious lesson, save it to `~/.claude/projects/C--Users-morad-AndroidStudioProjects-NavSight1/memory/` as a `feedback_*.md` or `project_*.md` entry, and add to `MEMORY.md`. The lesson must be specific enough that future-Claude reads it and avoids the same mistake.

---

## Existing discipline this skill ties into

This skill enforces and references existing project rules:

- [[feedback_no_deletions]] — comment out, don't delete. New code path replaces old? Wrap old in `/* ... */` with a date + reason header. Do not `rm`.
- [[feedback_no_disabling]] — if plan-introduced code regresses, fix the root cause. Don't gate behind a flag or revert to a stub.
- [[feedback_no_metric_celebration]] — when the chart improves but the user-visible behavior is still broken, report the system as broken, not as winning.
- [[feedback_follow_plan_rules]] — code written ≠ step closed. Acceptance criteria need real-data verification.
- [[feedback_explicit_commit_only]] — never auto-commit. Wait for the user to type "commit" in the most recent prompt.
- [[feedback_no_metric_celebration]] — lead reports with what the user can see, not with metric deltas.
- [[feedback_branch_workflow]] — always work on `morad`, never on `master` directly.

---

## Examples — patch vs root-cause fix

### Example A: SLAM ρ → 0 divergence (2026-05-12)

**Symptom**: SLAM feature inverse-depth ρ converges toward zero, producing `target_p` at hundreds of kilometres, polluting PnP.

**❌ Patch path** (what I almost shipped):
```cpp
// In Tracker.cpp at pts3d_world assembly:
if (p_w_norm > 200.0) continue;  // skip diverged SLAM features
```
This clamp hides the symptom. Diverged features still exist in the EKF state, still consume covariance bandwidth, still drag the pose update.

**✅ Root-cause path** (what shipped):
- Cause: `UpdaterSLAM::update_skf` (which has parallax + depth-observability + per-obs chi² gates) was extracted but never wired. `Tracker.cpp:2317` still called the older `updateSlamFeature` which has no gates. Without those gates, the EKF runs Kalman updates on (α,β,ρ) when geometry doesn't support depth observability → ρ random-walks → eventual divergence.
- Change: Wire `slam_updater_.update_skf(...)` at the call site. Add `UpdaterSLAM slam_updater_` member to Tracker. Add `UpdaterSLAM.cpp` to CMakeLists.
- Falsifier: a walk with this fix should show ρ stable (LOGI per-frame ρ trajectory bounded) and no `target_p` outliers > 100m. If we still see km-range targets, the wiring isn't the only cause.

This is the kind of fix this skill exists to enforce.

### Example B: Loop closure "applied" but ok=0

**Symptom**: `loop_closure_corrections_applied = 33` in event_summary, but every `LOOP_CLOSURE: damp` log line has `ok=0`. User says "corrections aren't doing anything".

**❌ Patch path**:
```cpp
// Force ok=1 if chi² is close to passing
if (m2 < kChi2Threshold * 1.1) return true;  // "let it through"
```

**✅ Root-cause path**:
- Cause: `updateRelativeRotation` at `EKFState.cpp:951` treats clone.R_GtoC as world→body, but after the 2026-05-09 Option-C migration, clone.R_GtoC stores world→camera (composed with R_bc). The composition produces a residual off by R_bc on every call.
- Change: Strip R_bc inside the function: `R_GtoI_clone = R_bc.t() * R_clone`. Update the clone Jacobian to `H_clone = -R_meas_body * R_bc.t()` for the camera-frame perturbation.
- Falsifier: post-fix walk should show `r_R.z` sign-skew drop from 17:1 toward 1:1 and chi² PASS rate rise from ~1% toward majority. If skew stays high, the convention bug isn't in this function alone.

### Example C: Orange dots don't appear

**Symptom**: SLAM features promoted but only 6 SLAM_RMS log lines per 4-min walk. User sees almost no orange dots.

**❌ Patch path**:
```cpp
// Loosen the parallax gate to fire more often
options.min_parallax_ratio = 0.001;  // was 0.01
```
This re-introduces the exact failure mode UpdaterSLAM's gate was added to prevent.

**✅ Root-cause path**:
- Cause: `getPromotableFeatures` gates on `lc.age >= 8` (raw frame count). But `pruneObservations` strips entries whose clone is out-of-window every frame. A feature with age=8 can have all 8 observations referencing marginalized clones → `obs->size() < 2` → silent rejection in the triangulation loop. The age gate tests the wrong thing.
- Change: Count in-window observations directly. Pass `ekf_.getWindow()` IDs to `getPromotableFeatures`, count how many `active_tracks_[fid]` entries reference in-window clones, require ≥ 2.
- Falsifier: post-fix walk should show promotions distributed across the walk (not a single burst), and the per-frame `promote_us` time spent triangulating-then-rejecting should drop. If promotions still cluster in one window, there's another gate upstream.

---

## What this skill does NOT do

- Decide architecture (that's the plan's job — read `docs/study/`)
- Decide priorities (that's the user's call)
- Replace `navsight-vio-specialist` / `navsight-sim-debugging` / `navsight-android-specialist` — those carry domain knowledge; this skill is a discipline enforcer that runs alongside them

---

## Snippet templates

### EventCounters entry

```cpp
// app/src/main/cpp/EventCounters.h, inside the struct:
std::atomic<long long> slam_promotion_rejected_rms{0};
std::atomic<long long> slam_promotion_rejected_baseline{0};
std::atomic<long long> slam_promotion_rejected_obs_count{0};
```

### Counted gate

```cpp
if (rms > kRmsThreshold) {
    navsight::eventCounters().slam_promotion_rejected_rms.fetch_add(
        1, std::memory_order_relaxed);
    LOGI("SLAM_PROMOTE_REJECT: reason=rms fid=%d rms=%.2fpx thresh=%.2f",
         fid, rms, kRmsThreshold);
    continue;
}
```

### Frame-boundary log

```cpp
// At native-lib.cpp Z-up → Y-up conversion:
LOGD("JNI_FRAME_BOUNDARY: site=getSlamSnapshot fid=%d "
     "zup=(%.3f,%.3f,%.3f) yup=(%.3f,%.3f,%.3f)",
     fid, zup_x, zup_y, zup_z, yup_x, yup_y, yup_z);
```

### Pre-fix symptom log

```cpp
// Add BEFORE the fix lands:
LOGI("SLAM_RHO_TRAJ: fid=%d slot=%d rho=%.6e depth_m=%.2f t_since_promote_s=%.2f",
     fid, slot, rho, 1.0/rho, age_since_promote_s);
// After the fix lands, this log line should show rho stable.
// If it still decays, the fix didn't address the root cause.
```

---

## Failure mode of this skill

If invoking this skill ever results in "OK I'll apply the patch anyway because the user is in a hurry", the skill failed. The point is that the patches cost MORE time than the discipline saves — multi-day debugging sessions where the symptom got moved around because no fix touched the cause. Stay disciplined.

If the user explicitly overrides with "I know it's a patch, ship it anyway as a known-debt fix", document the debt in `feedback_*.md` memory AND in a code comment with date + reference to the planned proper fix.

---

**Bound to**: NavSight project at `C:\Users\morad\AndroidStudioProjects\NavSight1`
**Companion skills**: [[navsight-vio-specialist]], [[navsight-sim-debugging]], [[navsight-android-specialist]]
**Lessons that produced this skill**: 2026-05-09 → 2026-05-12 multi-day SLAM-anchoring debug; 5 frame-convention bugs in 4 days; the UpdaterSLAM dead-code discovery; the gyro-bias-feedback-loop bug; the MSCKF .t() inversion; the celebrating-metrics-while-broken pattern.
