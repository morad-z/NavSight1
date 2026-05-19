---
name: "NavSight Sim Debugging"
description: "Data-first debugging for NavSight via simulator JSON, logcat, and the replay harness. Use when analyzing a sim, parsing logcat, running the replay harness, diagnosing drift, characterizing trajectory jumps, or validating any fix on real-walk data BEFORE changing constants. Carries the chi² fix history as a cautionary tale: the team spent 3 days tuning σ before reading the data."
---

# NavSight Sim Debugging

## Overview

NavSight produces three kinds of debugging artifacts per walk: a simulator JSON, a sibling frames directory, and Android logcat. This skill carries the workflow for pulling them, parsing them, and reaching defensible conclusions about VIO bugs — without reaching for parameter tuning until the data tells you what's broken.

## When to use

Trigger on any of:
- "analyze sim", "diagnose drift", "characterize jumps"
- "logcat parse", "logcat from the walk"
- "replay harness", "replay scorer"
- "validate fix on data", "before I change a constant"
- "what's the trajectory doing"
- After every walk, before proposing any threshold change

## The cardinal rule

> **Read the data before tuning the constants.**

Three days were lost to chi² tuning when the actual bug was 800 m of phantom Z drift in `p_G`. The chi² gate was correctly rejecting absurd corrections; the team kept loosening σ. One run of `scripts/analyze_chi2_rejections.py` ended the war in 20 minutes.

When you reach for a constant, ask:
1. What's the actual residual on a real walk?
2. Is the residual physically plausible?
3. If not, what state is corrupted upstream?

Only after those three answers should any constant change.

## Workflow

### 1. Pull the sim from the phone

```powershell
$ADB = "C:\Users\morad\AppData\Local\Android\Sdk\platform-tools\adb.exe"
$DEST = "C:\Users\morad\AndroidStudioProjects\NavSight1\tests\sims\regression\visual"
$SRC  = "/sdcard/Android/data/com.example.navsight1/files"

# List recent sims
& $ADB shell "ls -lt $SRC | grep -E 'simulation_data_(17783)' | head -10"

# Pull JSON + frames pair
& $ADB pull "$SRC/simulation_data_<TS>.json"   "$DEST\<NAME>.json"
& $ADB pull "$SRC/simulation_data_<TS>.frames" "$DEST\<NAME>.frames"
```

**Use PowerShell**, not Git Bash — Bash mangles `/sdcard/...` paths via MSYS path translation.

### 2. Pull logcat (for the LC_ABS chi² diagnostic block)

```powershell
& $ADB logcat -d 2>&1 | Out-File -Encoding utf8 "$DEST\<NAME>.logcat.txt"
```

Logcat is a ring buffer. Pull it within minutes of the walk or it rolls off. If you need persistent capture, `adb logcat -G 16M` bumps the buffer size.

### 3. Parse the JSON event_summary

Every sim's `event_summary` block carries the EventCounters snapshot at recording-stop time. Key counters to read first:

| Counter | What it tells you |
|---|---|
| `total_path_dm` | Distance walked × 10 (decimetres). Sanity-check: matches the user's reported route length? |
| `loop_closure_attempts` | BoW worker tick count |
| `loop_closure_accepts` | BoW + PnP passed |
| `loop_closure_chi2_rejected` | BoW accepted but EKF chi² gate rejected — count of damping-ramp frames |
| `loop_closure_corrections_applied` | Successful EKF injections |
| `loop_closure_geom_attempts/accepts/rejects_*` | Step 7.1 geometric path |
| `slam_promotions_total` | SLAM features promoted into EKF state |
| `slam_lifetime_obs_sum / count` | Mean SLAM feature lifetime in observations |
| `msckf_huber_rejected_sum` | Per-row Huber rejections in MSCKF (high count = visual front-end is noisy) |
| `extrinsics_rotation_angle_mdeg` | R_bc drift from initial. Should stay < 1000 (1°) on a healthy walk |

If `loop_closure_corrections_applied = 0` AND `loop_closure_chi2_rejected > 0` → corrections being rejected. That's when to dig into LC_ABS logcat lines.

### 4. Run the chi² analyzer

```bash
python scripts/analyze_chi2_rejections.py \
  tests/sims/regression/visual/<NAME>.logcat.txt
```

Outputs:
- Block dominance (m²_p vs m²_R) — tells you whether position or rotation residual is the killer
- Residual magnitudes (median + p95)
- p_G evolution over time
- σ_p needed to admit observed residuals

If σ_p needed is > 30× current → the EKF state is corrupted, not σ_p tuning territory.

### 5. Characterize trajectory jumps

```python
import json, math
with open("tests/sims/regression/visual/<NAME>.json") as fh:
    d = json.load(fh)
pts = d["points"]

# Group consecutive >0.4m jumps into clusters (one cluster = one loop-closure ramp)
events = []; cur = []
for i in range(1, len(pts)):
    a, b = pts[i-1], pts[i]
    d = math.sqrt((b["vx"]-a["vx"])**2 + (b["vy"]-a["vy"])**2 + (b["vz"]-a["vz"])**2)
    if d > 0.4: cur.append({"i":i, "d":d, "ts":b["ts"]})
    else:
        if len(cur) >= 2: events.append(cur)
        cur = []

# Per cluster: pre-position, post-position, total shift
for ev in events:
    pre = pts[ev[0]["i"]-1]
    post = pts[ev[-1]["i"]]
    pre_mag  = math.sqrt(pre["vx"]**2 + pre["vy"]**2 + pre["vz"]**2)
    post_mag = math.sqrt(post["vx"]**2 + post["vy"]**2 + post["vz"]**2)
    print(f"|pre|={pre_mag:.2f}  |post|={post_mag:.2f}  delta={post_mag-pre_mag:+.2f}")
```

Interpretation: if corrections push trajectory AWAY from origin on a closed loop, they're false positives or wrong-direction. Ratio of toward-origin vs away-from-origin clusters tells you whether loop closure is helping or thrashing.

### 6. Replay harness (offline reproduction)

```bash
cd tests/cpp
cmake --build build --target replay_harness --config Release

./build/replay_harness \
  ../sims/regression/visual/<NAME>.json \
  build/replay_out/<NAME>.csv \
  --frames-dir ../sims/regression/visual/<NAME>.frames \
  --frame-match-tolerance-ms 50

python replay_scorer.py build/replay_out/<NAME>.csv \
  --max-heading-rmse-rad 0.20 \
  --max-drift-per-meter 2.0 \
  --max-loop-gap-m 10.0 \
  --min-inlier-ratio 0.40 \
  --json-out build/replay_out/<NAME>.metrics.json
```

The harness produces a per-frame CSV (pose + visual stats + keyframe_stored flag) and a sidecar `event_counters.json`. The scorer reports the same metrics CI uses.

When validating a fix: run the harness on the SAME sim before AND after, compare metrics. The before/after diff is your evidence.

## Sim JSON schema (current)

```jsonc
{
  "startTime": <ms>,
  "event_summary": { ... full EventCounters dump ... },
  "frames_meta": {
    "dir": "simulation_data_<ms>.frames",
    "written": <int>,
    "dropped": <int>,
    "first_ts_ns": <wall_clock_ns>,
    "last_ts_ns":  <wall_clock_ns>
  },
  "points": [
    {"ts": <ms>, "vx": ..., "vy": ..., "vz": ...,        // Tracker output (Y-up)
     "vyaw": ..., "vsc": ..., "vql": ...,
     "rx": ..., "ry": ..., "rz": ..., "ryaw": ...,       // raw camera-frame VO
     "ax": ..., "ay": ..., "az": ...,                    // body accel
     "gx": ..., "gy": ..., "gz": ...,                    // body gyro
     "glat": ..., "glng": ..., "galt": ..., "gacc": ...,  // GPS (nullable)
     "mflow": ..., "inl": ...,                            // mean flow, inlier count
     "steps": ..., "sfreq": ..., "stride": ...,           // PDR
     "pflags": ..., "hdg": ...                            // pose flags + heading
    },
    ...
  ]
}
```

Note `vio.x/y/z` is the **JNI Y-up swap** of the internal Z-up `output.t`. So `vy` in the JSON = `output.t[Z]` internally = "up". Don't confuse "vy" in the JSON with "world Y".

## logcat tags to grep

| Tag | What |
|---|---|
| `NavSight-EKF` | LC_ABS, gravity-alignment, MSCKF updates |
| `NavSight-Tracker` | Per-frame pipeline, loop-closure ramp |
| `NavSight-VioEngine` | Engine lifecycle |
| `NavSight-Native` | JNI lifecycle |
| `LoopClosureDetector` | BoW worker, geom path, PnP |
| `SimFrameRecorder` | Recorder start/stop/drops |

LC_ABS line format (every loop-closure ramp frame):
```
LC_ABS: r_R=[..] r_p=[..] p_G=[..] target_p=[..]
        var_R=.. var_p=.. m2=.. m2_R=.. m2_p=.. thresh=22.5
```

This is the chi² block — the diagnostic we should have read on day 1.

## Fixture layout for CI

`tests/sims/regression/` (top-level): IMU-only sims, `replay` job
`tests/sims/regression/visual/`: visual sims (JSON + sibling `.frames/` dir), `replay-visual` job
`tests/sims/regression/visual/<name>_loop.json`: triggers `--min-loop-closures 1` per-fixture override

CI workflow: `.github/workflows/replay.yml`. The visual job auto-no-ops with a clear log line until first fixture lands (gracefully fails open).

## Implementation Playbooks

Every playbook is a complete procedure. Run all steps. No shortcuts. Don't skip the data-reading step to save time — that's the trap that cost 3 days.

### Playbook A — Diagnose "loop closure looks broken"

**Step A1 — Pull the sim + logcat from the phone.**
```powershell
$ADB = "C:\Users\morad\AppData\Local\Android\Sdk\platform-tools\adb.exe"
$DEST = "<project>\tests\sims\regression\visual"
$SRC  = "/sdcard/Android/data/com.example.navsight1/files"
& $ADB pull "$SRC/simulation_data_<TS>.json"   "$DEST\<NAME>.json"
& $ADB pull "$SRC/simulation_data_<TS>.frames" "$DEST\<NAME>.frames"
& $ADB logcat -d 2>&1 | Out-File -Encoding utf8 "$DEST\<NAME>.logcat.txt"
```

**Step A2 — Read `event_summary` from the JSON.**
Parse with Python; print every loop_closure_* counter. Identify which counter is anomalous compared to a healthy walk.

**Step A3 — If `chi2_rejected > 0` AND `corrections_applied = 0`, run the chi² analyzer.**
```bash
python scripts/analyze_chi2_rejections.py tests/sims/regression/visual/<NAME>.logcat.txt
```

**Step A4 — Categorise from the analyzer output.**
- 100% position-dominated AND median |r_p| > 30 m on a 100 m walk → propagation bug. Suspects: R_GtoI tilt → gravity miscancel → p_G drift. Switch to navsight-vio-specialist Playbook B.
- 100% rotation-dominated AND |r_R| > 30° → frame convention bug. Switch to navsight-vio-specialist Playbook C.
- Mixed → check if both blocks are individually plausible but cross-correlation in S blows up the joint m².

**Step A5 — If acceptances exist but trajectory still bad, run the cluster analyzer (Playbook B).**

**Step A6 — Form ONE hypothesis, propose fix, validate offline.**
Run replay harness on the same sim with the proposed fix. Compare metrics before/after. Fix only ships if it improves on the existing fixture without regressing other walks.

### Playbook B — Characterise trajectory jumps / loop-closure clusters

**Step B1 — Find consecutive >0.4m jumps (each cluster is one ramp).**
```python
import json, math
with open("tests/sims/regression/visual/<NAME>.json") as fh:
    pts = json.load(fh)["points"]
events = []; cur = []
for i in range(1, len(pts)):
    a, b = pts[i-1], pts[i]
    d = math.sqrt(sum((b[k]-a[k])**2 for k in ("vx","vy","vz")))
    if d > 0.4: cur.append(i)
    else:
        if len(cur) >= 2: events.append(cur)
        cur = []
```

**Step B2 — Compute pre/post position magnitude per cluster.**
For each cluster: `|pre|` (position right before the cluster) and `|post|` (position right after). On a closed-loop walk, corrections should reduce `|post|` (toward origin).

**Step B3 — Classify each cluster.**
- `|post| < |pre|` (toward origin) → good correction
- `|post| > |pre|` (away from origin) → false positive
- More than 50% away-from-origin → loop closure is thrashing on bad matches

**Step B4 — If thrashing, drill into individual matches.**
Grep logcat for `LOOP_CLOSURE: ACCEPT now_kf=X match_kf=Y bow=Z pnp_inl=N`. For each accept, check:
- Was `match_kf` a real revisit? (compare `t_cam_world` of the two keyframes)
- Was `bow_score` high enough? Below ~0.01 with weak adaptive minScore = noise.
- Was `pnp_inl` healthy? Below 20 = geometry barely supported.

**Step B5 — Propose mitigation at the BoW retrieval / PnP layer.**
NEVER tune chi² to admit thrashing — that just makes it worse. Tighten retrieval gates (kPnpMinInliers, adaptive minScore floor) or improve geometric verification.

### Playbook C — Validate a fix offline before flashing

**Step C1 — Identify the baseline sim.**
Pick a checked-in fixture under `tests/sims/regression/visual/` that exhibits the bug (or pull a fresh walk that does).

**Step C2 — Build the desktop replay harness.**
```bash
cd tests/cpp
cmake --build build --target replay_harness --config Release
```

**Step C3 — Run on the baseline.**
```bash
./build/replay_harness ../sims/regression/visual/<FIXTURE>.json \
  build/replay_out/<FIXTURE>_baseline.csv \
  --frames-dir ../sims/regression/visual/<FIXTURE>.frames \
  --frame-match-tolerance-ms 50
```

**Step C4 — Capture baseline metrics.**
```bash
python replay_scorer.py build/replay_out/<FIXTURE>_baseline.csv \
  --json-out build/replay_out/<FIXTURE>_baseline.metrics.json
```

**Step C5 — Apply the proposed fix.**
Edit code. No constant tweaks without a derivation comment in the code.

**Step C6 — Rebuild and re-run.**
Same harness, different output filename suffix (`_fixed.csv`, `_fixed.metrics.json`).

**Step C7 — Diff metrics.**
- `loop_closure_corrections_applied`: should rise (fix unblocked corrections)
- `loop_closure_chi2_rejected`: should fall (fewer absurd corrections)
- `drift_per_meter`: should drop or stay same
- `loop_closure_gap_m`: should drop or stay same
- `heading_rmse_rad`: should drop or stay same

**Step C8 — Write up the diff.**
Report includes: hypothesis, fix description, before/after metrics table, residual analysis if any. NEVER ship a "feels better" fix.

### Playbook D — Recorder drop ratio investigation

**Step D1 — Compute drop ratio.**
`event_summary.frames_meta.dropped / (frames_meta.dropped + frames_meta.written)`. Above 50% means encoder is saturated.

**Step D2 — Identify bottleneck.**
PNG encode at 640×480 is ~30 ms on this device. At 30 Hz capture, a single thread saturates a core.

**Step D3 — Pick a mitigation (REQUIRES ADR if shipped).**
- Capture every Nth frame at the source (`SimulationFrameRecorder.captureFrame` decimation). Cite the Nyquist bound for the visual signal of interest.
- Downsample to 320×240 before encoding. Cite resolution requirements for downstream replay accuracy.
- Switch to JPEG (lossy but ~3× faster). Document quality loss.

**Step D4 — Validate.**
Walk the same route. Drop ratio should fall below 5%. Replay harness should still load the resulting frames cleanly (unchanged metrics, modulo any resolution-induced shifts).

## Project guardrails (enforced — apply before proposing any fix)

### No magic numbers

Before changing any threshold, the fix must:
1. Cite the source — chi² table, sensor noise model, calibration RMS, measured statistic
2. Show the data that motivated the change
3. Validate against the existing fixture(s) — drift not getting worse on previously-passing walks

The Stage 1 gravity-alignment fix is the canonical example: gate band `g ± 0.8 m/s²` derived from `3σ_acc + walking-band`, with σ_acc = 0.1 m/s² cited from `EKFState.cpp:81`.

### No shortcuts, no TODOs

If a step in a playbook can't be completed in the current shot:
1. STOP at the step.
2. Tell the user what's blocking and exactly which step is incomplete.
3. Do NOT leave `// TODO`, `FIXME`, `XXX`, or stubs.
4. Do NOT skip to a later step and "come back to it."

### Comment, don't delete

Old analysis scripts, dead replay-harness branches, stale debug paths all stay commented in-tree with a `LEGACY:` marker. Same rule as the rest of the project.

### Read the data before tuning constants

This is the most important rule, learned the hard way:
- **NEVER** propose a constant change without reading the actual residual values from a real walk first.
- The chi² gate at 22.5 is correct. The σ formulas are derived. If they're rejecting your corrections, the bug is upstream.
- `scripts/analyze_chi2_rejections.py` exists specifically because the team forgot this rule.

## Gotchas

- **Logcat ring buffer rolls off in minutes.** Pull immediately after the walk or bump the buffer with `adb logcat -G 16M`.
- **Git Bash mangles `/sdcard/...` paths.** Use PowerShell or escape the leading slash.
- **`vio.x/y/z` in the JSON is Y-up** (post-JNI swap). The internal `output.t` is Z-up. Don't confuse them.
- **EventCounters increments are monotonic** within a session but `nativeResetEventCounters()` zeros them. The sim recorder calls reset at REC start, so counts are walk-scoped.
- **`frames_meta.dropped` includes pre-recorder-start frames** — the queue counter starts at recorder construction. A high drop count on a quick recording can be benign.
- **Frame timestamps in the recorder use wall-clock-ns**, while the JSON's `ts` field uses wall-clock-ms × 1e6. They're CLOSE but not exact (frame stamp at camera receive, JSON stamp at VIO update). Harness fuzzy match (default ±50 ms via `--frame-match-tolerance-ms`) absorbs this.

## References

- Diagnostics: `scripts/analyze_chi2_rejections.py`
- Harness: `tests/cpp/replay_harness.cpp`, `tests/cpp/replay_scorer.py`, `tests/cpp/CMakeLists.txt`
- CI: `.github/workflows/replay.yml`
- Spec: `docs/VISUAL_PLAN_STEP_7_1_GEOMETRIC_LOOP.md`, `docs/adr/ADR-007-replay-harness-imu-only.md`, `docs/adr/ADR-014-visual-replay-harness-recorded-frames.md` (Step 9 ADR)
- Studies: `docs/study/04_updaters_scale.md`, `docs/study/05_vio_engine_jni.md` (EventCounters inventory)
- KNOWN_ISSUES: `docs/KNOWN_ISSUES.md`
