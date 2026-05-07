# Continuation prompt for a fresh Claude Code session on the PC

Paste the block below verbatim into a fresh Claude Code session on your
PC. It is self-contained — it tells Claude to read the project state
documents and pick up where the laptop session left off.

---

```
You are continuing work on the NavSight1 Android VIO app on branch
`morad`. The previous session was on a different machine (laptop, session
ID 79610daf-47c4-4b65-8f87-6ada8b7fecc8, dated 2026-05-07). You have no
prior context — read this brief, then read the on-disk state documents
before doing anything else.

REQUIRED READS (in this order, before any action):

1. C:\Users\morad\.claude\projects\C--Users-morad-AndroidStudioProjects-NavSight1\memory\MEMORY.md
   — index of memory entries from the prior session. Read every entry
   it points at. Adjust the path if your Windows username or the
   project path is different.

2. docs/IMPLEMENTATION_STATUS.md — full snapshot of what's shipped vs
   pending in both production-readiness and visual plans, with today's
   uncommitted heading-bug fix detailed.

3. docs/VISUAL_PRODUCTION_PLAN.md — the active step-by-step plan
   driving development.

4. docs/MAP_MATCHING_PLAN.md — sibling plan for after the visual plan
   completes; recently reworked for VIO-input (no GPS in the matcher).
   Status: draft, dependencies on visual plan + heading fix validation.

5. docs/adr/ — ADRs 001–013 + ADR-017 (which is WITHDRAWN; do not
   revive). Read at least the index.

PLAN RULES — enforce strictly, no exceptions:
- A step is NOT shipped until acceptance criteria are verified with
  real sim data. Code written ≠ step closed.
- Check every step's "Do not start without" prerequisites before
  beginning work. Create missing prerequisites first.
- No magic numbers — every threshold derived and documented with a
  source citation.
- Every step needs a sim saved to tests/sims/regression/ before the
  next step starts (Principle 4: replay before re-flash).
- Do not disable or gate-flag broken code; fix the root cause.
- NEVER fuse GPS into the EKF runtime. NavSight is VIO-only by design
  (ADR-004 + the "no GPS in the EKF" feedback memory). GPS at startup
  is allowed (world anchor + scale seed) but no per-frame fusion.
- Do not commit unless I explicitly say "commit" in that message.

CURRENT STATE — pick up here:

The laptop session diagnosed and fixed an EKF visual yaw sign bug that
was producing 75° heading drift on real walks. Three lines changed in
app/src/main/cpp/EKFState.cpp (lines 1052, 1097, 1217). Plus added MiDaS
event_summary counters and a per-DOF chi² breakdown in the LC_ABS log.
Plus rewrote MAP_MATCHING_PLAN.md for VIO-input. Plus withdrew ADR-017.
See docs/IMPLEMENTATION_STATUS.md §"Today's work" for the full list.

If `git status` shows the heading fix and counter edits as uncommitted,
the laptop session may not have committed before handoff. Do NOT commit
without my say-so; just verify the changes look intact and report.

The next concrete action waiting on me (Morad) is:
1. Build the app with the EKFState changes + MiDaS counters.
2. Walk a daytime out-and-back sim like 1778147132092.
3. Run scripts/analyze_sim.py + scripts/compare_gps_vio.py +
   scripts/heading_audit.py on the new sim. Use
   $env:PYTHONIOENCODING="utf-8" in PowerShell.
4. Acceptance: |vyaw − gps_course| at end < 15° (was −112°), GPS↔VIO
   endpoint divergence < 20 m (was 104 m), MSCKF huber rate toward
   ~0.05 (was 1.66/update), loop_closure_corrections_applied > 0.

After that sim is analyzed, the deferred work list to address (in
order): Step 7/8 acceptance, Step 9 (replay harness CI), Step 10
(scooter), MSCKF pixel_noise units fix (task #8), magnetometer-
revisit ADR.

AGENT USAGE: For non-trivial tasks, spawn parallel agents in a single
message. Roles: cpp-reviewer + coder for C++ implementation, tdd-guide
for tests, code-analyzer / python-reviewer for sim analysis, architect
for architecture decisions, security-reviewer + code-reviewer for
post-edit review. Never sequential when independent.

FIRST RESPONSE: confirm you've read MEMORY.md and IMPLEMENTATION_STATUS.md
and summarize in 2-3 sentences what state we left things in and what
you understand the next action to be. Don't take any actions until I
respond.
```

---

## Notes on what this prompt does and doesn't preserve

**Preserves (via files in the repo + memory):**
- The full plan rules, no-GPS-in-EKF principle, agent-usage policy.
- The verdict on sim 1778147132092 (75° drift, 70 m endpoint error,
  loop closure 0 corrections, etc.).
- Today's three EKF code changes with their rationale.
- The withdrawn-ADR-017 history.
- The map-matching plan rework status.
- The deferred task list with priorities.

**Doesn't preserve (would require manual context if the new session
needs it):**
- Why specific agent decisions were made (e.g. why we chose to
  measure m2_R/m2_p block-wise rather than per-axis) — captured in
  code comments though.
- The exact step-by-step debugging dialogue that led to the heading
  fix — captured at the conclusion-and-evidence level in
  `memory/project_heading_drift_root_cause.md` and
  `memory/project_heading_fix_2026_05_07.md`.

If the new session asks "why did we do X?" and the memory + status
doc don't answer it, the laptop transcript at
`docs/handoff/session-79610daf.jsonl` (4.7 MB JSONL) has the full
detail and can be searched / read on demand.
