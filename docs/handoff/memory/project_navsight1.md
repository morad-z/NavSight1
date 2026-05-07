---
name: NavSight1 VIO project
description: Android Visual-Inertial Odometry app on branch morad working through docs/VISUAL_PRODUCTION_PLAN.md
type: project
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
NavSight1 is an Android VIO navigation app at C:\Users\morad\AndroidStudioProjects\NavSight1, currently on branch `morad`. Active development driver is `docs/VISUAL_PRODUCTION_PLAN.md` (Steps 1–11). Steps 1–8 code is committed at 221b22b; Steps 7 and 8 still pending acceptance via daytime sim validation. Companion plan `PRODUCTION_READINESS_PLAN.md` covers the inertial half.

**Why:** Visual layer was disabled in many places (MSCKF, Mapper, LoopClosureDetector); the plan re-enables them in production-quality form to bound long-session drift.

**How to apply:** All work happens through plan steps with strict acceptance gates. Sims live in `tests/sims/` (recent) and `tests/sims/regression/` (frozen baselines). Analyzer is `scripts/analyze_sim.py`.
