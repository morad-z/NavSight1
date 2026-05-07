---
name: Visual plan rules — strictly enforced
description: Acceptance gating, sim-before-next-step, no magic numbers, fix root cause not gate-flag
type: feedback
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
VISUAL_PRODUCTION_PLAN.md rules — enforce strictly, no exceptions:
1. A step is NOT shipped until acceptance criteria verified with real sim data. Code written ≠ step closed.
2. Check every step's "Do not start without" prerequisites before beginning. If missing, create first.
3. No magic numbers — every threshold derived and documented with a comment citing source (chi² table, pixel-noise model, calibration RMS, measured sim statistic).
4. Every step needs a sim saved to `tests/sims/regression/` before the next step starts (Principle 4: replay before re-flash).
5. Do not disable or gate-flag broken code; fix the root cause.

**Why:** These mirror the plan's "Guiding principles" (lines 38–91). Violations are how the visual stack got into its current half-working state.

**How to apply:** Every code change verifies by sim. Every threshold gets a comment-cited source. When something fails, find why; don't add a flag to skip it.
