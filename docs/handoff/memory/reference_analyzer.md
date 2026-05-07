---
name: Sim analyzer script
description: scripts/analyze_sim.py — single-sim analysis and Step-8 baseline comparison; needs PYTHONIOENCODING=utf-8 on Windows
type: reference
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
`scripts/analyze_sim.py` is the canonical sim analyzer.

**Single-sim analysis:**
```
python scripts/analyze_sim.py <sim>.json
```

**Step-8 acceptance comparison:**
```
python scripts/analyze_sim.py --compare-baseline tests/sims/regression/baseline_walk_001.json <new>.json
```
Prints PASS/FAIL/REVIEW verdicts on:
- Criterion 1: heading_gap_deg ≥10% improvement
- Criterion 2: |TD_offset_ms − 2.0| ≤ 5.0 ms (warmup baseline)

**Windows note:** The script uses Unicode (→, °, ↓). Run from PowerShell with `$env:PYTHONIOENCODING="utf-8"` or it errors on charmap and truncates output mid-stream.

Pre-Step-8 frozen baseline: `tests/sims/regression/baseline_walk_001.json` (committed in e5c53fc; small 323KB walk; loop heading gap +6.64°, position gap 1.79m, no TD/extr counters).
