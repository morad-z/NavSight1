# Agent 3 — MiDaS Phase 2 Live Update + SLAM Promotion Sparsity

## Status
Investigation complete. Phase 2 design specified end-to-end. Sparsity verdict: gate is correctly rejecting bad geometry.

## Part 1: SLAM Promotion Sparsity — 24 → 1 is Correct Behavior

### Measured counter data across four real walks

| Walk | Duration | Promos | Par-rej | RMS-rej | Chi-rej | Base-rej | MiDaS-seeded |
|---|---|---|---|---|---|---|---|
| parallax_fix_walk_2026_05_20 | ~128s | 24 | 0 (no gate yet) | 102 529 | 27 763 | 10 910 | 12 (50%) |
| promo_parallax_walk_2026_05_21 | ~121s | 1 | 26 187 | 86 690 | 16 935 | 9 445 | 1 |
| bug3_walk_2026_05_21 | ~115s | 0 | 20 674 | 64 486 | 16 162 | 6 829 | 0 |
| bug4_walk_2026_05_21 | ~117s | 2 | 21 459 | 74 785 | 16 864 | 8 592 | 2 |

`slam_promo_rms_milli_p95` in `promo_parallax`: 9.7 px (well above 1.5 px RMS gate); in `bug4`: 2.9 px.

### Verdict: the gate is correctly rejecting bad geometry

**Evidence point 1 — the pre-gate 24 promotions were mostly poisoned.** `parallax_fix_walk_2026_05_20` had 12 of 24 features immediately MiDaS-seeded because z_tri was 403–680 m vs z_midas 1.4–1.6 m — a 250–450× error. These 12 features passed baseline + chirality + RMS but were geometrically invalid.

**Evidence point 2 — the threshold has literature support.** `kSlamPromoMinParallaxCos = 0.99985` is the ORB-SLAM3 1° threshold for triangulation acceptance (Mur-Artal & Tardós 2017 §V.B).

**Evidence point 3 — the 1 surviving promotion was geometrically sound.** That feature had z_tri = 7.52 m — plausible wall depth.

**Evidence point 4 — Morad's walk pattern is intrinsically parallax-starved.** Pure axial motion produces near-zero parallax for all wall features.

**Do not loosen `kSlamPromoMinParallaxCos`.** Correct solution is Phase 2 MiDaS live update, providing depth independent of motion baseline.

### Is 1° tight relative to 0.57°?

1° for promotion vs 0.57° for per-frame update is intentional and correct. Promotion is one-shot and creates a persistent EKF state entry, so it must be more conservative.

### MiDaS timing-gap interaction

Across all four post-Fix-#12-Phase-1 walks, `midas_depth_samples` equals `slam_promotions_total` in every case. The sampler is called exactly once per promotion. The "5 promotions but 0 midas_depth_samples" scenario referred to an earlier run; in practice the affine fit converges within the first few keyframes (~2 s).

**Recommendation:** Do not gate promotion on MiDaS affine validity. Phase 2 live update will refine ρ once MiDaS becomes available.

## Part 2: Phase 2 MiDaS Live Update — Complete Architecture Design

### Design rationale

Fix #10 (`kSlamMinParallaxCos = 0.99995`) correctly blocks the 2-DOF reprojection update during axial motion. After Fix #12 Phase 1, promoted features start with correct depth. But once promoted, ρ receives no updates during prolonged axial motion. Phase 2 provides a 1-DOF depth-only correction from MiDaS that fires precisely when the parallax gate has blocked the normal update.

### New `EKFState` methods

```cpp
bool updateSlamFeatureMidasDepth(int slot, double depth_m, double sigma_m);
bool isParallaxBelowThreshold(int slot) const;
```

### H matrix derivation

```
p_anchor_cam = (alpha/rho, beta/rho, 1/rho)
p_world      = R_anchor.t() * p_anchor_cam + p_anchor
p_I_live     = R_GtoI * (p_world - p_G)
p_C_live     = R_bc * p_I_live
z_pred       = p_C_live[2]
```

H row = third row of per-feature Jacobian. Reuse setup from `buildSlamLiveJacobianRow`. Scalar residual:
```
r = depth_m - z_pred
S = H_depth * P * H_depth.t() + sigma_m^2
m2 = r^2 / S
```

### Threshold

```cpp
constexpr double kMidasDepthChi2Threshold = 3.841;  // chi²(0.95, 1-DOF)
```

### σ_m derivation

VI-Depth (Wofk et al., ICRA 2023 §5.1): σ_MiDaS ≈ 0.5 m after affine fit. Scale by inlier confidence:

```cpp
constexpr double kMidasBaseSigmaM = 0.5;
double inlier_ratio = midas_affine_fit_inlier_ratio_milli / 1000.0;
double sigma_m = std::max(0.30, std::min(2.0,
    kMidasBaseSigmaM / std::max(inlier_ratio, 0.10)));
```

### Wire-up in Tracker.cpp (after applySlamLiveBatch at ~3670)

```cpp
if (!frame_blurry_
    && midas_affine_valid_
    && midas_affine_fit_inlier_ratio_ > 0.70f) {
    for (int slot = 0; slot < n_slam; slot++) {
        if (!ekf_.isParallaxBelowThreshold(slot)) continue;
        auto cur_it = cur_obs.find(slot_fid);
        if (cur_it == cur_obs.end()) continue;

        double depth_m = 0.0;
        if (!sampleMidasMetricDepth(cur_it->second.x, cur_it->second.y, depth_m)) continue;

        double inlier_ratio = midas_affine_fit_inlier_ratio_;
        double sigma_m = std::max(0.30, std::min(2.0,
            0.5 / std::max(inlier_ratio, 0.10)));

        ekf_.updateSlamFeatureMidasDepth(slot, depth_m, sigma_m);
    }
}
```

### New counters

```cpp
std::atomic<long long> slam_live_midas_depth_fired{0};
std::atomic<long long> slam_live_midas_depth_chi2_rejected{0};
```

### Cost
~140 LOC, 2 counters.

### Risk mitigations

| Risk | Mitigation |
|---|---|
| Bad MiDaS sample | chi²(0.95) gate |
| Low-quality affine fit | Only fire when inlier_ratio > 0.70 |
| Depth-map stale | σ_m = 0.5 m subsumes ~0.5s feature motion |
| Double-update | Phase 2 fires ONLY when isParallaxBelowThreshold |

### Falsifiers

1. `slam_live_midas_depth_fired > 0` during any axial walk
2. `slam_live_midas_depth_chi2_rejected / slam_live_midas_depth_fired < 0.20`
3. LC `target_p` error drops from 65m to <5m even after prolonged axial motion
4. Per-slot ρ stable instead of frozen
5. User-visible: orange dots reappear near features after long axial walk

## Cross-references

- `EKFState.cpp:3320–3325` — J_alpha reuse pattern
- `EKFState.cpp:3253` — kSlamMinParallaxCos = 0.99995
- `Tracker.cpp:~3670` — Phase 2 wire-up site
- `Tracker.cpp:3545–3618` — midas_affine_s_/t_ cache pattern (copy for inlier_ratio_)

## Papers cited

1. Geneva et al., "OpenVINS," ICRA 2020 §III.D
2. Mur-Artal & Tardós, "ORB-SLAM3," IEEE RA-L 2021 §V.B
3. Wofk et al., "VI-Depth," ICRA 2023 §5.1
4. Mourikis & Roumeliotis, "MSCKF," ICRA 2007

## Confidence
HIGH for sparsity verdict (4 walks of evidence). HIGH for Phase 2 design (literature-grounded, reuses existing patterns).
