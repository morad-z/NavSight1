#ifndef NAVSIGHT_EVENT_COUNTERS_H
#define NAVSIGHT_EVENT_COUNTERS_H

// EventCounters — process-wide atomic counters for VIO pipeline events.
//
// PURPOSE
//   The simulator app records simulation_data_<ts>.json on phone storage.
//   When Morad walks untethered, an adb-logcat companion stream is
//   impractical: Samsung scoped storage / wrong-tag filtering / buffer
//   roll all conspire to drop or shred the trace. Embedding per-event
//   counts directly into the same JSON the simulator already writes
//   removes the second pipe entirely.
//
// DESIGN
//   * Flat struct of std::atomic<long long> — one process-singleton.
//   * Increment alongside every existing LOGI site we care about. The
//     LOGI lines stay (live debug still uses them); we just add an atomic
//     fetch_add(1) next to each.
//   * No mutex anywhere. fetch_add(_, std::memory_order_relaxed) is the
//     hot path; the only read site is JNI getEventCountersJson(), which
//     also runs lock-free (snapshot reads with relaxed ordering).
//   * Roll our own JSON formatter — flat key:value, ~20 fields, trivial.
//
// SCHEMA (the JSON object embedded into simulation_data_<ts>.json under
// the top-level key "event_summary"):
//
//   {
//     "reloc_orb_accepts": N,
//     "reloc_orb_rejects": N,
//     "reloc_orb_size_skipped": N,
//     "reloc_orb_slam_guarded": N,
//     "blur_enter_events": N,
//     "blur_total_skip_frames": N,
//     "lowlight_state_active_frames": N,
//     "lowlight_log_lines": N,
//     "rot_gate_pure_rot_confirmed": N,
//     "rot_gate_log_lines": N,
//     "klt_adaptive_window_hits": N,
//     "ba_solves_total": N,
//     "ba_solves_accepted": N,
//     "ba_solves_rejected": N,
//     "ba_skipped_in_flight": N,
//     "ba_solve_us_sum": N,
//     "ba_solve_us_max": N,
//     "ba_iters_sum": N,
//     "msckf_update_lines": N,
//     "msckf_huber_rejected_sum": N,
//     "loop_closure_attempts": N,
//     "loop_closure_accepts": N,
//     "loop_closure_rejects_low_score": N,
//     "loop_closure_rejects_pnp": N,
//     "loop_closure_kf_count_in_db": N,
//     "loop_closure_chi2_rejected": N,
//     "loop_closure_corrections_applied": N,
//     "extrinsics_rotation_angle_mdeg": N,
//     "rolling_shutter_skew_ns": N,
//     "midas_entries": N,
//     "midas_bailout_no_depth": N,
//     "midas_bailout_few_pts3d": N,
//     "midas_bailout_invalid_intrinsics": N,
//     "midas_bailout_camera_h": N,
//     "midas_bailout_low_g": N,
//     "midas_bailout_few_floor_matches": N,
//     "midas_rejected_extreme": N,
//     "midas_fused": N,
//     "midas_skipped": N,
//     "midas_affine_fit_singular": N,
//     "midas_affine_fit_low_inliers": N,
//     "midas_affine_fit_inlier_ratio_milli": N,   // gauge: latest inlier ratio × 1000
//     "midas_affine_fit_shift_milli": N           // gauge: latest fit shift t × 1000
//   }

#include <atomic>
#include <cstdint>
#include <string>

namespace navsight {

struct EventCounters {
    // Step 4 — RELOC_ORB (Tracker.cpp tryRelocalizeWithORB)
    std::atomic<long long> reloc_orb_accepts{0};
    std::atomic<long long> reloc_orb_rejects{0};
    std::atomic<long long> reloc_orb_size_skipped{0};
    std::atomic<long long> reloc_orb_slam_guarded{0};

    // Step 5 — BLUR (Tracker.cpp)
    std::atomic<long long> blur_enter_events{0};
    std::atomic<long long> blur_total_skip_frames{0};

    // Step 5 — LOWLIGHT (FeatureManager.cpp replenishSparse)
    std::atomic<long long> lowlight_state_active_frames{0};
    std::atomic<long long> lowlight_log_lines{0};

    // Step 5 — ROT_GATE (Tracker.cpp Rayleigh dual-gate)
    std::atomic<long long> rot_gate_pure_rot_confirmed{0};
    std::atomic<long long> rot_gate_log_lines{0};

    // Step 5 — KLT adaptive window (Tracker.cpp)
    std::atomic<long long> klt_adaptive_window_hits{0};

    // Step 6 — BA (Tracker.cpp kickOffBARound + worker)
    std::atomic<long long> ba_solves_total{0};
    std::atomic<long long> ba_solves_accepted{0};
    std::atomic<long long> ba_solves_rejected{0};
    std::atomic<long long> ba_skipped_in_flight{0};
    std::atomic<long long> ba_solve_us_sum{0};
    std::atomic<long long> ba_solve_us_max{0};
    std::atomic<long long> ba_iters_sum{0};
    // Step 6 diagnostic counters — each kickOffBARound early-return path.
    // Added 2026-05-04 after a 100 m walk showed ba_solves_total=0 with no
    // way to tell which gate had failed. Now next sim's event_summary
    // shows exactly where BA was bottlenecked.
    std::atomic<long long> ba_skipped_no_init{0};
    std::atomic<long long> ba_skipped_too_few_clones{0};
    std::atomic<long long> ba_skipped_no_intrinsics{0};
    std::atomic<long long> ba_skipped_too_few_landmarks{0};

    // Step 3b SLAM promotion total. Increment every time addSlamFeature
    // returns a valid slot. If this stays 0 across a long walk we know
    // no feature ever passed the SLAM-promotion gate — the BA root cause.
    std::atomic<long long> slam_promotions_total{0};

    // Step 9 / ADR-014 — SLAM feature lifetime histogram (sums + count).
    // Accumulated by FeatureManager::setSlamSlot when a feature transitions
    // out of an EKF slot (slam_slot >= 0 -> slam_slot < 0) AND inside
    // dropLifecycle when a still-promoted feature is reclaimed without an
    // explicit demotion. Lifetime is measured in observations (the
    // FeatureLifecycle::age delta between promotion and exit), which is a
    // close proxy for frames on a steady KLT track. Replay scorer derives
    // mean lifetime as obs_sum / max(count, 1); the plan asked for p50,
    // which would need a histogram bucket — folded into a future ADR.
    std::atomic<long long> slam_lifetime_obs_sum{0};
    std::atomic<long long> slam_lifetime_count{0};

    // Phase 1 Step 4 (post_v19_sprint_plan.md) — SLAM feature re-anchoring.
    // Before this fix, marginalizeOldestCloneNoLock would call removeSlamFeature
    // on every SLAM feature anchored at the dropping clone — i.e. ALL of them
    // within ~2 s at MAX_CLONES=11/5Hz keyframes. The "flashing dots" bug.
    //   slam_reanchor_total          — successful re-anchors (the win)
    //   slam_reanchor_failed_no_clone — no surviving clone after pop (window<2)
    //   slam_reanchor_failed_geometry — new Zc <= 0.01 (degenerate, point behind)
    // Healthy walk: total / (total + failures) > 0.5 ; slam_lifetime_obs_sum
    // grows because features now survive clone churn.
    std::atomic<long long> slam_reanchor_total{0};
    std::atomic<long long> slam_reanchor_failed_no_clone{0};
    std::atomic<long long> slam_reanchor_failed_geometry{0};

    // Step 3a/3b — MSCKF (EKFState.cpp applyMSCKFUpdate)
    std::atomic<long long> msckf_update_lines{0};
    std::atomic<long long> msckf_huber_rejected_sum{0};

    // ── Silent-failure observability — 2026-05-16 audit (docs/review_2026_05_16/silent_failures.md)
    // Each counter converts a previously-invisible failure path into a JSON event_summary signal.

    // EKFState.cpp applyMSCKFUpdate: both Cholesky AND SVD inversion fail on S —
    // catches numerical instability (NaN/Inf in S) silently dropping the full MSCKF update.
    std::atomic<long long> applyMSCKFUpdate_inversion_failed{0};

    // WindowedBA.cpp: landmark Hessian det < 1e-30 (singular) —
    // catches BA Schur computation continuing with garbage-quality pseudo-inverse.
    std::atomic<long long> ba_singular_landmarks{0};

    // EKFState.cpp propagateIMU: early-return without propagating —
    // catches timestamp wraparound / JNI unit mismatch / uninitialized state silently freezing EKF.
    std::atomic<long long> propagate_skipped{0};
    std::atomic<long long> propagate_skipped_not_init{0};       // full_initialized_ false
    std::atomic<long long> propagate_skipped_dt_nonpositive{0}; // dt <= 0
    std::atomic<long long> propagate_skipped_p_empty{0};        // P_.empty()

    // UpdaterMSCKF.cpp: per-feature chi² gate reject —
    // catches high rejection rates distinguishing "no features" from "all chi²-rejected".
    std::atomic<long long> msckf_chi2_rejected{0};

    // UpdaterMSCKF.cpp: per-feature Cholesky inversion fails —
    // catches NaN/Inf in feature covariance as first symptom of numerical divergence.
    std::atomic<long long> msckf_inversion_failed{0};

    // EKFState.cpp propagateIMU: velocity clamped at 5 m/s —
    // catches preintegration bugs driving velocity to physically impossible values (50+ m/s).
    std::atomic<long long> velocity_clamped{0};

    // EKFState.cpp updateScale: chi² gate reject —
    // catches MiDaS depth scale observations being silently discarded.
    std::atomic<long long> scale_chi2_rejected{0};

    // EKFState.cpp applyMSCKFUpdate: camera-IMU time offset clamped to ±100 ms —
    // catches persistent bias saturation of the physical bound.
    std::atomic<long long> time_offset_clamped{0};

    // UpdaterMSCKF.cpp: getCloneFEJ returns false (clone evicted before Jacobian build) —
    // catches stale clone refs leaving zero Jacobian rows and corrupting chi² gates.
    std::atomic<long long> fej_clone_miss{0};

    // Tracker.cpp: updateGravityAlignment returned false —
    // catches gravity-alignment rejections (EKF not init, accel band, factorisation fail).
    std::atomic<long long> gravity_alignment_rejected{0};

    // Tracker.cpp: updateRelativeRotation returned false —
    // catches relative-rotation rejections (clone missing, EKF not init, malformed R).
    std::atomic<long long> relative_rotation_rejected{0};

    // LoopClosureDetector.cpp: heading/PnP rejects across ALL candidates (not just ci==0) —
    // catches the majority of rejection events that were invisible when ci>0 was excluded.
    std::atomic<long long> loop_closure_rejects_heading_total{0};
    std::atomic<long long> loop_closure_rejects_pnp_total{0};

    // Step 7 — Loop closure (LoopClosureDetector.cpp, ADR-013)
    //   loop_closure_attempts             every tryDetectLoop() entry
    //   loop_closure_accepts              tryDetectLoop returned true
    //   loop_closure_rejects_low_score    BoW top score < threshold
    //   loop_closure_rejects_pnp          PnP failed / too few inliers
    //   loop_closure_kf_count_in_db       size of the BoW database
    //                                     (rewritten on every addKeyframe)
    std::atomic<long long> loop_closure_attempts{0};
    std::atomic<long long> loop_closure_accepts{0};
    std::atomic<long long> loop_closure_rejects_low_score{0};
    std::atomic<long long> loop_closure_rejects_pnp{0};
    // Candidate heading differed from query heading by > 90°: same-session
    // Step 7 only handles same-direction revisits; opposite-direction
    // approaches produce geometrically inconsistent 3D-2D pairs for PnP.
    std::atomic<long long> loop_closure_rejects_heading{0};
    // 2026-05-21 Bug 4 — chi² gate on visual rotation measurement updates.
    //
    // Cause: even after the front-end parallax gate (Tracker.cpp:2162) and
    // the SLAM promotion parallax gate (Tracker.cpp:~3372), EKF R_GtoI
    // yaw still jumps 30-65°/s occasionally because some frames have
    // adequate parallax (> 0.01 rad) but still produce noisy R_vo from
    // recoverPose (motion blur during turn, low-texture wall stretch,
    // KLT outliers, etc.). With var_yaw_floor = (0.57°)² (line 3895 in
    // Tracker.cpp) and typical EKF P_yaw ≈ (1°)², the Kalman gain
    // K = P / (P + R) ≈ 0.99 — a single bad measurement gets applied at
    // ~99% strength, producing 30-65° R_GtoI jumps.
    //
    // These residual jumps mean the EKF R_GtoI yaw drifts ~10-15° between
    // LC accepts even on the post-parallax-fix walks. Because trajectory
    // `global_t_` is integrated via `R_GtoI.t() * t_scaled` per-frame,
    // the trajectory shape rotates by the same amount → user sees the
    // map showing a different orientation each loop pass.
    //
    // Measurement evidence: bug3_walk_2026_05_21.json showed loop-1 peak
    // bearing -47.4° vs loop-2 peak bearing -62.8°, a 15° per-loop
    // rotation despite Madgwick yaw drifting only 0.3° over the same
    // span — confirming the rotation comes from EKF R_GtoI, not the
    // gyro reference.
    //
    // Change: add per-measurement chi² gate on both visual rotation
    // update paths.
    //   * updateGravityAlignedYaw (1-DOF):  chi²(0.999, 1) = 10.83
    //   * updateRelativeRotation  (3-DOF):  chi²(0.999, 3) = 16.27
    // Mahalanobis = r^T · (H P H^T + R)^-1 · r. Above threshold, reject
    // the measurement silently and increment the corresponding counter.
    //
    // Threshold rationale: chi²(0.999) is the Engineering-standard
    // "obvious outlier" cutoff used everywhere else in NavSight (MSCKF
    // per-row Huber, LC absolute-pose gate). For a measurement that
    // legitimately differs from EKF prediction by <= 3.3 σ_total, the
    // gate passes (admits the honest correction). For the 30-65° jumps
    // we observe, m² is in the 100-300 range — well above 10.83/16.27.
    //
    // Falsifier: post-fix walk →
    //   visual_yaw_chi2_rejected_total            > 0
    //   visual_relative_rotation_chi2_rejected_total > 0
    //   `OVERLAY_SNAPSHOT R_GtoI_yaw_deg` >= 30°/s jumps → 0
    //   Trajectory bearing across loops within ±5°
    //   `hunt_ekf_yaw_jumps.py` EKF p99 → < 5°/s (currently 50-65°/s)
    // 2026-05-21 — chi² gate counters kept for back-compat after Bug 4 revert.
    // Both will stay at 0 because the gates are now disabled (see LEGACY: blocks
    // in EKFState.cpp). Their replacement counters are below.
    std::atomic<long long> visual_yaw_chi2_rejected_total{0};
    std::atomic<long long> visual_relative_rotation_chi2_rejected_total{0};

    // 2026-05-21 — gyro-vs-visual consistency rejection (root-cause Bug 4 fix).
    // Increments when the per-frame R_vo from recoverPose disagrees with the
    // gyro-integrated rotation by more than the physical-bound threshold (2°
    // for frame-to-frame). The threshold is grounded in measured Madgwick
    // yaw-rate p99 = 1.87°/s × frame interval (33ms) + safety margin, NOT in
    // statistical cuts that depend on EKF state covariance. recoverPose
    // occasionally returns sign-flipped or otherwise wildly wrong R on
    // planar / low-parallax scenes even with high inlier counts; this gate
    // catches them at the sensor-disagreement layer before they reach the
    // EKF.
    //
    // Falsifier: post-fix walks → this counter > 0 AND OVERLAY_SNAPSHOT
    // R_GtoI_yaw_deg jumps >= 30°/s drop to 0.
    std::atomic<long long> visual_relative_rotation_gyro_mismatch_total{0};

    // 2026-05-21 Bug 5 — visual yaw → Madgwick direct sync.
    // Increments every keyframe where the visual yaw estimate (from KLT
    // dot rotation since the last keyframe) was used to nudge Madgwick
    // toward the visual world reference. Combined with Tracker.cpp:3816
    // gyro-consistency gate (which makes the visual yaw trustworthy),
    // this bounds Madgwick gyro bias drift between LC accepts.
    //
    // Health: count / keyframes-per-walk should be ~50-100%. If much
    // lower, the upstream gyro-consistency gate is rejecting too many
    // visual measurements (look at KF_HEADING_CORR drift histogram).
    std::atomic<long long> madgwick_visual_yaw_nudges_total{0};

    // 2026-05-21 Bug 3 — post-PnP rotation-residual sanity gate.
    // Increments when an LC accept's PnP-derived target_R is > π/2 (90°)
    // away from current EKF R_GtoI. Counts each ramp-frame that hits the
    // gate, so a single bad match contributes up to 10 (matching the damp-
    // frame cadence). Distinct from loop_closure_rejects_heading (which is
    // at BoW retrieval time, gates Madgwick yaw delta between candidate
    // and current keyframes) and from loop_closure_chi2_rejected (which is
    // the EKF's own chi² inside updateAbsolutePose). The PnP sanity gate
    // sits BETWEEN those two: PnP produced a plausible inlier count + low
    // BoW score, but the resulting target_R is geometrically impossible
    // given how little the body actually rotated since the matched
    // keyframe (Madgwick is stable). Catches the essential-matrix π-flip
    // ambiguity that planar scenes induce. Evidence: 2026-05-20 walk
    // 19:03 logged one LC with r_R = [2.821 0.750 0.418] ≈ 169° rotation
    // residual; chi² correctly rejected all 10 damp frames (m²_R ≈ 72
    // vs threshold 22.5) but the system still burned the work. This
    // gate cuts the wasted effort AND defends against borderline cases
    // where the bad PnP target marginally passes chi².
    std::atomic<long long> loop_closure_rejects_rot_sanity_total{0};
    std::atomic<long long> loop_closure_kf_count_in_db{0};
    // Step 7 absolute-pose injection (ADR-013 §"Correction injection —
    // absolute pose path"). Completes accounting from detection to
    // correction: chi²_rejected counts wildly-wrong matches caught by
    // the outer gate; corrections_applied counts every successful
    // updateAbsolutePose injection through the EKF.
    std::atomic<long long> loop_closure_chi2_rejected{0};
    std::atomic<long long> loop_closure_corrections_applied{0};

    // Phase 1 Step 2b verification: counts loop closures that succeeded
    // on a candidate OTHER than results[0] (i.e. the top-1 BoW match
    // failed PnP but a lower-ranked candidate did match). If this counter
    // is > 0 across a walk, the multi-candidate retry is doing real work.
    // If = 0, the top-1 was always sufficient and Step 2b had no effect
    // on this dataset. Counter increments inside LoopClosureDetector
    // (Tracker doesn't see the candidate index).
    std::atomic<long long> loop_closure_lower_rank_accepts{0};

    // Phase 6.4c — Loop-closure spatial pre-filter (LandmarkMap-driven).
    //
    // Before each BoW query, the LC worker asks LandmarkMap for the union
    // of observed_in_kfs across landmarks within kLcSpatialSearchRadiusM
    // of the EKF-predicted camera position, then hands those keyframe ids
    // to tryDetectLoopWithCandidates as a filter on DBoW2's score list.
    //
    //   loop_closure_spatial_candidates_total
    //     SUM of `n_candidates` across every LC worker tick (NOT a count
    //     of ticks). Divide by loop_closure_attempts for the mean
    //     candidate-set size per query — useful for tuning the radius.
    //
    //   loop_closure_spatial_fallback_total
    //     Number of LC worker ticks where the LandmarkMap returned an
    //     empty candidate set and the detector fell back to the full
    //     keyframe DB. Expected to be high at the start of a walk
    //     (LandmarkMap is empty) and approach zero once the map populates.
    std::atomic<long long> loop_closure_spatial_candidates_total{0};
    std::atomic<long long> loop_closure_spatial_fallback_total{0};

    // Phase 1 Step 2c verification: rolling-window camera frame rate
    // statistics, persisted to event_summary so we don't depend on
    // logcat retention to verify the [30, 30] fps lock. Stored in
    // milli-Hz (×1000) because std::atomic<double> isn't part of C++17.
    // Updated by Tracker every kCamFpsWindow=30 frames from a Welford-
    // style running aggregate. Final values written into event_summary
    // by toJson(). Convert to Hz on read by dividing by 1000.0.
    std::atomic<long long> cam_fps_mean_milli_hz{0};
    std::atomic<long long> cam_fps_stdev_milli_hz{0};
    std::atomic<long long> cam_fps_window_count{0};   // # of fps samples

    // Step 7.1 — Geometric loop closure (additive to the BoW path; spec
    // in docs/VISUAL_PLAN_STEP_7_1_GEOMETRIC_LOOP.md). The geometric path
    // does position-based candidate retrieval + projection of pts3d_world
    // into the predicted current camera + KLT-corner NN matching + PnP.
    // Fires only when the BoW path did NOT accept on the same query
    // (fallback semantics), so:
    //   loop_closure_corrections_applied
    //     = (BoW accepts that survived χ²) + (Geom accepts that survived χ²)
    //   loop_closure_corrections_applied - loop_closure_geom_accepts
    //     = pre-Step-7.1 BoW-only baseline
    // Reject reasons are explicit so attribution is clean post-hoc.
    std::atomic<long long> loop_closure_geom_attempts{0};
    std::atomic<long long> loop_closure_geom_accepts{0};
    std::atomic<long long> loop_closure_geom_rejects_no_position{0};
    std::atomic<long long> loop_closure_geom_rejects_no_candidate{0};
    std::atomic<long long> loop_closure_geom_rejects_few_inframe{0};
    std::atomic<long long> loop_closure_geom_rejects_pnp{0};
    // 2026-05-13 Phase 1 Step 7.1 fix: per-correspondence ORB-descriptor
    // verification (Hamming ≤ kGeomDescriptorMaxDistance). Increments once
    // per in-frame projection that found a spatial neighbour in the 15 px
    // gate but failed the descriptor distance check. Fires per-pair, so
    // typical values are >> loop_closure_geom_rejects_few_inframe (each
    // candidate has ~50-150 in-frame projections, each is a vote).
    std::atomic<long long> loop_closure_geom_rejects_descriptor{0};
    // 2026-05-26 — split loop_closure_geom_rejects_descriptor into its two
    // failure modes (the LC_GEOM log computes them per-attempt but logcat rolls
    // off; promote to event_summary for durable post-heading-fix diagnosis).
    // spatial_miss = no current kp within kGeomMatchRadiusPx of the projection
    // (geometry/pose → widen radius); hamming_miss = kp nearby but min-Hamming >
    // kGeomDescriptorMaxDistance (appearance/descriptor-staleness → refresh).
    // hamming_min_{sum,count} → mean min-Hamming-in-radius (~106=random, <50=matchable).
    std::atomic<long long> loop_closure_geom_spatial_miss{0};
    std::atomic<long long> loop_closure_geom_hamming_miss{0};
    std::atomic<long long> loop_closure_geom_hamming_min_sum{0};
    std::atomic<long long> loop_closure_geom_hamming_min_count{0};
    // total_path_m × 10 (integer decimeters) — verifies the dynamic sigma
    // formula (sigma_p = max(2.0, 0.032 × path_m)) is tracking correctly.
    std::atomic<long long> total_path_dm{0};

    // Phase 1 Step 5 (post_v19_sprint_plan.md) — Pose-graph back-end.
    // Retroactive drift correction at loop closure: 4-DOF (x, y, z, yaw)
    // Gauss-Newton optimization over keyframes between K_match and K_now.
    //
    // Counters here exist because the team has been bitten by "the metric
    // looks right but the user-visible behaviour is broken" (see
    // feedback_no_metric_celebration). Each pose-graph invocation publishes
    // pre/post residual norms and applied correction magnitudes so the
    // sim JSON proves the optimizer is actually doing work, AND the
    // back-write is reaching the keyframe DB + global_t_.
    //
    //   pose_graph_optimize_calls         every optimize() entry
    //   pose_graph_iters_used_sum         total Gauss-Newton iterations
    //   pose_graph_residual_norm_pre_mm   sum of pre-optimize ||r|| × 1000
    //   pose_graph_residual_norm_post_mm  sum of post-optimize ||r|| × 1000
    //                                     (ratio post/pre < 0.1 = healthy)
    //   pose_graph_max_correction_mm      max Δposition applied (×1000, mm)
    //   pose_graph_max_correction_mrad    max Δyaw applied (×1000, mrad)
    //   pose_graph_rejected_singular      Cholesky+SVD both failed (counter)
    //   pose_graph_loop_edges_added       total addLoopEdge() calls
    //   pose_graph_apply_calls            successful back-write events
    std::atomic<long long> pose_graph_optimize_calls{0};
    std::atomic<long long> pose_graph_iters_used_sum{0};
    std::atomic<long long> pose_graph_residual_norm_pre_mm{0};
    std::atomic<long long> pose_graph_residual_norm_post_mm{0};
    std::atomic<long long> pose_graph_max_correction_mm{0};
    std::atomic<long long> pose_graph_max_correction_mrad{0};
    std::atomic<long long> pose_graph_rejected_singular{0};
    std::atomic<long long> pose_graph_loop_edges_added{0};
    std::atomic<long long> pose_graph_apply_calls{0};

    // Step 8b — Online IMU-camera extrinsics calibration (EKFState.cpp).
    // extrinsics_rotation_angle_mdeg: angle-from-identity of the current
    // body→camera rotation R_bc_ (||Rodrigues(R_bc)||_2 in milli-degrees).
    // Written every MSCKF update. The initial default diag(1,-1,-1) has
    // angle_from_identity ≈ 180,000 mdeg (180 deg, the axis-angle of that
    // particular rotation). Real deviations during online calibration are
    // O(few mdeg). Use the delta between two consecutive reads to assess
    // convergence speed; a stable value across frames means the EKF has
    // converged.
    std::atomic<long long> extrinsics_rotation_angle_mdeg{0};

    // PHASE_A_8B_REENABLE_2026_05_19 REVERTED — counter retained for future
    // re-attempts. R_bc safety-cap rejection count (currently never fires
    // because R_bc update is disabled at the apply site).
    std::atomic<long long> rbc_runaway_total{0};

    // PHASE_B_6_4_REENABLE_2026_05_19 — safety counter for landmark obs
    // updates. Increments when applyLandmarkObservations would move EKF
    // position by > 1.0m in a single call (precedent: v28 4× runaway).
    // If this climbs > 5 during a normal walk, Phase 6.4 is regressing
    // toward the v28 failure mode; revert by grepping the marker.
    std::atomic<long long> landmark_obs_runaway_total{0};

    // Step 8a — Online IMU-camera time offset δt_d (EKFState row 15).
    // Stored as integer microseconds (round-to-nearest). Warmup prior is ~2 ms
    // (2000 µs); converges toward the true hw offset (typically ±5 ms).
    // Written in the same throttled block as extrinsics_rotation_angle_mdeg.
    std::atomic<long long> cam_imu_time_offset_us{0};

    // Step 8c — Rolling-shutter row read-out time from Camera2.
    // CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW (API level 21+), in
    // nanoseconds (first-row to last-row read-out). Not a counter; stores
    // the last-seen value so the sim JSON shows what the device reported.
    // 0 = global-shutter device or key not supported by the hardware.
    std::atomic<long long> rolling_shutter_skew_ns{0};

    // 2026-05-16 audit Fix C — EKF init deferred because Madgwick is not ready.
    // Incremented each camera frame where the first-frame EKF init block fires
    // but getRotationGtoI() returns empty (Madgwick hasn't settled yet). If this
    // is > 0 in a walk's event_summary it means the bootstrap race window was
    // open and the init was correctly deferred rather than falling back to the
    // old pure-Rz path. Zero on healthy cold-start (Madgwick ready before first
    // camera frame via the SensorRepository seedMadgwickYaw early-seed path).
    std::atomic<long long> ekf_init_deferred_madgwick_not_ready{0};

    // 2026-05-16 — ZRUP (Zero-Rotation-Rate Update) fired. Dual of ZUPT:
    // observes mean(gyro_window) = b_g_ when is_stationary detects stationarity.
    // Without this, EKF b_g_ was frozen during stationary periods (no parallax
    // → no MSCKF update) and gyro integration drifted at the bias rate.
    // User-visible symptom before fix: heading slowly rotating + SLAM dots
    // sliding top-right when phone is held still.
    std::atomic<long long> zrup_fired_total{0};

    // 2026-05-26 — else-branch ZUPT (rotate-in-place / no-translation guard) fired.
    // Replaces the old PDR step-speed translation proxy, which was pedestrian-only
    // and zeroed v_G_ on every moving non-stepping frame (scooter/bike → reported
    // speed could never build). New gate: gyro_norm > 0.8 rad/s AND |v_G_| < 0.5 m/s.
    // Falsifier: ≈ 0 on a real moving ride (cruising); > 0 only on a deliberate
    // stationary spin test. See Tracker.cpp else-branch ZUPT block.
    std::atomic<long long> zupt_rotinplace_fired{0};

    // 2026-05-26 — depth-weighted metric speed estimator (Tracker::updateDepthFlowSpeed).
    //   depth_flow_updates          frames that produced a valid speed estimate
    //   depth_flow_skipped_few_pts  frames skipped: < 6 points with valid MiDaS depth
    //   depth_flow_outlier_rejected frames rejected: per-frame speed > 30 m/s
    std::atomic<long long> depth_flow_updates{0};
    std::atomic<long long> depth_flow_skipped_few_pts{0};
    std::atomic<long long> depth_flow_outlier_rejected{0};
    //   depth_flow_total_mm         Σ per-frame depth-flow displacement (mm) — compare
    //                               to the real distance walked for a direct scale check
    //                               (under-counts by the skipped frames above).
    std::atomic<long long> depth_flow_total_mm{0};
    //   depth_flow_calib_updates    accel-excitation windows (post-ZUPT) that calibrated
    //                               the relative->metric scale K (>0 once it's working)
    std::atomic<long long> depth_flow_calib_updates{0};
    // 2026-05-27 — calibrated MiDaS relative->metric scale K (x1000): last value +
    // min/max over the recording, so K STABILITY is visible in event_summary
    // (logcat rolls off before it can be pulled). A small max/min spread = stable K.
    std::atomic<long long> midas_scale_k_milli{0};
    std::atomic<long long> midas_scale_k_min_milli{0};
    std::atomic<long long> midas_scale_k_max_milli{0};
    // 2026-05-28 — Looming / flow-divergence speed estimator (updateExpansionSpeed).
    //   depth_flow_looming_updates   frames the looming Vz estimate fired (≥10 inliers)
    //   depth_flow_looming_used      frames the FUSED speed was looming-dominant (w_loom>0.5)
    //   depth_flow_looming_skipped   frames skipped: <10 valid points after de-rotation
    std::atomic<long long> depth_flow_looming_updates{0};
    std::atomic<long long> depth_flow_looming_used{0};
    std::atomic<long long> depth_flow_looming_skipped{0};

    // 2026-05-16 — Tracker.cpp:1039 EKF→IMU gyro-bias feedback loop fix.
    // Incremented each time the corrective additive loop fires: EKF's b_g_
    // residual is absorbed into IMU's gyro_bias_, then EKF's b_g_ is reset
    // to zero. See EKFState::setGyroBias for the full cause/change rationale.
    // A healthy walk should show this counter > 0 with the absorbed magnitude
    // converging (each absorption shrinks the residual the next one captures).
    std::atomic<long long> ekf_bg_absorbed_total{0};

    // 2026-05-16 — Keyframe-cadence EKF→IMU gyro-bias push (Agent A finding).
    // Restored after Tier 1 verified setPosition is back. The push at every
    // frame (swarm pattern) oscillates and broke v25-v31; at zero frequency
    // (post-Option-B revert) EKF and IMU bias estimates drift apart, MSCKF
    // chi² rejects 89% of features, SLAM features only promote when stationary.
    // The middle ground: push once per ~6 camera frames (~5 Hz at 30 Hz capture),
    // matching v22's keyframe-cadence feedback. Counter > 0 confirms wire-up;
    // rate ≈ 5/sec confirms throttling.
    std::atomic<long long> gyro_bias_pushed_total{0};

    // 2026-05-16 — visual yaw correction skipped because motion is pure-rotation
    // or translation-degenerate. v33 stationary-rotate walk showed EKF R_GtoI
    // yaw drifting +0.73°/sec while Madgwick drifted -0.55°/sec — the +1.28°/sec
    // divergence came from updateGravityAlignedYaw being fed monocular yaw
    // measurements that are geometrically degenerate under pure rotation. Gating
    // this update channel matches updateRelativeRotation's existing gate
    // (Tracker.cpp:1741) — both visual rotation channels now share the same
    // motion-characteristics guard. Counter > 0 confirms the gate is firing.
    std::atomic<long long> visual_yaw_gated_pure_rotation_total{0};

    // 2026-05-16 — MSCKF processLostFeatures gated on is_static.
    // Per the EKF-update audit (agent finding Q3): processLostFeatures was
    // the root cause of EKF p_G drifting ~40cm during 9.7s stationary in v35.
    // KLT subpixel noise produces small visual residuals → MSCKF applies
    // ~1mm δp corrections × 56 calls/sec → accumulated linear drift.
    // Counter > 0 confirms the gate fires; rate matches msckf_update_lines
    // drop during stationary periods.
    std::atomic<long long> msckf_gated_static_total{0};

    // 2026-05-16 — SLAM updater (Step 5) gated on is_static. Same rationale
    // as msckf_gated_static_total: SLAM reprojection residuals against
    // stationary features carry no signal, only KLT noise.
    std::atomic<long long> slam_update_gated_static_total{0};

    // 2026-05-16 Phase 1 Step 6 — visual-only LandmarkMap counters.
    //   landmarks_added_total       new landmarks created (no dedup hit) per keyframe.
    //   landmarks_merged_total      existing landmark merged (within kDedup3DDistanceM
    //                               AND Hamming ≤ kDescriptorMaxHamming). High ratio
    //                               of merged/added indicates good revisit behavior.
    //   landmarks_culled_total      removed by cullStaleLandmarks (times_observed
    //                               < kMinObservationsAfterGrace after grace period).
    //   landmarks_skipped_invalid   addOrMergeLandmark refused input (non-finite
    //                               p_world, empty/wrong-shape descriptor). Should
    //                               stay near 0 in a healthy walk.
    //   landmark_map_size           Last observed map.size() at keyframe-storage tick.
    //                               Snapshot value (not a running sum); diff between
    //                               consecutive simulations indicates map growth rate.
    // Phase 6.2 wire site: Tracker.cpp at the keyframe storage block right after
    // the triangulation pass that fills pts3d_world.
    std::atomic<long long> landmarks_added_total{0};
    std::atomic<long long> landmarks_merged_total{0};
    std::atomic<long long> landmarks_culled_total{0};
    std::atomic<long long> landmarks_skipped_invalid{0};
    std::atomic<long long> landmark_map_size{0};

    // 2026-05-19 — Orange-dot anchor-relative render telemetry.
    // Each render frame, getLandmarkSnapshot iterates up to 500 landmarks. For
    // each, if the landmark's host_clone_id is still in EKFState::window_,
    // render via the anchor-relative path
    //     p_world_render = R_host_GtoC_LIVE.t() * p_anchor_cam + p_host_G_LIVE
    // → orange dot tracks the actual image feature regardless of EKF drift
    // between landmark-add time and render time (same mechanism live SLAM
    // features use). Otherwise fall back to the stored frozen p_world.
    // Both counters increment per-landmark-render so the ratio in the post-
    // walk event_summary tells us what fraction of orange dots benefited
    // from the anchor path versus drifted via the world-fixed fallback.
    std::atomic<long long> landmarks_rendered_anchor_total{0};
    std::atomic<long long> landmarks_rendered_world_fixed_total{0};

    // 2026-05-19 Fix #4 — Local BA refinement of LandmarkMap entries.
    //   landmarks_in_ba_solve_sum   Σ landmark observations fed into BA across
    //                               all solves in the walk. Diff to
    //                               ba_solves_total tells average landmarks/solve.
    //   landmarks_refined_total     Σ landmarks whose p_world was written
    //                               back via setLandmarkPosition after an
    //                               accepted BA solve. If 0 across a long
    //                               walk, BA is either never running or
    //                               never accepting — see ba_solves_accepted.
    //   landmark_obs_in_ba_history_min  Minimum observation-history-length
    //                               seen for a BA-input landmark (≥ 2 by
    //                               construction, but useful to confirm we
    //                               don't have a stuck-at-2 pattern that
    //                               would indicate the ring buffer's not
    //                               accumulating across keyframes).
    std::atomic<long long> landmarks_in_ba_solve_sum{0};
    std::atomic<long long> landmarks_refined_total{0};
    std::atomic<long long> landmark_obs_in_ba_history_min{0};

    // 2026-05-19 Fix #11 — landmarks_pixel_refreshed_total.
    //
    // Increments by the number of landmark-observation pixels refreshed
    // per frame from the KLT-tracked positions. At 30 Hz × ~50 active
    // observed landmarks ≈ 1500/sec. If this is 0 across a walk where
    // landmarks_matched_total > 0, the per-frame refresh path is broken
    // (likely the feature_id ↔ landmark_id link isn't being captured at
    // keyframe time). User-visible symptom of a broken refresh: orange
    // dots appear stuck at the last keyframe's pixel positions while the
    // image content moves past them.
    std::atomic<long long> landmarks_pixel_refreshed_total{0};

    // 2026-05-24 BUG-01 fix — landmarks_descriptor_refreshed_total.
    //
    // Increments each time a landmark's representative ORB descriptor is
    // recomputed to a DIFFERENT sample from its descriptor_history ring
    // (LandmarkMap::recomputeDistinctiveDescriptorLocked, ORB-SLAM3
    // ComputeDistinctiveDescriptors). This is the BUG-01 falsifier instrument:
    // the stored descriptor used to be frozen at first observation, decaying
    // the per-frame match rate as the viewpoint drifted. If this stays 0
    // across a walk where landmarks_matched_total > 0, the 5-arg touchLandmark
    // descriptor-refresh wiring is broken. Expected to rise steadily once the
    // landmark map populates. Mirrored from
    // LandmarkMap::descriptorRefreshesTotal().
    std::atomic<long long> landmarks_descriptor_refreshed_total{0};

    // 2026-05-19 Fix #9 — landmarks_reanchored_total.
    // Increments each time LoopClosureDetector accept fires AND the EKF
    // updateAbsolutePose succeeds, by the number of LandmarkMap entries
    // whose host_clone_id was still in the EKF window (so their p_world
    // could be recomputed from the corrected clone pose). On a healthy
    // revisit walk this should rise by ~50-500 per accepted LC (depends
    // on map density). If LC ACCEPT fires but this stays at 0, the
    // re-anchor lookup is missing all host clones — investigate clone
    // window marginalisation cadence vs LC accept latency.
    std::atomic<long long> landmarks_reanchored_total{0};

    // 2026-05-19 Fix #7 — per-frame SLAM update against LIVE IMU state.
    //   slam_live_updates_fired    increments per (frame × SLAM feature)
    //                              that EKFState::updateSlamFeatureLive
    //                              successfully applied (chi² passed,
    //                              valid depth, valid pose lookup).
    //   slam_live_updates_skipped  increments when the call returned false
    //                              (gated by chi², degenerate depth, or
    //                              missing anchor). Ratio fired/total
    //                              indicates how often per-frame SLAM is
    //                              converging on the EKF state.
    // If fired is 0 across a walk but the EKF is initialized and SLAM
    // features exist, something is gating updateSlamFeatureLive
    // continuously — check chi² magnitudes and depth values.
    std::atomic<long long> slam_live_updates_fired{0};
    std::atomic<long long> slam_live_updates_skipped{0};

    // 2026-05-19 Fix #10 — Parallax-baseline gate.
    //
    // Increments when EKFState::buildSlamLiveJacobianRow skips an update
    // because the parallax angle between ray_anchor and ray_now is below
    // the 0.57° threshold (cos > 0.99995). High counts during a pure-
    // axial walk indicate the gate is doing its job: the SLAM (α, β, ρ)
    // state stays stable because we refused to apply uninformative
    // updates. If this stays at 0 across an axial walk, the gate isn't
    // firing — investigate the parallax computation.
    //
    // Healthy ratios on a walk with mixed motion:
    //   skipped_no_parallax / (fired + skipped_no_parallax) ≈ 20-50%
    //   during purely-axial moments → 90%+, during lateral moves → < 10%.
    std::atomic<long long> slam_live_skipped_no_parallax{0};

    // 2026-05-19 Fix #8 — count batched applySlamLiveBatch invocations.
    // Together with slam_live_updates_fired/skipped this tells us the
    // average yield (rows applied per batch). A healthy walk should show
    // slam_live_batch_calls ≈ frames-with-SLAM-features and
    // slam_live_updates_fired ≈ slam_live_batch_calls × N_features.
    // Replaces Fix #7b's 1/3-frame throttle — every frame now batches.
    std::atomic<long long> slam_live_batch_calls{0};

    // 2026-05-20 Visual-front-end parallax-baseline gate (mirror of Fix #10).
    //
    // Cause: the previous translation-degeneracy gate in Tracker.cpp tested
    // `cv::norm(t_vo) < 0.001`, but cv::recoverPose returns t as a unit
    // vector (essential matrix only determines translation direction).
    // Empirical proof in 2026-05-20 walks: every EMAT_FAIL line logged
    // |t_vo|=1.0000. The gate never fired → translation_degenerate was
    // hard-wired false → both updateRelativeRotation (per-frame) and
    // updateGravityAlignedYaw (per-keyframe) accepted catastrophic
    // recoverPose decompositions when the user walked forward facing a
    // flat wall (low translation parallax). Result: 13 catastrophic
    // R_GtoI yaw jumps (>30°/s) per walk while Madgwick stayed at <5°/s.
    //
    // Increments when the visual front-end computes mean KLT parallax
    // angle < kVisualMinParallaxRad (= 0.01 rad ≈ 0.57°). Same threshold
    // and citation as Fix #10's slam_live_skipped_no_parallax (OpenVINS
    // UpdaterSLAM min_parallax_ratio = 0.01, Geneva et al. 2020 §III.D).
    //
    // Falsifier: forward-walk past a flat wall →
    //   visual_translation_degenerate_total > 0 (gate firing on the bad
    //     frames). If it stays at 0 across a wall-facing walk, the focal-
    //     length / parallax computation is broken — investigate before
    //     trusting that the gate is doing its job.
    std::atomic<long long> visual_translation_degenerate_total{0};

    // 2026-05-16 Phase 1 Step 6.4 — Tracker → EKF landmark-observation wire-up.
    //   landmarks_observed_total    Σ nearby_ids.size() per keyframe (all landmarks
    //                               returned by getLandmarksInRadius before any
    //                               descriptor / pixel matching). If this is large
    //                               but landmarks_matched_total is ~0, the
    //                               descriptor + 15 px gates are too strict — or
    //                               the predicted camera pose is wildly off.
    //   landmarks_matched_total     Σ obs_vec.size() per keyframe — successful
    //                               (projection + KNN + Lowe-0.75 + Hamming ≤ 50)
    //                               matches that are handed to the EKF. Diff to
    //                               landmarks_observed_total quantifies the cost
    //                               of the matching stage.
    // The downstream landmarks_msckf_* counters (accepted / rejected_*) are
    // declared by Phase 6.3 alongside EKFState::applyLandmarkObservations.
    std::atomic<long long> landmarks_observed_total{0};
    std::atomic<long long> landmarks_matched_total{0};

    // 2026-05-16 Phase 1 Step 6.3 — fixed-landmark MSCKF measurement update.
    //
    // Counters incremented by Tracker.cpp (Phase 6.4 wire site) from the
    // LandmarkUpdateResult returned by EKFState::applyLandmarkObservations.
    // EKFState itself is pure — it does NOT touch these atomics. Bucketing
    // here lets us track update quality across the entire walk:
    //   landmarks_msckf_accepted_total        observations folded into the EKF
    //                                         update batch (chi² + Huber survived).
    //   landmarks_msckf_rejected_chi2_total   per-obs χ²(0.95, 2-DOF) gate
    //                                         rejects + singular S inversions.
    //   landmarks_msckf_rejected_huber_total  per-obs Huber hard-reject
    //                                         (pixel residual ≥ 5·δ).
    //   landmarks_msckf_rejected_depth_total  camera-frame depth outside
    //                                         [kLandmarkDepthMinM, kLandmarkDepthMaxM].
    //   landmarks_msckf_rejected_image_total  predicted pixel outside the
    //                                         image rectangle.
    // Healthy walk: accepted >> sum(rejected_*); chi²+huber dominate over
    // depth+image (depth/image catch data-association bugs upstream).
    std::atomic<long long> landmarks_msckf_accepted_total{0};
    std::atomic<long long> landmarks_msckf_rejected_chi2_total{0};
    std::atomic<long long> landmarks_msckf_rejected_huber_total{0};
    std::atomic<long long> landmarks_msckf_rejected_depth_total{0};
    std::atomic<long long> landmarks_msckf_rejected_image_total{0};

    // 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
    // Incremented each time native-lib.cpp refreshes the cached OverlaySnapshot
    // from EKFState::snapshotForOverlay. If this counter is > 0 in event_summary
    // the new consolidated-snapshot render path is live; rate ≈ Kotlin overlay
    // tick rate (one refresh per overlay frame after cache TTL expiry).
    std::atomic<long long> overlay_snapshots_taken_total{0};

    // Step 3 Observer B — MiDaS depth scale constraint
    // (Tracker.cpp applyDepthScaleConstraint). Added 2026-05-07 after
    // sim 1778147132092 had no way to tell which of the seven bailout
    // gates was firing — MiDaS feedback was logcat-only, and AI_HANDOFF
    // had been carrying "MiDaS hasn't fired in any sim — investigate"
    // since the prior session.
    //   midas_entries                       every applyDepthScaleConstraint() entry
    //   midas_bailout_no_depth              depth_map_ empty (DepthEstimator silent)
    //   midas_bailout_few_pts3d             pts3d.size() < 15 (need 15 triangulated)
    //   midas_bailout_invalid_intrinsics    fx/fy ≤ 0 (calibration not loaded)
    //   midas_bailout_camera_h              [LEGACY] camera_h ∉ [0.8, 2.2] m — never fires post Step 4.2.1
    //   midas_bailout_low_g                 [LEGACY] |accel| < 5 m/s² — never fires post Step 4.2.1
    //   midas_bailout_few_floor_matches     post Step 4.2.1: < 8 valid ratios after affine fit
    //   midas_rejected_extreme              target_scale > 3× current (3x safety gate)
    //   midas_fused                         scale_fuser_.update accepted the obs
    //   midas_skipped                       scale_fuser_.update skipped the obs
    //   midas_affine_fit_singular           Step 4.2.1: det ≤ 0 or N < 8 — fit aborted
    //   midas_affine_fit_low_inliers        Step 4.2.1: residual-inlier ratio < 50 %
    //   midas_affine_fit_inlier_ratio_milli Step 4.2.1 GAUGE: latest inlier ratio × 1000
    //   midas_affine_fit_shift_milli        Step 4.2.1 GAUGE: latest fit shift t × 1000
    std::atomic<long long> midas_entries{0};
    std::atomic<long long> midas_bailout_no_depth{0};
    std::atomic<long long> midas_bailout_few_pts3d{0};
    std::atomic<long long> midas_bailout_invalid_intrinsics{0};
    std::atomic<long long> midas_bailout_camera_h{0};
    std::atomic<long long> midas_bailout_low_g{0};
    std::atomic<long long> midas_bailout_few_floor_matches{0};
    std::atomic<long long> midas_rejected_extreme{0};
    std::atomic<long long> midas_fused{0};
    std::atomic<long long> midas_skipped{0};
    // Phase 2 Step 4.2.1 — VI-Depth affine-fit diagnostics (Tracker.cpp).
    std::atomic<long long> midas_affine_fit_singular{0};
    std::atomic<long long> midas_affine_fit_low_inliers{0};
    std::atomic<long long> midas_affine_fit_inlier_ratio_milli{0};
    std::atomic<long long> midas_affine_fit_shift_milli{0};

    // 2026-05-19 Fix #12 — Per-pixel MiDaS metric-depth sampler usage.
    //
    //   midas_depth_samples                  Tracker::sampleMidasMetricDepth
    //                                        successful returns (valid metric
    //                                        depth in [0.3, 30] m).
    //   slam_promotions_seeded_with_midas    SLAM promotion sites that
    //                                        replaced the triangulated depth
    //                                        with the MiDaS metric depth
    //                                        because the two disagreed
    //                                        beyond `kMidasReplaceRatio` (2×).
    //
    // Health diagnostic: if midas_depth_samples > 0 across a walk but
    // slam_promotions_seeded_with_midas stays 0, triangulation and MiDaS
    // agree (no axial-motion divergence). On a pure-axial walk the
    // replacement counter should be > 0.
    std::atomic<long long> midas_depth_samples{0};
    std::atomic<long long> slam_promotions_seeded_with_midas{0};

    // 2026-05-20 — SLAM promotion gate instrumentation.
    //
    // `slam_promotions_total` (the success counter) was 0 in replay despite
    // the rest of the pipeline firing (landmarks, MSCKF, BA, LC attempts).
    // These per-gate counters expose which rejection reason dominates so we
    // can fix the upstream cause instead of loosening the gate value.
    //
    //   slam_promo_candidates_total            features passed
    //                                          getPromotableFeatures (age,
    //                                          kf_count, in_window gates)
    //   slam_promo_rejected_cap_total          MAX_SLAM_FEATURES reached
    //   slam_promo_rejected_no_obs_total       getObservations returned <2
    //   slam_promo_rejected_no_anchor_far_total no two distinct in-window
    //                                          clones for triangulation
    //   slam_promo_rejected_baseline_total     anchor-far baseline < 1.5 cm
    //   slam_promo_rejected_solve_total        cv::solve numerical failure
    //   slam_promo_rejected_chirality_total    tA or tB ≤ 0.05 m (behind
    //                                          camera or too close)
    //   slam_promo_rejected_n_used_total       <2 observations survived the
    //                                          per-obs reprojection loop
    //   slam_promo_rejected_rms_total          two-view RMS > 1.5 px
    //   slam_promo_rejected_parallax_total     2026-05-20 — cos(angle between
    //                                          dAw_n and dBw_n) > 0.99985
    //                                          (parallax < ~1°). Mirrors
    //                                          Fix #10 / front-end parallax
    //                                          gate. Catches the case where
    //                                          baseline ≥ 1.5 cm but the rays
    //                                          are still near-parallel (user
    //                                          walking slowly toward a wall)
    //                                          → midpoint triangulation depth
    //                                          blows up to 400-680 m even
    //                                          though the reprojection RMS
    //                                          gate passes (RMS is in pixel
    //                                          space; at low parallax depth
    //                                          error doesn't translate to
    //                                          pixel error). Empirical
    //                                          evidence: parallax_fix_walk_
    //                                          2026_05_20 had 6 promotions
    //                                          at z_tri=403-680 m vs MiDaS
    //                                          z=1.4-1.6 m.
    //   slam_promo_rms_milli_p95               GAUGE: latest rejected RMS
    //                                          × 1000 — diagnose magnitude
    //
    // Health: if rejected_rms dominates, the EKF clone poses are wrong
    // (heading drift, R_GtoI seeding error). If rejected_baseline
    // dominates, the user wasn't moving enough. If rejected_chirality
    // dominates, p_world is being computed behind the camera (sign-flip
    // bug). If rejected_parallax dominates, the user IS moving (baseline
    // gate passes) but always near a wall (no parallax) — promotions
    // suppressed correctly. All-zero rejections suggests promotable came
    // back empty.
    std::atomic<long long> slam_promo_candidates_total{0};
    std::atomic<long long> slam_promo_rejected_cap_total{0};
    std::atomic<long long> slam_promo_rejected_no_obs_total{0};
    std::atomic<long long> slam_promo_rejected_no_anchor_far_total{0};
    std::atomic<long long> slam_promo_rejected_baseline_total{0};
    std::atomic<long long> slam_promo_rejected_solve_total{0};
    std::atomic<long long> slam_promo_rejected_chirality_total{0};
    std::atomic<long long> slam_promo_rejected_n_used_total{0};
    std::atomic<long long> slam_promo_rejected_rms_total{0};
    std::atomic<long long> slam_promo_rejected_parallax_total{0};
    std::atomic<long long> slam_promo_rms_milli_p95{0};
    // 2026-05-26 Hidden Bug #3 — REAL distribution of REJECTED two-view RMS (the
    // *_p95 above is a misnomer: it's the LAST rejected RMS, not a percentile).
    // mean = slam_promo_rms_sum_milli / (1000 * slam_promo_rejected_rms_total);
    // max = slam_promo_rms_milli_max. mean ~2px ⇒ marginal (parallax/baseline);
    // mean ~10px+ ⇒ clone poses inconsistent upstream (do NOT loosen the 1.5px gate).
    std::atomic<long long> slam_promo_rms_sum_milli{0};
    std::atomic<long long> slam_promo_rms_milli_max{0};

    // 2026-05-17 — translation gated because motion is rotation-only.
    // Catches "user sitting in chair rotating phone" that defeats is_static
    // (gyro != 0) and is_pure_rotation (Rayleigh threshold too strict).
    std::atomic<long long> global_t_gated_rotation_dominated_total{0};

    // 2026-05-17 — revert path: per-frame heading-projection runs after
    // the rotation_dominated / static gate passes. High count + low
    // total_path_dm → visual yaw stuck (heading noise dominates) ;
    // high count + plausible total_path_dm → working as v22 did.
    std::atomic<long long> translation_heading_projection_total{0};

    // Reset every counter to 0. Called on simulator-recording start so
    // each sim begins with a clean slate. Cheap atomic stores; safe to
    // call concurrently with hot-path increments (we accept that a few
    // increments queued just before the reset may be lost — this is
    // intentional and matches the recording-start semantics).
    void reset() {
        reloc_orb_accepts.store(0, std::memory_order_relaxed);
        reloc_orb_rejects.store(0, std::memory_order_relaxed);
        reloc_orb_size_skipped.store(0, std::memory_order_relaxed);
        reloc_orb_slam_guarded.store(0, std::memory_order_relaxed);
        blur_enter_events.store(0, std::memory_order_relaxed);
        blur_total_skip_frames.store(0, std::memory_order_relaxed);
        lowlight_state_active_frames.store(0, std::memory_order_relaxed);
        lowlight_log_lines.store(0, std::memory_order_relaxed);
        rot_gate_pure_rot_confirmed.store(0, std::memory_order_relaxed);
        rot_gate_log_lines.store(0, std::memory_order_relaxed);
        klt_adaptive_window_hits.store(0, std::memory_order_relaxed);
        ba_solves_total.store(0, std::memory_order_relaxed);
        ba_solves_accepted.store(0, std::memory_order_relaxed);
        ba_solves_rejected.store(0, std::memory_order_relaxed);
        ba_skipped_in_flight.store(0, std::memory_order_relaxed);
        ba_solve_us_sum.store(0, std::memory_order_relaxed);
        ba_solve_us_max.store(0, std::memory_order_relaxed);
        ba_iters_sum.store(0, std::memory_order_relaxed);
        ba_skipped_no_init.store(0, std::memory_order_relaxed);
        ba_skipped_too_few_clones.store(0, std::memory_order_relaxed);
        ba_skipped_no_intrinsics.store(0, std::memory_order_relaxed);
        ba_skipped_too_few_landmarks.store(0, std::memory_order_relaxed);
        slam_promotions_total.store(0, std::memory_order_relaxed);
        slam_lifetime_obs_sum.store(0, std::memory_order_relaxed);
        slam_lifetime_count.store(0, std::memory_order_relaxed);
        slam_reanchor_total.store(0, std::memory_order_relaxed);
        slam_reanchor_failed_no_clone.store(0, std::memory_order_relaxed);
        slam_reanchor_failed_geometry.store(0, std::memory_order_relaxed);
        msckf_update_lines.store(0, std::memory_order_relaxed);
        msckf_huber_rejected_sum.store(0, std::memory_order_relaxed);
        applyMSCKFUpdate_inversion_failed.store(0, std::memory_order_relaxed);
        ba_singular_landmarks.store(0, std::memory_order_relaxed);
        propagate_skipped.store(0, std::memory_order_relaxed);
        propagate_skipped_not_init.store(0, std::memory_order_relaxed);
        propagate_skipped_dt_nonpositive.store(0, std::memory_order_relaxed);
        propagate_skipped_p_empty.store(0, std::memory_order_relaxed);
        msckf_chi2_rejected.store(0, std::memory_order_relaxed);
        msckf_inversion_failed.store(0, std::memory_order_relaxed);
        velocity_clamped.store(0, std::memory_order_relaxed);
        scale_chi2_rejected.store(0, std::memory_order_relaxed);
        time_offset_clamped.store(0, std::memory_order_relaxed);
        fej_clone_miss.store(0, std::memory_order_relaxed);
        gravity_alignment_rejected.store(0, std::memory_order_relaxed);
        relative_rotation_rejected.store(0, std::memory_order_relaxed);
        loop_closure_rejects_heading_total.store(0, std::memory_order_relaxed);
        loop_closure_rejects_pnp_total.store(0, std::memory_order_relaxed);
        loop_closure_attempts.store(0, std::memory_order_relaxed);
        loop_closure_accepts.store(0, std::memory_order_relaxed);
        loop_closure_rejects_low_score.store(0, std::memory_order_relaxed);
        loop_closure_rejects_pnp.store(0, std::memory_order_relaxed);
        loop_closure_rejects_heading.store(0, std::memory_order_relaxed);
        loop_closure_rejects_rot_sanity_total.store(0, std::memory_order_relaxed);
        visual_yaw_chi2_rejected_total.store(0, std::memory_order_relaxed);
        visual_relative_rotation_chi2_rejected_total.store(0, std::memory_order_relaxed);
        visual_relative_rotation_gyro_mismatch_total.store(0, std::memory_order_relaxed);
        madgwick_visual_yaw_nudges_total.store(0, std::memory_order_relaxed);
        loop_closure_kf_count_in_db.store(0, std::memory_order_relaxed);
        loop_closure_chi2_rejected.store(0, std::memory_order_relaxed);
        loop_closure_corrections_applied.store(0, std::memory_order_relaxed);
        loop_closure_lower_rank_accepts.store(0, std::memory_order_relaxed);
        loop_closure_spatial_candidates_total.store(0, std::memory_order_relaxed);
        loop_closure_spatial_fallback_total.store(0, std::memory_order_relaxed);
        cam_fps_mean_milli_hz.store(0, std::memory_order_relaxed);
        cam_fps_stdev_milli_hz.store(0, std::memory_order_relaxed);
        cam_fps_window_count.store(0, std::memory_order_relaxed);
        loop_closure_geom_attempts.store(0, std::memory_order_relaxed);
        loop_closure_geom_accepts.store(0, std::memory_order_relaxed);
        loop_closure_geom_rejects_no_position.store(0, std::memory_order_relaxed);
        loop_closure_geom_rejects_no_candidate.store(0, std::memory_order_relaxed);
        loop_closure_geom_rejects_few_inframe.store(0, std::memory_order_relaxed);
        loop_closure_geom_rejects_pnp.store(0, std::memory_order_relaxed);
        loop_closure_geom_rejects_descriptor.store(0, std::memory_order_relaxed);
        loop_closure_geom_spatial_miss.store(0, std::memory_order_relaxed);
        loop_closure_geom_hamming_miss.store(0, std::memory_order_relaxed);
        loop_closure_geom_hamming_min_sum.store(0, std::memory_order_relaxed);
        loop_closure_geom_hamming_min_count.store(0, std::memory_order_relaxed);
        total_path_dm.store(0, std::memory_order_relaxed);
        pose_graph_optimize_calls.store(0, std::memory_order_relaxed);
        pose_graph_iters_used_sum.store(0, std::memory_order_relaxed);
        pose_graph_residual_norm_pre_mm.store(0, std::memory_order_relaxed);
        pose_graph_residual_norm_post_mm.store(0, std::memory_order_relaxed);
        pose_graph_max_correction_mm.store(0, std::memory_order_relaxed);
        pose_graph_max_correction_mrad.store(0, std::memory_order_relaxed);
        pose_graph_rejected_singular.store(0, std::memory_order_relaxed);
        pose_graph_loop_edges_added.store(0, std::memory_order_relaxed);
        pose_graph_apply_calls.store(0, std::memory_order_relaxed);
        extrinsics_rotation_angle_mdeg.store(0, std::memory_order_relaxed);
        // PHASE_A_8B_REENABLE_2026_05_19
        rbc_runaway_total.store(0, std::memory_order_relaxed);
        // PHASE_B_6_4_REENABLE_2026_05_19
        landmark_obs_runaway_total.store(0, std::memory_order_relaxed);
        cam_imu_time_offset_us.store(0, std::memory_order_relaxed);
        rolling_shutter_skew_ns.store(0, std::memory_order_relaxed);
        ekf_init_deferred_madgwick_not_ready.store(0, std::memory_order_relaxed);
        zrup_fired_total.store(0, std::memory_order_relaxed);
        zupt_rotinplace_fired.store(0, std::memory_order_relaxed);
        depth_flow_updates.store(0, std::memory_order_relaxed);
        depth_flow_skipped_few_pts.store(0, std::memory_order_relaxed);
        depth_flow_outlier_rejected.store(0, std::memory_order_relaxed);
        depth_flow_total_mm.store(0, std::memory_order_relaxed);
        depth_flow_calib_updates.store(0, std::memory_order_relaxed);
        midas_scale_k_milli.store(0, std::memory_order_relaxed);
        midas_scale_k_min_milli.store(0, std::memory_order_relaxed);
        midas_scale_k_max_milli.store(0, std::memory_order_relaxed);
        depth_flow_looming_updates.store(0, std::memory_order_relaxed);
        depth_flow_looming_used.store(0, std::memory_order_relaxed);
        depth_flow_looming_skipped.store(0, std::memory_order_relaxed);
        ekf_bg_absorbed_total.store(0, std::memory_order_relaxed);
        gyro_bias_pushed_total.store(0, std::memory_order_relaxed);
        visual_yaw_gated_pure_rotation_total.store(0, std::memory_order_relaxed);
        msckf_gated_static_total.store(0, std::memory_order_relaxed);
        slam_update_gated_static_total.store(0, std::memory_order_relaxed);
        landmarks_added_total.store(0, std::memory_order_relaxed);
        landmarks_merged_total.store(0, std::memory_order_relaxed);
        landmarks_culled_total.store(0, std::memory_order_relaxed);
        landmarks_skipped_invalid.store(0, std::memory_order_relaxed);
        landmark_map_size.store(0, std::memory_order_relaxed);
        landmarks_rendered_anchor_total.store(0, std::memory_order_relaxed);
        landmarks_rendered_world_fixed_total.store(0, std::memory_order_relaxed);
        landmarks_in_ba_solve_sum.store(0, std::memory_order_relaxed);
        landmarks_refined_total.store(0, std::memory_order_relaxed);
        landmark_obs_in_ba_history_min.store(0, std::memory_order_relaxed);
        landmarks_reanchored_total.store(0, std::memory_order_relaxed);
        landmarks_pixel_refreshed_total.store(0, std::memory_order_relaxed);
        landmarks_descriptor_refreshed_total.store(0, std::memory_order_relaxed);
        slam_live_updates_fired.store(0, std::memory_order_relaxed);
        slam_live_updates_skipped.store(0, std::memory_order_relaxed);
        slam_live_skipped_no_parallax.store(0, std::memory_order_relaxed);
        slam_live_batch_calls.store(0, std::memory_order_relaxed);
        visual_translation_degenerate_total.store(0, std::memory_order_relaxed);
        landmarks_observed_total.store(0, std::memory_order_relaxed);
        landmarks_matched_total.store(0, std::memory_order_relaxed);
        landmarks_msckf_accepted_total.store(0, std::memory_order_relaxed);
        landmarks_msckf_rejected_chi2_total.store(0, std::memory_order_relaxed);
        landmarks_msckf_rejected_huber_total.store(0, std::memory_order_relaxed);
        landmarks_msckf_rejected_depth_total.store(0, std::memory_order_relaxed);
        landmarks_msckf_rejected_image_total.store(0, std::memory_order_relaxed);
        overlay_snapshots_taken_total.store(0, std::memory_order_relaxed);  // 2026-05-16 single-snapshot consistency fix
        midas_entries.store(0, std::memory_order_relaxed);
        midas_bailout_no_depth.store(0, std::memory_order_relaxed);
        midas_bailout_few_pts3d.store(0, std::memory_order_relaxed);
        midas_bailout_invalid_intrinsics.store(0, std::memory_order_relaxed);
        midas_bailout_camera_h.store(0, std::memory_order_relaxed);
        midas_bailout_low_g.store(0, std::memory_order_relaxed);
        midas_bailout_few_floor_matches.store(0, std::memory_order_relaxed);
        midas_rejected_extreme.store(0, std::memory_order_relaxed);
        midas_fused.store(0, std::memory_order_relaxed);
        midas_skipped.store(0, std::memory_order_relaxed);
        midas_affine_fit_singular.store(0, std::memory_order_relaxed);
        midas_affine_fit_low_inliers.store(0, std::memory_order_relaxed);
        midas_affine_fit_inlier_ratio_milli.store(0, std::memory_order_relaxed);
        midas_affine_fit_shift_milli.store(0, std::memory_order_relaxed);
        midas_depth_samples.store(0, std::memory_order_relaxed);
        slam_promotions_seeded_with_midas.store(0, std::memory_order_relaxed);
        slam_promo_candidates_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_cap_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_no_obs_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_no_anchor_far_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_baseline_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_solve_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_chirality_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_n_used_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_rms_total.store(0, std::memory_order_relaxed);
        slam_promo_rejected_parallax_total.store(0, std::memory_order_relaxed);
        slam_promo_rms_milli_p95.store(0, std::memory_order_relaxed);
        slam_promo_rms_sum_milli.store(0, std::memory_order_relaxed);
        slam_promo_rms_milli_max.store(0, std::memory_order_relaxed);
        global_t_gated_rotation_dominated_total.store(0, std::memory_order_relaxed);
        translation_heading_projection_total.store(0, std::memory_order_relaxed);
    }

    // Monotonic max-update for ba_solve_us_max. Lock-free CAS loop.
    void update_ba_solve_us_max(long long candidate) {
        long long prev = ba_solve_us_max.load(std::memory_order_relaxed);
        while (candidate > prev &&
               !ba_solve_us_max.compare_exchange_weak(prev, candidate,
                                                      std::memory_order_relaxed)) {
            // prev is reloaded by compare_exchange_weak; loop until either
            // candidate <= prev (someone else won with a larger value) or
            // we successfully publish candidate.
        }
    }

    // Snapshot every counter into a flat one-line JSON object string.
    // Format: {"key1":N,"key2":N,...} — no whitespace, suitable for
    // dropping straight into the simulator JSON.
    std::string serializeAsJsonString() const {
        // Snapshot first so the JSON is internally consistent (a bit, anyway —
        // increments racing with the read are still possible across fields,
        // but never within a single field).
        const long long v_reloc_orb_accepts             = reloc_orb_accepts.load(std::memory_order_relaxed);
        const long long v_reloc_orb_rejects             = reloc_orb_rejects.load(std::memory_order_relaxed);
        const long long v_reloc_orb_size_skipped        = reloc_orb_size_skipped.load(std::memory_order_relaxed);
        const long long v_reloc_orb_slam_guarded        = reloc_orb_slam_guarded.load(std::memory_order_relaxed);
        const long long v_blur_enter_events             = blur_enter_events.load(std::memory_order_relaxed);
        const long long v_blur_total_skip_frames        = blur_total_skip_frames.load(std::memory_order_relaxed);
        const long long v_lowlight_state_active_frames  = lowlight_state_active_frames.load(std::memory_order_relaxed);
        const long long v_lowlight_log_lines            = lowlight_log_lines.load(std::memory_order_relaxed);
        const long long v_rot_gate_pure_rot_confirmed   = rot_gate_pure_rot_confirmed.load(std::memory_order_relaxed);
        const long long v_rot_gate_log_lines            = rot_gate_log_lines.load(std::memory_order_relaxed);
        const long long v_klt_adaptive_window_hits      = klt_adaptive_window_hits.load(std::memory_order_relaxed);
        const long long v_ba_solves_total               = ba_solves_total.load(std::memory_order_relaxed);
        const long long v_ba_solves_accepted            = ba_solves_accepted.load(std::memory_order_relaxed);
        const long long v_ba_solves_rejected            = ba_solves_rejected.load(std::memory_order_relaxed);
        const long long v_ba_skipped_in_flight          = ba_skipped_in_flight.load(std::memory_order_relaxed);
        const long long v_ba_solve_us_sum               = ba_solve_us_sum.load(std::memory_order_relaxed);
        const long long v_ba_solve_us_max               = ba_solve_us_max.load(std::memory_order_relaxed);
        const long long v_ba_iters_sum                  = ba_iters_sum.load(std::memory_order_relaxed);
        const long long v_ba_skipped_no_init            = ba_skipped_no_init.load(std::memory_order_relaxed);
        const long long v_ba_skipped_too_few_clones     = ba_skipped_too_few_clones.load(std::memory_order_relaxed);
        const long long v_ba_skipped_no_intrinsics      = ba_skipped_no_intrinsics.load(std::memory_order_relaxed);
        const long long v_ba_skipped_too_few_landmarks  = ba_skipped_too_few_landmarks.load(std::memory_order_relaxed);
        const long long v_slam_promotions_total         = slam_promotions_total.load(std::memory_order_relaxed);
        const long long v_slam_lifetime_obs_sum         = slam_lifetime_obs_sum.load(std::memory_order_relaxed);
        const long long v_slam_lifetime_count           = slam_lifetime_count.load(std::memory_order_relaxed);
        const long long v_slam_reanchor_total           = slam_reanchor_total.load(std::memory_order_relaxed);
        const long long v_slam_reanchor_failed_no_clone = slam_reanchor_failed_no_clone.load(std::memory_order_relaxed);
        const long long v_slam_reanchor_failed_geometry = slam_reanchor_failed_geometry.load(std::memory_order_relaxed);
        const long long v_msckf_update_lines            = msckf_update_lines.load(std::memory_order_relaxed);
        const long long v_msckf_huber_rejected_sum      = msckf_huber_rejected_sum.load(std::memory_order_relaxed);
        const long long v_applyMSCKFUpdate_inversion_failed = applyMSCKFUpdate_inversion_failed.load(std::memory_order_relaxed);
        const long long v_ba_singular_landmarks          = ba_singular_landmarks.load(std::memory_order_relaxed);
        const long long v_propagate_skipped              = propagate_skipped.load(std::memory_order_relaxed);
        const long long v_propagate_skipped_not_init     = propagate_skipped_not_init.load(std::memory_order_relaxed);
        const long long v_propagate_skipped_dt_nonpositive = propagate_skipped_dt_nonpositive.load(std::memory_order_relaxed);
        const long long v_propagate_skipped_p_empty      = propagate_skipped_p_empty.load(std::memory_order_relaxed);
        const long long v_msckf_chi2_rejected            = msckf_chi2_rejected.load(std::memory_order_relaxed);
        const long long v_msckf_inversion_failed         = msckf_inversion_failed.load(std::memory_order_relaxed);
        const long long v_velocity_clamped               = velocity_clamped.load(std::memory_order_relaxed);
        const long long v_scale_chi2_rejected            = scale_chi2_rejected.load(std::memory_order_relaxed);
        const long long v_time_offset_clamped            = time_offset_clamped.load(std::memory_order_relaxed);
        const long long v_fej_clone_miss                 = fej_clone_miss.load(std::memory_order_relaxed);
        const long long v_gravity_alignment_rejected     = gravity_alignment_rejected.load(std::memory_order_relaxed);
        const long long v_relative_rotation_rejected     = relative_rotation_rejected.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_heading_total = loop_closure_rejects_heading_total.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_pnp_total = loop_closure_rejects_pnp_total.load(std::memory_order_relaxed);
        const long long v_loop_closure_attempts         = loop_closure_attempts.load(std::memory_order_relaxed);
        const long long v_loop_closure_accepts          = loop_closure_accepts.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_low_score= loop_closure_rejects_low_score.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_pnp      = loop_closure_rejects_pnp.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_heading  = loop_closure_rejects_heading.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_rot_sanity_total = loop_closure_rejects_rot_sanity_total.load(std::memory_order_relaxed);
        const long long v_visual_yaw_chi2_rejected_total = visual_yaw_chi2_rejected_total.load(std::memory_order_relaxed);
        const long long v_visual_relative_rotation_chi2_rejected_total = visual_relative_rotation_chi2_rejected_total.load(std::memory_order_relaxed);
        const long long v_visual_relative_rotation_gyro_mismatch_total = visual_relative_rotation_gyro_mismatch_total.load(std::memory_order_relaxed);
        const long long v_madgwick_visual_yaw_nudges_total = madgwick_visual_yaw_nudges_total.load(std::memory_order_relaxed);
        const long long v_loop_closure_kf_count_in_db   = loop_closure_kf_count_in_db.load(std::memory_order_relaxed);
        const long long v_loop_closure_chi2_rejected    = loop_closure_chi2_rejected.load(std::memory_order_relaxed);
        const long long v_loop_closure_corrections_applied = loop_closure_corrections_applied.load(std::memory_order_relaxed);
        const long long v_loop_closure_lower_rank_accepts   = loop_closure_lower_rank_accepts.load(std::memory_order_relaxed);
        const long long v_loop_closure_spatial_candidates_total = loop_closure_spatial_candidates_total.load(std::memory_order_relaxed);
        const long long v_loop_closure_spatial_fallback_total   = loop_closure_spatial_fallback_total.load(std::memory_order_relaxed);
        const long long v_cam_fps_mean_milli_hz             = cam_fps_mean_milli_hz.load(std::memory_order_relaxed);
        const long long v_cam_fps_stdev_milli_hz            = cam_fps_stdev_milli_hz.load(std::memory_order_relaxed);
        const long long v_cam_fps_window_count              = cam_fps_window_count.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_attempts                 = loop_closure_geom_attempts.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_accepts                  = loop_closure_geom_accepts.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_rejects_no_position      = loop_closure_geom_rejects_no_position.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_rejects_no_candidate     = loop_closure_geom_rejects_no_candidate.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_rejects_few_inframe      = loop_closure_geom_rejects_few_inframe.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_rejects_pnp              = loop_closure_geom_rejects_pnp.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_rejects_descriptor       = loop_closure_geom_rejects_descriptor.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_spatial_miss             = loop_closure_geom_spatial_miss.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_hamming_miss             = loop_closure_geom_hamming_miss.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_hamming_min_sum          = loop_closure_geom_hamming_min_sum.load(std::memory_order_relaxed);
        const long long v_loop_closure_geom_hamming_min_count        = loop_closure_geom_hamming_min_count.load(std::memory_order_relaxed);
        const long long v_total_path_dm               = total_path_dm.load(std::memory_order_relaxed);
        const long long v_pose_graph_optimize_calls          = pose_graph_optimize_calls.load(std::memory_order_relaxed);
        const long long v_pose_graph_iters_used_sum          = pose_graph_iters_used_sum.load(std::memory_order_relaxed);
        const long long v_pose_graph_residual_norm_pre_mm    = pose_graph_residual_norm_pre_mm.load(std::memory_order_relaxed);
        const long long v_pose_graph_residual_norm_post_mm   = pose_graph_residual_norm_post_mm.load(std::memory_order_relaxed);
        const long long v_pose_graph_max_correction_mm       = pose_graph_max_correction_mm.load(std::memory_order_relaxed);
        const long long v_pose_graph_max_correction_mrad     = pose_graph_max_correction_mrad.load(std::memory_order_relaxed);
        const long long v_pose_graph_rejected_singular       = pose_graph_rejected_singular.load(std::memory_order_relaxed);
        const long long v_pose_graph_loop_edges_added        = pose_graph_loop_edges_added.load(std::memory_order_relaxed);
        const long long v_pose_graph_apply_calls             = pose_graph_apply_calls.load(std::memory_order_relaxed);
        const long long v_extrinsics_rotation_angle_mdeg = extrinsics_rotation_angle_mdeg.load(std::memory_order_relaxed);
        // PHASE_A_8B_REENABLE_2026_05_19
        const long long v_rbc_runaway_total = rbc_runaway_total.load(std::memory_order_relaxed);
        // PHASE_B_6_4_REENABLE_2026_05_19
        const long long v_landmark_obs_runaway_total = landmark_obs_runaway_total.load(std::memory_order_relaxed);
        const long long v_cam_imu_time_offset_us  = cam_imu_time_offset_us.load(std::memory_order_relaxed);
        const long long v_rolling_shutter_skew_ns = rolling_shutter_skew_ns.load(std::memory_order_relaxed);
        const long long v_ekf_init_deferred_madgwick_not_ready = ekf_init_deferred_madgwick_not_ready.load(std::memory_order_relaxed);
        const long long v_zrup_fired_total = zrup_fired_total.load(std::memory_order_relaxed);
        const long long v_zupt_rotinplace_fired = zupt_rotinplace_fired.load(std::memory_order_relaxed);
        const long long v_depth_flow_updates = depth_flow_updates.load(std::memory_order_relaxed);
        const long long v_depth_flow_skipped_few_pts = depth_flow_skipped_few_pts.load(std::memory_order_relaxed);
        const long long v_depth_flow_outlier_rejected = depth_flow_outlier_rejected.load(std::memory_order_relaxed);
        const long long v_depth_flow_total_mm = depth_flow_total_mm.load(std::memory_order_relaxed);
        const long long v_depth_flow_calib_updates = depth_flow_calib_updates.load(std::memory_order_relaxed);
        const long long v_midas_scale_k_milli = midas_scale_k_milli.load(std::memory_order_relaxed);
        const long long v_midas_scale_k_min_milli = midas_scale_k_min_milli.load(std::memory_order_relaxed);
        const long long v_midas_scale_k_max_milli = midas_scale_k_max_milli.load(std::memory_order_relaxed);
        const long long v_depth_flow_looming_updates = depth_flow_looming_updates.load(std::memory_order_relaxed);
        const long long v_depth_flow_looming_used = depth_flow_looming_used.load(std::memory_order_relaxed);
        const long long v_depth_flow_looming_skipped = depth_flow_looming_skipped.load(std::memory_order_relaxed);
        const long long v_ekf_bg_absorbed_total = ekf_bg_absorbed_total.load(std::memory_order_relaxed);
        const long long v_gyro_bias_pushed_total = gyro_bias_pushed_total.load(std::memory_order_relaxed);
        const long long v_visual_yaw_gated_pure_rotation_total = visual_yaw_gated_pure_rotation_total.load(std::memory_order_relaxed);
        const long long v_msckf_gated_static_total = msckf_gated_static_total.load(std::memory_order_relaxed);
        const long long v_slam_update_gated_static_total = slam_update_gated_static_total.load(std::memory_order_relaxed);
        const long long v_landmarks_added_total     = landmarks_added_total.load(std::memory_order_relaxed);
        const long long v_landmarks_merged_total    = landmarks_merged_total.load(std::memory_order_relaxed);
        const long long v_landmarks_culled_total    = landmarks_culled_total.load(std::memory_order_relaxed);
        const long long v_landmarks_skipped_invalid = landmarks_skipped_invalid.load(std::memory_order_relaxed);
        const long long v_landmark_map_size         = landmark_map_size.load(std::memory_order_relaxed);
        const long long v_landmarks_rendered_anchor_total      = landmarks_rendered_anchor_total.load(std::memory_order_relaxed);
        const long long v_landmarks_rendered_world_fixed_total = landmarks_rendered_world_fixed_total.load(std::memory_order_relaxed);
        const long long v_landmarks_in_ba_solve_sum      = landmarks_in_ba_solve_sum.load(std::memory_order_relaxed);
        const long long v_landmarks_refined_total        = landmarks_refined_total.load(std::memory_order_relaxed);
        const long long v_landmarks_reanchored_total     = landmarks_reanchored_total.load(std::memory_order_relaxed);
        const long long v_landmarks_pixel_refreshed_total = landmarks_pixel_refreshed_total.load(std::memory_order_relaxed);
        const long long v_landmarks_descriptor_refreshed_total = landmarks_descriptor_refreshed_total.load(std::memory_order_relaxed);
        const long long v_landmark_obs_in_ba_history_min = landmark_obs_in_ba_history_min.load(std::memory_order_relaxed);
        const long long v_slam_live_updates_fired        = slam_live_updates_fired.load(std::memory_order_relaxed);
        const long long v_slam_live_updates_skipped      = slam_live_updates_skipped.load(std::memory_order_relaxed);
        const long long v_slam_live_skipped_no_parallax  = slam_live_skipped_no_parallax.load(std::memory_order_relaxed);
        const long long v_slam_live_batch_calls          = slam_live_batch_calls.load(std::memory_order_relaxed);
        const long long v_visual_translation_degenerate_total = visual_translation_degenerate_total.load(std::memory_order_relaxed);
        const long long v_landmarks_observed_total  = landmarks_observed_total.load(std::memory_order_relaxed);
        const long long v_landmarks_matched_total   = landmarks_matched_total.load(std::memory_order_relaxed);
        const long long v_landmarks_msckf_accepted_total       = landmarks_msckf_accepted_total.load(std::memory_order_relaxed);
        const long long v_landmarks_msckf_rejected_chi2_total  = landmarks_msckf_rejected_chi2_total.load(std::memory_order_relaxed);
        const long long v_landmarks_msckf_rejected_huber_total = landmarks_msckf_rejected_huber_total.load(std::memory_order_relaxed);
        const long long v_landmarks_msckf_rejected_depth_total = landmarks_msckf_rejected_depth_total.load(std::memory_order_relaxed);
        const long long v_landmarks_msckf_rejected_image_total = landmarks_msckf_rejected_image_total.load(std::memory_order_relaxed);
        // 2026-05-16 single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
        const long long v_overlay_snapshots_taken_total        = overlay_snapshots_taken_total.load(std::memory_order_relaxed);
        const long long v_midas_entries                       = midas_entries.load(std::memory_order_relaxed);
        const long long v_midas_bailout_no_depth              = midas_bailout_no_depth.load(std::memory_order_relaxed);
        const long long v_midas_bailout_few_pts3d             = midas_bailout_few_pts3d.load(std::memory_order_relaxed);
        const long long v_midas_bailout_invalid_intrinsics    = midas_bailout_invalid_intrinsics.load(std::memory_order_relaxed);
        const long long v_midas_bailout_camera_h              = midas_bailout_camera_h.load(std::memory_order_relaxed);
        const long long v_midas_bailout_low_g                 = midas_bailout_low_g.load(std::memory_order_relaxed);
        const long long v_midas_bailout_few_floor_matches     = midas_bailout_few_floor_matches.load(std::memory_order_relaxed);
        const long long v_midas_rejected_extreme              = midas_rejected_extreme.load(std::memory_order_relaxed);
        const long long v_midas_fused                         = midas_fused.load(std::memory_order_relaxed);
        const long long v_midas_skipped                       = midas_skipped.load(std::memory_order_relaxed);
        const long long v_midas_affine_fit_singular           = midas_affine_fit_singular.load(std::memory_order_relaxed);
        const long long v_midas_affine_fit_low_inliers        = midas_affine_fit_low_inliers.load(std::memory_order_relaxed);
        const long long v_midas_affine_fit_inlier_ratio_milli = midas_affine_fit_inlier_ratio_milli.load(std::memory_order_relaxed);
        const long long v_midas_affine_fit_shift_milli        = midas_affine_fit_shift_milli.load(std::memory_order_relaxed);
        const long long v_midas_depth_samples                 = midas_depth_samples.load(std::memory_order_relaxed);
        const long long v_slam_promotions_seeded_with_midas   = slam_promotions_seeded_with_midas.load(std::memory_order_relaxed);
        const long long v_slam_promo_candidates_total         = slam_promo_candidates_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_cap_total       = slam_promo_rejected_cap_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_no_obs_total    = slam_promo_rejected_no_obs_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_no_anchor_far_total = slam_promo_rejected_no_anchor_far_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_baseline_total  = slam_promo_rejected_baseline_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_solve_total     = slam_promo_rejected_solve_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_chirality_total = slam_promo_rejected_chirality_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_n_used_total    = slam_promo_rejected_n_used_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_rms_total       = slam_promo_rejected_rms_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rejected_parallax_total  = slam_promo_rejected_parallax_total.load(std::memory_order_relaxed);
        const long long v_slam_promo_rms_milli_p95            = slam_promo_rms_milli_p95.load(std::memory_order_relaxed);
        const long long v_slam_promo_rms_sum_milli            = slam_promo_rms_sum_milli.load(std::memory_order_relaxed);
        const long long v_slam_promo_rms_milli_max            = slam_promo_rms_milli_max.load(std::memory_order_relaxed);
        const long long v_global_t_gated_rotation_dominated_total = global_t_gated_rotation_dominated_total.load(std::memory_order_relaxed);
        const long long v_translation_heading_projection_total = translation_heading_projection_total.load(std::memory_order_relaxed);

        std::string out;
        out.reserve(2200);  // expanded for 18 new silent-failure counters (2026-05-16)
        out += '{';
        appendKv(out, "reloc_orb_accepts",            v_reloc_orb_accepts);            out += ',';
        appendKv(out, "reloc_orb_rejects",            v_reloc_orb_rejects);            out += ',';
        appendKv(out, "reloc_orb_size_skipped",       v_reloc_orb_size_skipped);       out += ',';
        appendKv(out, "reloc_orb_slam_guarded",       v_reloc_orb_slam_guarded);       out += ',';
        appendKv(out, "blur_enter_events",            v_blur_enter_events);            out += ',';
        appendKv(out, "blur_total_skip_frames",       v_blur_total_skip_frames);       out += ',';
        appendKv(out, "lowlight_state_active_frames", v_lowlight_state_active_frames); out += ',';
        appendKv(out, "lowlight_log_lines",           v_lowlight_log_lines);           out += ',';
        appendKv(out, "rot_gate_pure_rot_confirmed",  v_rot_gate_pure_rot_confirmed);  out += ',';
        appendKv(out, "rot_gate_log_lines",           v_rot_gate_log_lines);           out += ',';
        appendKv(out, "klt_adaptive_window_hits",     v_klt_adaptive_window_hits);     out += ',';
        appendKv(out, "ba_solves_total",              v_ba_solves_total);              out += ',';
        appendKv(out, "ba_solves_accepted",           v_ba_solves_accepted);           out += ',';
        appendKv(out, "ba_solves_rejected",           v_ba_solves_rejected);           out += ',';
        appendKv(out, "ba_skipped_in_flight",         v_ba_skipped_in_flight);         out += ',';
        appendKv(out, "ba_solve_us_sum",              v_ba_solve_us_sum);              out += ',';
        appendKv(out, "ba_solve_us_max",              v_ba_solve_us_max);              out += ',';
        appendKv(out, "ba_iters_sum",                 v_ba_iters_sum);                 out += ',';
        appendKv(out, "ba_skipped_no_init",           v_ba_skipped_no_init);           out += ',';
        appendKv(out, "ba_skipped_too_few_clones",    v_ba_skipped_too_few_clones);    out += ',';
        appendKv(out, "ba_skipped_no_intrinsics",     v_ba_skipped_no_intrinsics);     out += ',';
        appendKv(out, "ba_skipped_too_few_landmarks", v_ba_skipped_too_few_landmarks); out += ',';
        appendKv(out, "slam_promotions_total",        v_slam_promotions_total);        out += ',';
        appendKv(out, "slam_lifetime_obs_sum",         v_slam_lifetime_obs_sum);         out += ',';
        appendKv(out, "slam_lifetime_count",           v_slam_lifetime_count);           out += ',';
        appendKv(out, "slam_reanchor_total",           v_slam_reanchor_total);           out += ',';
        appendKv(out, "slam_reanchor_failed_no_clone", v_slam_reanchor_failed_no_clone); out += ',';
        appendKv(out, "slam_reanchor_failed_geometry", v_slam_reanchor_failed_geometry); out += ',';
        appendKv(out, "msckf_update_lines",            v_msckf_update_lines);            out += ',';
        appendKv(out, "msckf_huber_rejected_sum",      v_msckf_huber_rejected_sum);      out += ',';
        appendKv(out, "applyMSCKFUpdate_inversion_failed", v_applyMSCKFUpdate_inversion_failed); out += ',';
        appendKv(out, "ba_singular_landmarks",          v_ba_singular_landmarks);          out += ',';
        appendKv(out, "propagate_skipped",              v_propagate_skipped);              out += ',';
        appendKv(out, "propagate_skipped_not_init",     v_propagate_skipped_not_init);     out += ',';
        appendKv(out, "propagate_skipped_dt_nonpositive", v_propagate_skipped_dt_nonpositive); out += ',';
        appendKv(out, "propagate_skipped_p_empty",      v_propagate_skipped_p_empty);      out += ',';
        appendKv(out, "msckf_chi2_rejected",            v_msckf_chi2_rejected);            out += ',';
        appendKv(out, "msckf_inversion_failed",         v_msckf_inversion_failed);         out += ',';
        appendKv(out, "velocity_clamped",               v_velocity_clamped);               out += ',';
        appendKv(out, "scale_chi2_rejected",            v_scale_chi2_rejected);            out += ',';
        appendKv(out, "time_offset_clamped",            v_time_offset_clamped);            out += ',';
        appendKv(out, "fej_clone_miss",                 v_fej_clone_miss);                 out += ',';
        appendKv(out, "gravity_alignment_rejected",     v_gravity_alignment_rejected);     out += ',';
        appendKv(out, "relative_rotation_rejected",     v_relative_rotation_rejected);     out += ',';
        appendKv(out, "loop_closure_rejects_heading_total", v_loop_closure_rejects_heading_total); out += ',';
        appendKv(out, "loop_closure_rejects_pnp_total", v_loop_closure_rejects_pnp_total); out += ',';
        appendKv(out, "loop_closure_attempts",         v_loop_closure_attempts);         out += ',';
        appendKv(out, "loop_closure_accepts",          v_loop_closure_accepts);          out += ',';
        appendKv(out, "loop_closure_rejects_low_score", v_loop_closure_rejects_low_score); out += ',';
        appendKv(out, "loop_closure_rejects_pnp",      v_loop_closure_rejects_pnp);      out += ',';
        appendKv(out, "loop_closure_rejects_heading",  v_loop_closure_rejects_heading);  out += ',';
        appendKv(out, "loop_closure_rejects_rot_sanity_total", v_loop_closure_rejects_rot_sanity_total); out += ',';
        appendKv(out, "visual_yaw_chi2_rejected_total", v_visual_yaw_chi2_rejected_total); out += ',';
        appendKv(out, "visual_relative_rotation_chi2_rejected_total", v_visual_relative_rotation_chi2_rejected_total); out += ',';
        appendKv(out, "visual_relative_rotation_gyro_mismatch_total", v_visual_relative_rotation_gyro_mismatch_total); out += ',';
        appendKv(out, "madgwick_visual_yaw_nudges_total", v_madgwick_visual_yaw_nudges_total); out += ',';
        appendKv(out, "loop_closure_kf_count_in_db",   v_loop_closure_kf_count_in_db);   out += ',';
        appendKv(out, "loop_closure_chi2_rejected",    v_loop_closure_chi2_rejected);    out += ',';
        appendKv(out, "loop_closure_corrections_applied", v_loop_closure_corrections_applied); out += ',';
        appendKv(out, "loop_closure_lower_rank_accepts",  v_loop_closure_lower_rank_accepts);  out += ',';
        appendKv(out, "loop_closure_spatial_candidates_total", v_loop_closure_spatial_candidates_total); out += ',';
        appendKv(out, "loop_closure_spatial_fallback_total",   v_loop_closure_spatial_fallback_total);   out += ',';
        appendKv(out, "cam_fps_mean_milli_hz",            v_cam_fps_mean_milli_hz);            out += ',';
        appendKv(out, "cam_fps_stdev_milli_hz",           v_cam_fps_stdev_milli_hz);           out += ',';
        appendKv(out, "cam_fps_window_count",             v_cam_fps_window_count);             out += ',';
        appendKv(out, "loop_closure_geom_attempts",                 v_loop_closure_geom_attempts);                 out += ',';
        appendKv(out, "loop_closure_geom_accepts",                  v_loop_closure_geom_accepts);                  out += ',';
        appendKv(out, "loop_closure_geom_rejects_no_position",      v_loop_closure_geom_rejects_no_position);      out += ',';
        appendKv(out, "loop_closure_geom_rejects_no_candidate",     v_loop_closure_geom_rejects_no_candidate);     out += ',';
        appendKv(out, "loop_closure_geom_rejects_few_inframe",      v_loop_closure_geom_rejects_few_inframe);      out += ',';
        appendKv(out, "loop_closure_geom_rejects_pnp",              v_loop_closure_geom_rejects_pnp);              out += ',';
        appendKv(out, "loop_closure_geom_rejects_descriptor",       v_loop_closure_geom_rejects_descriptor);       out += ',';
        appendKv(out, "loop_closure_geom_spatial_miss",            v_loop_closure_geom_spatial_miss);            out += ',';
        appendKv(out, "loop_closure_geom_hamming_miss",            v_loop_closure_geom_hamming_miss);            out += ',';
        appendKv(out, "loop_closure_geom_hamming_min_sum",         v_loop_closure_geom_hamming_min_sum);         out += ',';
        appendKv(out, "loop_closure_geom_hamming_min_count",       v_loop_closure_geom_hamming_min_count);       out += ',';
        appendKv(out, "total_path_dm",                  v_total_path_dm);                  out += ',';
        appendKv(out, "pose_graph_optimize_calls",        v_pose_graph_optimize_calls);        out += ',';
        appendKv(out, "pose_graph_iters_used_sum",        v_pose_graph_iters_used_sum);        out += ',';
        appendKv(out, "pose_graph_residual_norm_pre_mm",  v_pose_graph_residual_norm_pre_mm);  out += ',';
        appendKv(out, "pose_graph_residual_norm_post_mm", v_pose_graph_residual_norm_post_mm); out += ',';
        appendKv(out, "pose_graph_max_correction_mm",     v_pose_graph_max_correction_mm);     out += ',';
        appendKv(out, "pose_graph_max_correction_mrad",   v_pose_graph_max_correction_mrad);   out += ',';
        appendKv(out, "pose_graph_rejected_singular",     v_pose_graph_rejected_singular);     out += ',';
        appendKv(out, "pose_graph_loop_edges_added",      v_pose_graph_loop_edges_added);      out += ',';
        appendKv(out, "pose_graph_apply_calls",           v_pose_graph_apply_calls);           out += ',';
        appendKv(out, "extrinsics_rotation_angle_mdeg", v_extrinsics_rotation_angle_mdeg); out += ',';
        // PHASE_A_8B_REENABLE_2026_05_19
        appendKv(out, "rbc_runaway_total",              v_rbc_runaway_total);              out += ',';
        // PHASE_B_6_4_REENABLE_2026_05_19
        appendKv(out, "landmark_obs_runaway_total",     v_landmark_obs_runaway_total);     out += ',';
        appendKv(out, "cam_imu_time_offset_us",         v_cam_imu_time_offset_us);         out += ',';
        appendKv(out, "rolling_shutter_skew_ns",        v_rolling_shutter_skew_ns);        out += ',';
        appendKv(out, "ekf_init_deferred_madgwick_not_ready", v_ekf_init_deferred_madgwick_not_ready); out += ',';
        appendKv(out, "zrup_fired_total", v_zrup_fired_total); out += ',';
        appendKv(out, "zupt_rotinplace_fired", v_zupt_rotinplace_fired); out += ',';
        appendKv(out, "depth_flow_updates", v_depth_flow_updates); out += ',';
        appendKv(out, "depth_flow_skipped_few_pts", v_depth_flow_skipped_few_pts); out += ',';
        appendKv(out, "depth_flow_outlier_rejected", v_depth_flow_outlier_rejected); out += ',';
        appendKv(out, "depth_flow_total_mm", v_depth_flow_total_mm); out += ',';
        appendKv(out, "depth_flow_calib_updates", v_depth_flow_calib_updates); out += ',';
        appendKv(out, "midas_scale_k_milli", v_midas_scale_k_milli); out += ',';
        appendKv(out, "midas_scale_k_min_milli", v_midas_scale_k_min_milli); out += ',';
        appendKv(out, "midas_scale_k_max_milli", v_midas_scale_k_max_milli); out += ',';
        appendKv(out, "depth_flow_looming_updates", v_depth_flow_looming_updates); out += ',';
        appendKv(out, "depth_flow_looming_used", v_depth_flow_looming_used); out += ',';
        appendKv(out, "depth_flow_looming_skipped", v_depth_flow_looming_skipped); out += ',';
        appendKv(out, "ekf_bg_absorbed_total", v_ekf_bg_absorbed_total); out += ',';
        appendKv(out, "gyro_bias_pushed_total", v_gyro_bias_pushed_total); out += ',';
        appendKv(out, "visual_yaw_gated_pure_rotation_total", v_visual_yaw_gated_pure_rotation_total); out += ',';
        appendKv(out, "msckf_gated_static_total", v_msckf_gated_static_total); out += ',';
        appendKv(out, "slam_update_gated_static_total", v_slam_update_gated_static_total); out += ',';
        appendKv(out, "landmarks_added_total",     v_landmarks_added_total);     out += ',';
        appendKv(out, "landmarks_merged_total",    v_landmarks_merged_total);    out += ',';
        appendKv(out, "landmarks_culled_total",    v_landmarks_culled_total);    out += ',';
        appendKv(out, "landmarks_skipped_invalid", v_landmarks_skipped_invalid); out += ',';
        appendKv(out, "landmark_map_size",         v_landmark_map_size);         out += ',';
        appendKv(out, "landmarks_rendered_anchor_total",      v_landmarks_rendered_anchor_total);      out += ',';
        appendKv(out, "landmarks_rendered_world_fixed_total", v_landmarks_rendered_world_fixed_total); out += ',';
        appendKv(out, "landmarks_in_ba_solve_sum",      v_landmarks_in_ba_solve_sum);      out += ',';
        appendKv(out, "landmarks_refined_total",        v_landmarks_refined_total);        out += ',';
        appendKv(out, "landmarks_reanchored_total",     v_landmarks_reanchored_total);     out += ',';
        appendKv(out, "landmarks_pixel_refreshed_total", v_landmarks_pixel_refreshed_total); out += ',';
        appendKv(out, "landmarks_descriptor_refreshed_total", v_landmarks_descriptor_refreshed_total); out += ',';
        appendKv(out, "landmark_obs_in_ba_history_min", v_landmark_obs_in_ba_history_min); out += ',';
        appendKv(out, "slam_live_updates_fired",        v_slam_live_updates_fired);        out += ',';
        appendKv(out, "slam_live_updates_skipped",      v_slam_live_updates_skipped);      out += ',';
        appendKv(out, "slam_live_skipped_no_parallax",  v_slam_live_skipped_no_parallax);  out += ',';
        appendKv(out, "slam_live_batch_calls",          v_slam_live_batch_calls);          out += ',';
        appendKv(out, "visual_translation_degenerate_total", v_visual_translation_degenerate_total); out += ',';
        appendKv(out, "landmarks_observed_total",  v_landmarks_observed_total);  out += ',';
        appendKv(out, "landmarks_matched_total",   v_landmarks_matched_total);   out += ',';
        appendKv(out, "landmarks_msckf_accepted_total",       v_landmarks_msckf_accepted_total);       out += ',';
        appendKv(out, "landmarks_msckf_rejected_chi2_total",  v_landmarks_msckf_rejected_chi2_total);  out += ',';
        appendKv(out, "landmarks_msckf_rejected_huber_total", v_landmarks_msckf_rejected_huber_total); out += ',';
        appendKv(out, "landmarks_msckf_rejected_depth_total", v_landmarks_msckf_rejected_depth_total); out += ',';
        appendKv(out, "landmarks_msckf_rejected_image_total", v_landmarks_msckf_rejected_image_total); out += ',';
        // 2026-05-16 single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
        appendKv(out, "overlay_snapshots_taken_total",        v_overlay_snapshots_taken_total);        out += ',';
        appendKv(out, "midas_entries",                       v_midas_entries);                       out += ',';
        appendKv(out, "midas_bailout_no_depth",              v_midas_bailout_no_depth);              out += ',';
        appendKv(out, "midas_bailout_few_pts3d",             v_midas_bailout_few_pts3d);             out += ',';
        appendKv(out, "midas_bailout_invalid_intrinsics",    v_midas_bailout_invalid_intrinsics);    out += ',';
        appendKv(out, "midas_bailout_camera_h",              v_midas_bailout_camera_h);              out += ',';
        appendKv(out, "midas_bailout_low_g",                 v_midas_bailout_low_g);                 out += ',';
        appendKv(out, "midas_bailout_few_floor_matches",     v_midas_bailout_few_floor_matches);     out += ',';
        appendKv(out, "midas_rejected_extreme",              v_midas_rejected_extreme);              out += ',';
        appendKv(out, "midas_fused",                         v_midas_fused);                         out += ',';
        appendKv(out, "midas_skipped",                       v_midas_skipped);                       out += ',';
        appendKv(out, "midas_affine_fit_singular",           v_midas_affine_fit_singular);           out += ',';
        appendKv(out, "midas_affine_fit_low_inliers",        v_midas_affine_fit_low_inliers);        out += ',';
        appendKv(out, "midas_affine_fit_inlier_ratio_milli", v_midas_affine_fit_inlier_ratio_milli); out += ',';
        appendKv(out, "midas_affine_fit_shift_milli",        v_midas_affine_fit_shift_milli);        out += ',';
        appendKv(out, "midas_depth_samples",                 v_midas_depth_samples);                 out += ',';
        appendKv(out, "slam_promotions_seeded_with_midas",   v_slam_promotions_seeded_with_midas);   out += ',';
        appendKv(out, "slam_promo_candidates_total",         v_slam_promo_candidates_total);         out += ',';
        appendKv(out, "slam_promo_rejected_cap_total",       v_slam_promo_rejected_cap_total);       out += ',';
        appendKv(out, "slam_promo_rejected_no_obs_total",    v_slam_promo_rejected_no_obs_total);    out += ',';
        appendKv(out, "slam_promo_rejected_no_anchor_far_total", v_slam_promo_rejected_no_anchor_far_total); out += ',';
        appendKv(out, "slam_promo_rejected_baseline_total",  v_slam_promo_rejected_baseline_total);  out += ',';
        appendKv(out, "slam_promo_rejected_solve_total",     v_slam_promo_rejected_solve_total);     out += ',';
        appendKv(out, "slam_promo_rejected_chirality_total", v_slam_promo_rejected_chirality_total); out += ',';
        appendKv(out, "slam_promo_rejected_n_used_total",    v_slam_promo_rejected_n_used_total);    out += ',';
        appendKv(out, "slam_promo_rejected_rms_total",       v_slam_promo_rejected_rms_total);       out += ',';
        appendKv(out, "slam_promo_rejected_parallax_total",  v_slam_promo_rejected_parallax_total);  out += ',';
        appendKv(out, "slam_promo_rms_milli_p95",            v_slam_promo_rms_milli_p95);            out += ',';
        appendKv(out, "slam_promo_rms_sum_milli",            v_slam_promo_rms_sum_milli);            out += ',';
        appendKv(out, "slam_promo_rms_milli_max",            v_slam_promo_rms_milli_max);            out += ',';
        appendKv(out, "global_t_gated_rotation_dominated_total", v_global_t_gated_rotation_dominated_total); out += ',';
        appendKv(out, "translation_heading_projection_total",    v_translation_heading_projection_total);
        out += '}';
        return out;
    }

private:
    // Append `"key":value` to `out`. Values are int64-safe; std::to_string
    // handles negatives correctly even though our counters are
    // monotonically non-negative in practice.
    static void appendKv(std::string& out, const char* key, long long value) {
        out += '"';
        out += key;
        out += "\":";
        out += std::to_string(value);
    }
};

// Process-singleton accessor. Defined inline so the header is self-contained
// and we avoid a separate .cpp; the static local is initialised once with
// thread-safe magic-statics (C++11+) and lives for the lifetime of the
// process. The native lib is loaded once and stays loaded, so this is fine.
inline EventCounters& eventCounters() {
    static EventCounters g_counters;
    return g_counters;
}

}  // namespace navsight

#endif  // NAVSIGHT_EVENT_COUNTERS_H
