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
//     "rolling_shutter_skew_ns": N
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

    // Step 3a/3b — MSCKF (EKFState.cpp applyMSCKFUpdate)
    std::atomic<long long> msckf_update_lines{0};
    std::atomic<long long> msckf_huber_rejected_sum{0};

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
    std::atomic<long long> loop_closure_kf_count_in_db{0};
    // Step 7 absolute-pose injection (ADR-013 §"Correction injection —
    // absolute pose path"). Completes accounting from detection to
    // correction: chi²_rejected counts wildly-wrong matches caught by
    // the outer gate; corrections_applied counts every successful
    // updateAbsolutePose injection through the EKF.
    std::atomic<long long> loop_closure_chi2_rejected{0};
    std::atomic<long long> loop_closure_corrections_applied{0};
    // total_path_m × 10 (integer decimeters) — verifies the dynamic sigma
    // formula (sigma_p = max(2.0, 0.032 × path_m)) is tracking correctly.
    std::atomic<long long> total_path_dm{0};

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
        msckf_update_lines.store(0, std::memory_order_relaxed);
        msckf_huber_rejected_sum.store(0, std::memory_order_relaxed);
        loop_closure_attempts.store(0, std::memory_order_relaxed);
        loop_closure_accepts.store(0, std::memory_order_relaxed);
        loop_closure_rejects_low_score.store(0, std::memory_order_relaxed);
        loop_closure_rejects_pnp.store(0, std::memory_order_relaxed);
        loop_closure_rejects_heading.store(0, std::memory_order_relaxed);
        loop_closure_kf_count_in_db.store(0, std::memory_order_relaxed);
        loop_closure_chi2_rejected.store(0, std::memory_order_relaxed);
        loop_closure_corrections_applied.store(0, std::memory_order_relaxed);
        total_path_dm.store(0, std::memory_order_relaxed);
        extrinsics_rotation_angle_mdeg.store(0, std::memory_order_relaxed);
        cam_imu_time_offset_us.store(0, std::memory_order_relaxed);
        rolling_shutter_skew_ns.store(0, std::memory_order_relaxed);
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
        const long long v_msckf_update_lines            = msckf_update_lines.load(std::memory_order_relaxed);
        const long long v_msckf_huber_rejected_sum      = msckf_huber_rejected_sum.load(std::memory_order_relaxed);
        const long long v_loop_closure_attempts         = loop_closure_attempts.load(std::memory_order_relaxed);
        const long long v_loop_closure_accepts          = loop_closure_accepts.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_low_score= loop_closure_rejects_low_score.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_pnp      = loop_closure_rejects_pnp.load(std::memory_order_relaxed);
        const long long v_loop_closure_rejects_heading  = loop_closure_rejects_heading.load(std::memory_order_relaxed);
        const long long v_loop_closure_kf_count_in_db   = loop_closure_kf_count_in_db.load(std::memory_order_relaxed);
        const long long v_loop_closure_chi2_rejected    = loop_closure_chi2_rejected.load(std::memory_order_relaxed);
        const long long v_loop_closure_corrections_applied = loop_closure_corrections_applied.load(std::memory_order_relaxed);
        const long long v_total_path_dm               = total_path_dm.load(std::memory_order_relaxed);
        const long long v_extrinsics_rotation_angle_mdeg = extrinsics_rotation_angle_mdeg.load(std::memory_order_relaxed);
        const long long v_cam_imu_time_offset_us  = cam_imu_time_offset_us.load(std::memory_order_relaxed);
        const long long v_rolling_shutter_skew_ns = rolling_shutter_skew_ns.load(std::memory_order_relaxed);

        std::string out;
        out.reserve(900);
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
        appendKv(out, "msckf_update_lines",            v_msckf_update_lines);            out += ',';
        appendKv(out, "msckf_huber_rejected_sum",      v_msckf_huber_rejected_sum);      out += ',';
        appendKv(out, "loop_closure_attempts",         v_loop_closure_attempts);         out += ',';
        appendKv(out, "loop_closure_accepts",          v_loop_closure_accepts);          out += ',';
        appendKv(out, "loop_closure_rejects_low_score", v_loop_closure_rejects_low_score); out += ',';
        appendKv(out, "loop_closure_rejects_pnp",      v_loop_closure_rejects_pnp);      out += ',';
        appendKv(out, "loop_closure_rejects_heading",  v_loop_closure_rejects_heading);  out += ',';
        appendKv(out, "loop_closure_kf_count_in_db",   v_loop_closure_kf_count_in_db);   out += ',';
        appendKv(out, "loop_closure_chi2_rejected",    v_loop_closure_chi2_rejected);    out += ',';
        appendKv(out, "loop_closure_corrections_applied", v_loop_closure_corrections_applied); out += ',';
        appendKv(out, "total_path_dm",                  v_total_path_dm);                  out += ',';
        appendKv(out, "extrinsics_rotation_angle_mdeg", v_extrinsics_rotation_angle_mdeg); out += ',';
        appendKv(out, "cam_imu_time_offset_us",         v_cam_imu_time_offset_us);         out += ',';
        appendKv(out, "rolling_shutter_skew_ns",        v_rolling_shutter_skew_ns);
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
