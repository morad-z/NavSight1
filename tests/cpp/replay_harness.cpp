// Step 7 — Replay Harness (extended for Step 9 visual coverage)
//
// Reads a NavSight simulation JSON (schema documented in
// scripts/analyze_sim.py and matched by the runtime sim recorder), drives the
// VIO pipeline at the recorded sample timestamps, and emits a CSV trace of
// EKF pose + horizontal-plane covariance + visual front-end stats per frame
// for downstream regression scoring (replay_scorer.py).
//
// Frame source (Plan Step 9 / ADR-014):
//   • IMU-only mode (default, backward-compatible with ADR-007): a synthetic
//     uniform 320×240 mid-grey YUV buffer drives processFrame so the EKF
//     propagates against IMU only. The visual front-end finds nothing in a
//     flat frame, so MSCKF/SLAM/loop-closure paths are all dormant.
//   • Recorded-frames mode: pass `--frames-dir <path>` (or place a sibling
//     directory named `frames/` next to the simulation JSON). The harness
//     loads `<ts_ns>.png` for each camera tick, packs it into NV21 (Y plane
//     = greyscale, UV plane = mid-grey), and feeds processFrame the real
//     pixels. The Step 7/8 visual stack runs end-to-end on a deterministic
//     fixture, replacing what previously could only be exercised on a phone.
//
// CLI:
//     replay_harness <sim_input.json> <pose_output.csv>
//                    [--frames-dir <path>]
//                    [--frame-match-tolerance-ms <N>]
//
//   --frame-match-tolerance-ms <N>   default 0 (exact-name match). When > 0,
//                                    look up each sample's <ts_ns>.png; if
//                                    missing, scan frames_dir for the nearest
//                                    neighbour and accept if within N ms. Use
//                                    this for fixtures recorded by the runtime
//                                    SimulationFrameRecorder (Step 9 / ADR-014),
//                                    where the camera-thread frame timestamp
//                                    and the VIO-thread sample timestamp are a
//                                    few ms apart. Hand-curated fixtures whose
//                                    filenames already match the JSON `ts`
//                                    exactly should keep the default 0.
//
// Exit codes: 0 success, non-zero failure.
//
// CSV columns (extended in Step 9 — column order is stable, append-only):
//     ts_ns, ekf_init, ekf_x, ekf_y, ekf_z, ekf_yaw_rad,
//     sigma_xx, sigma_xz, sigma_zz,
//     recorded_vx, recorded_vy, recorded_vz, recorded_vyaw_rad,
//     glat, glng, gacc_m,
//     inlier_count, tracked_count, total_count, mean_flow,
//     pose_flags, vision_valid, frame_loaded, keyframe_stored
//
// Sidecar JSON:
//     <pose_output.csv>.event_counters.json — final EventCounters snapshot.
//     Used by replay_scorer.py to surface event-level metrics
//     (loop_closure_corrections_applied, msckf_huber_rejected_sum, etc.).
//
// Determinism:
//   • Samples are sorted ascending by ts before streaming.
//   • IMU is fed strictly in timestamp order.
//   • Synthetic-frame fill is constant across runs.
//   • Recorded frames are looked up by exact `<ts_ns>.png` filename — no
//     interpolation, no resampling.

#include "VioEngine.h"
#include "EventCounters.h"
#include "EKFState.h"  // 2026-05-23 yaw-variance probe for the map-anchor noise fix

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

// IMU-only fallback dimensions. When recorded frames are present, the
// harness uses each loaded frame's actual size (no resize), so these only
// gate the synthetic-grey path.
constexpr int kSyntheticWidth  = 320;
constexpr int kSyntheticHeight = 240;
constexpr uint8_t kSyntheticFill = 64;  // mid-grey, away from clipping
constexpr uint8_t kUvNeutralFill = 128; // U=V=128 → no chroma when re-decoded

struct SimSample {
    int64_t ts_ns;
    float ax, ay, az;
    float gx, gy, gz;
    float vx, vy, vz;
    float vyaw;
    bool   has_gps;
    double glat, glng;
    float  gacc_m;
};

bool jsonNumber(const nlohmann::json& v, double& out) {
    if (v.is_null()) return false;
    if (!v.is_number()) return false;
    out = v.get<double>();
    return true;
}

bool readJsonFile(const std::string& path, nlohmann::json& out) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "replay_harness: cannot open %s\n", path.c_str());
        return false;
    }
    try {
        in >> out;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "replay_harness: parse error: %s\n", e.what());
        return false;
    }
    return true;
}

bool loadSim(const nlohmann::json& j, std::vector<SimSample>& out) {
    if (!j.contains("points") || !j["points"].is_array()) {
        std::fprintf(stderr, "replay_harness: missing or non-array 'points'\n");
        return false;
    }

    const auto& pts = j["points"];
    out.reserve(pts.size());
    for (const auto& p : pts) {
        SimSample s{};
        // Required scalar fields. Missing/null IMU disqualifies the sample.
        double ts_ms = 0.0;
        if (!p.contains("ts") || !jsonNumber(p["ts"], ts_ms)) continue;
        s.ts_ns = static_cast<int64_t>(ts_ms * 1e6);

        auto getf = [&](const char* k, float& f) -> bool {
            if (!p.contains(k)) return false;
            double tmp = 0.0;
            if (!jsonNumber(p[k], tmp)) return false;
            f = static_cast<float>(tmp);
            return true;
        };
        if (!getf("ax", s.ax) || !getf("ay", s.ay) || !getf("az", s.az)) continue;
        if (!getf("gx", s.gx) || !getf("gy", s.gy) || !getf("gz", s.gz)) continue;

        getf("vx", s.vx);
        getf("vy", s.vy);
        getf("vz", s.vz);
        getf("vyaw", s.vyaw);

        s.has_gps = false;
        if (p.contains("glat") && p.contains("glng") && !p["glat"].is_null() && !p["glng"].is_null()) {
            double lat = 0.0, lng = 0.0, acc = 0.0;
            if (jsonNumber(p["glat"], lat) && jsonNumber(p["glng"], lng)) {
                s.glat = lat;
                s.glng = lng;
                s.has_gps = true;
                if (p.contains("gacc") && !p["gacc"].is_null() && jsonNumber(p["gacc"], acc)) {
                    s.gacc_m = static_cast<float>(acc);
                }
            }
        }
        out.push_back(s);
    }
    std::sort(out.begin(), out.end(),
              [](const SimSample& a, const SimSample& b) { return a.ts_ns < b.ts_ns; });
    return true;
}

// Pack a greyscale image into the NV21 layout `processFrame` expects:
// [Y plane: H × W bytes] + [VU plane: H/2 × W bytes interleaved V, U]. With
// U=V=128 the decoded chroma is neutral, so the runtime grayscale-via-NV21
// path returns exactly the input Y plane.
void greyToNv21(const cv::Mat& grey, std::vector<uint8_t>& nv21) {
    const int W = grey.cols;
    const int H = grey.rows;
    nv21.assign(static_cast<size_t>(W) * H * 3 / 2, kUvNeutralFill);
    if (grey.isContinuous()) {
        std::memcpy(nv21.data(), grey.data, static_cast<size_t>(W) * H);
    } else {
        for (int y = 0; y < H; ++y) {
            std::memcpy(nv21.data() + static_cast<size_t>(y) * W,
                        grey.ptr<uint8_t>(y), static_cast<size_t>(W));
        }
    }
}

// Locate the frames directory:
//   1. Explicit --frames-dir wins.
//   2. Otherwise honour `frames_meta.dir` from the JSON (set by the runtime
//      SimulationFrameRecorder per ADR-014). Resolved relative to the JSON.
//   3. Otherwise probe a sibling `frames/` directory next to the JSON.
//   4. Empty string ⇒ IMU-only synthetic mode.
std::string resolveFramesDir(const std::string& sim_path,
                             const std::string& cli_override,
                             const nlohmann::json& sim_json) {
    namespace fs = std::filesystem;
    if (!cli_override.empty()) {
        if (!fs::is_directory(cli_override)) {
            std::fprintf(stderr,
                "replay_harness: --frames-dir %s is not a directory\n",
                cli_override.c_str());
            return {};  // hard fail upstream via empty + std::exit handled by caller
        }
        return cli_override;
    }
    fs::path sim(sim_path);
    if (sim_json.contains("frames_meta") && sim_json["frames_meta"].is_object()) {
        const auto& meta = sim_json["frames_meta"];
        if (meta.contains("dir") && meta["dir"].is_string()) {
            fs::path declared = sim.parent_path() / meta["dir"].get<std::string>();
            if (fs::is_directory(declared)) return declared.string();
            std::fprintf(stderr,
                "replay_harness: frames_meta.dir=%s does not resolve under %s\n",
                meta["dir"].get<std::string>().c_str(),
                sim.parent_path().string().c_str());
        }
    }
    fs::path sibling = sim.parent_path() / "frames";
    if (fs::is_directory(sibling)) return sibling.string();
    return {};
}

// Index of every <int64>.png file in a frames_dir, sorted ascending by the
// integer encoded in the filename. Built once per harness run and used for
// nearest-neighbour timestamp matching when --frame-match-tolerance-ms > 0.
struct FramesIndex {
    std::vector<int64_t> ts_ns;
    std::vector<std::filesystem::path> paths;
};

FramesIndex indexFramesDir(const std::string& frames_dir) {
    FramesIndex idx;
    if (frames_dir.empty()) return idx;
    namespace fs = std::filesystem;
    for (const auto& entry : fs::directory_iterator(frames_dir)) {
        if (!entry.is_regular_file()) continue;
        const auto& path = entry.path();
        if (path.extension() != ".png") continue;
        const std::string stem = path.stem().string();
        // strict integer parse — silently skip filenames that aren't pure
        // <int64>.png (e.g. README.png).
        if (stem.empty()) continue;
        size_t pos = 0;
        int64_t ts = 0;
        try {
            ts = std::stoll(stem, &pos);
        } catch (...) { continue; }
        if (pos != stem.size()) continue;
        idx.ts_ns.push_back(ts);
        idx.paths.push_back(path);
    }
    // co-sort by ts_ns
    std::vector<size_t> order(idx.ts_ns.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) { return idx.ts_ns[a] < idx.ts_ns[b]; });
    std::vector<int64_t> sorted_ts; sorted_ts.reserve(order.size());
    std::vector<std::filesystem::path> sorted_paths; sorted_paths.reserve(order.size());
    for (size_t i : order) {
        sorted_ts.push_back(idx.ts_ns[i]);
        sorted_paths.push_back(std::move(idx.paths[i]));
    }
    idx.ts_ns = std::move(sorted_ts);
    idx.paths = std::move(sorted_paths);
    return idx;
}

// Resolve a sample timestamp to a frame path. Strategy:
//   1. Try exact `<ts_ns>.png` (fast filesystem stat).
//   2. If not found AND tolerance_ns > 0, binary-search the index for the
//      nearest neighbour and accept if within tolerance.
// Returns the matching path on success, empty path otherwise.
std::filesystem::path resolveFramePath(const std::string& frames_dir,
                                        const FramesIndex& idx,
                                        int64_t ts_ns,
                                        int64_t tolerance_ns) {
    namespace fs = std::filesystem;
    if (frames_dir.empty()) return {};
    fs::path exact = fs::path(frames_dir) / (std::to_string(ts_ns) + ".png");
    if (fs::exists(exact)) return exact;
    if (tolerance_ns <= 0 || idx.ts_ns.empty()) return {};
    auto it = std::lower_bound(idx.ts_ns.begin(), idx.ts_ns.end(), ts_ns);
    int64_t best_dt = INT64_MAX;
    size_t best_i = idx.ts_ns.size();
    if (it != idx.ts_ns.end()) {
        size_t i = static_cast<size_t>(it - idx.ts_ns.begin());
        int64_t dt = idx.ts_ns[i] - ts_ns;
        if (dt < best_dt) { best_dt = dt; best_i = i; }
    }
    if (it != idx.ts_ns.begin()) {
        size_t i = static_cast<size_t>(it - idx.ts_ns.begin()) - 1;
        int64_t dt = ts_ns - idx.ts_ns[i];
        if (dt < best_dt) { best_dt = dt; best_i = i; }
    }
    if (best_i >= idx.ts_ns.size()) return {};
    if (best_dt > tolerance_ns) return {};
    return idx.paths[best_i];
}

// Decode an 8-bit greyscale Mat from the resolved frame path. Empty path or
// decode failure ⇒ false (caller falls back to synthetic). Decode failures
// are reported but non-fatal — we keep the run going so a few corrupt frames
// don't tank a CI fixture.
bool tryLoadFrame(const std::string& frames_dir,
                  const FramesIndex& idx,
                  int64_t ts_ns,
                  int64_t tolerance_ns,
                  cv::Mat& out,
                  std::filesystem::path& matched_out) {
    auto p = resolveFramePath(frames_dir, idx, ts_ns, tolerance_ns);
    if (p.empty()) return false;
    matched_out = p;  // 2026-05-29 — expose the matched frame so the caller can
                      // locate its MiDaS depth raster (<stem>.f32) for --depth-dir.
    cv::Mat raw = cv::imread(p.string(), cv::IMREAD_UNCHANGED);
    if (raw.empty()) {
        std::fprintf(stderr,
            "replay_harness: cv::imread failed for %s (skipping frame)\n",
            p.string().c_str());
        return false;
    }
    if (raw.channels() == 1) {
        out = raw;
    } else if (raw.channels() == 3) {
        cv::cvtColor(raw, out, cv::COLOR_BGR2GRAY);
    } else if (raw.channels() == 4) {
        cv::cvtColor(raw, out, cv::COLOR_BGRA2GRAY);
    } else {
        std::fprintf(stderr,
            "replay_harness: unsupported channel count %d in %s\n",
            raw.channels(), p.string().c_str());
        return false;
    }
    if (out.depth() != CV_8U) {
        cv::Mat tmp;
        out.convertTo(tmp, CV_8U);
        out = tmp;
    }
    return true;
}

// 2026-05-29 — Load a MiDaS raw-disparity raster (<frame_stem>.f32, row-major
// float32, 256x256 per DepthEstimator.kt) produced by scripts/gen_midas_depth.py.
// The on-device pipeline computes this in Kotlin and pushes it via
// VioEngine::setDepthMap; the harness historically never fed it, so
// updateDepthFlowSpeed/updateExpansionSpeed (the depth-flow SPEED path) were
// dead in replay. Feeding it makes the speed/scale path testable offline.
// Returns the loaded floats + side length on success.
bool tryLoadDepthRaster(const std::string& depth_dir,
                        const std::filesystem::path& frame_path,
                        std::vector<float>& out, int& side) {
    namespace fs = std::filesystem;
    if (depth_dir.empty() || frame_path.empty()) return false;
    fs::path dp = fs::path(depth_dir) / (frame_path.stem().string() + ".f32");
    std::error_code ec;
    auto bytes = fs::file_size(dp, ec);
    if (ec || bytes == 0 || bytes % sizeof(float) != 0) return false;
    const size_t n = bytes / sizeof(float);
    // Expect a square raster (256x256). Reject anything else loudly rather than
    // silently feeding a mis-shaped map.
    int s = static_cast<int>(std::lround(std::sqrt(static_cast<double>(n))));
    if (static_cast<size_t>(s) * s != n) {
        std::fprintf(stderr, "replay_harness: depth raster %s is not square (n=%zu)\n",
                     dp.string().c_str(), n);
        return false;
    }
    std::ifstream in(dp, std::ios::binary);
    if (!in) return false;
    out.resize(n);
    in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(bytes));
    if (!in) return false;
    side = s;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    // Parse positional + flag args. Keeping it hand-rolled avoids pulling in
    // a CLI library for two flags.
    std::string sim_path;
    std::string csv_path;
    std::string frames_dir_override;
    std::string depth_dir;  // 2026-05-29 — MiDaS depth rasters (--depth-dir)
    double seed_k = 0.0;    // 2026-05-29 — warm-start K seed (--seed-k); 0 = cold
    double camera_height = 0.0;  // 2026-06-02 — scooter mount camera-to-road height (--camera-height); 0 = off
    int64_t frame_tolerance_ns = 0;
    bool autonomous_pg = false;  // 2026-05-31 — --autonomous-pg (verdict §4):
                                 // skip ekf_.setPosition(global_t_) so p_G_
                                 // propagates autonomously. Default OFF.
    {
        std::vector<std::string> positional;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--frames-dir") {
                if (i + 1 >= argc) {
                    std::fprintf(stderr,
                        "replay_harness: --frames-dir requires a path argument\n");
                    return 2;
                }
                frames_dir_override = argv[++i];
            } else if (a == "--depth-dir") {
                if (i + 1 >= argc) {
                    std::fprintf(stderr,
                        "replay_harness: --depth-dir requires a path argument\n");
                    return 2;
                }
                depth_dir = argv[++i];
            } else if (a == "--seed-k") {
                // 2026-05-29 — seed midas_scale_K_/expansion_scale_K_ to emulate
                // the on-device WARM start (K persisted in SharedPreferences).
                // Lets us test whether the looming speed path carries the dot
                // when K is pre-calibrated, isolating the cold-start K-deadlock.
                if (i + 1 >= argc) {
                    std::fprintf(stderr, "replay_harness: --seed-k requires a value\n");
                    return 2;
                }
                try { seed_k = std::stod(argv[++i]); }
                catch (const std::exception& e) {
                    std::fprintf(stderr, "replay_harness: --seed-k parse error: %s\n", e.what());
                    return 2;
                }
            } else if (a == "--camera-height") {
                // 2026-06-02 — measured scooter camera-to-road height (m) for the read-only ground-plane
                // metric-scale eval (gpt_speed_suggestion.md Phase 0). Drives Tracker::setCameraHeight.
                if (i + 1 >= argc) {
                    std::fprintf(stderr, "replay_harness: --camera-height requires a value\n");
                    return 2;
                }
                try { camera_height = std::stod(argv[++i]); }
                catch (const std::exception& e) {
                    std::fprintf(stderr, "replay_harness: --camera-height parse error: %s\n", e.what());
                    return 2;
                }
            } else if (a == "--autonomous-pg") {
                // 2026-05-31 — verdict §4 safe A/B: skip the per-frame
                // ekf_.setPosition(global_t_) overwrite so the EKF p_G_ runs
                // autonomously (MSCKF/ZUPT updates survive). No value arg.
                autonomous_pg = true;
            } else if (a == "--frame-match-tolerance-ms") {
                if (i + 1 >= argc) {
                    std::fprintf(stderr,
                        "replay_harness: --frame-match-tolerance-ms requires a value\n");
                    return 2;
                }
                try {
                    long long ms = std::stoll(argv[++i]);
                    if (ms < 0) throw std::out_of_range("negative");
                    frame_tolerance_ns = ms * 1'000'000LL;
                } catch (const std::exception& e) {
                    std::fprintf(stderr,
                        "replay_harness: --frame-match-tolerance-ms parse error: %s\n",
                        e.what());
                    return 2;
                }
            } else {
                positional.push_back(a);
            }
        }
        if (positional.size() != 2) {
            std::fprintf(stderr,
                "Usage: %s <sim_input.json> <pose_output.csv> "
                "[--frames-dir <path>] [--depth-dir <path>] [--seed-k <v>] "
                "[--frame-match-tolerance-ms <N>] [--autonomous-pg]\n",
                (argc > 0 ? argv[0] : "replay_harness"));
            return 2;
        }
        sim_path = positional[0];
        csv_path = positional[1];
    }

    nlohmann::json sim_json;
    if (!readJsonFile(sim_path, sim_json)) return 3;

    std::vector<SimSample> samples;
    if (!loadSim(sim_json, samples)) return 3;
    if (samples.empty()) {
        std::fprintf(stderr, "replay_harness: no usable samples in %s\n", sim_path.c_str());
        return 4;
    }

    std::string frames_dir = resolveFramesDir(sim_path, frames_dir_override, sim_json);
    if (!frames_dir_override.empty() && frames_dir.empty()) {
        // Explicit override that didn't resolve (resolveFramesDir already
        // logged the reason). Fail loudly rather than silently degrading
        // to synthetic; CI shouldn't accept that quietly.
        return 5;
    }
    const bool have_frames = !frames_dir.empty();
    std::fprintf(stdout,
        "replay_harness: %s mode (%s)\n",
        have_frames ? "recorded-frames" : "IMU-only synthetic",
        have_frames ? frames_dir.c_str() : "no frames/ dir found");
    // 2026-05-29 — depth-dir validation. Requires frames (depth is keyed by the
    // matched frame's stem). Fail loudly if the dir is bogus rather than silently
    // running depth-starved (which is exactly the artifact we are fixing).
    if (!depth_dir.empty()) {
        if (!have_frames) {
            std::fprintf(stderr,
                "replay_harness: --depth-dir requires --frames-dir / a frames/ dir\n");
            return 5;
        }
        if (!std::filesystem::is_directory(depth_dir)) {
            std::fprintf(stderr,
                "replay_harness: --depth-dir %s is not a directory\n",
                depth_dir.c_str());
            return 5;
        }
        std::fprintf(stdout, "replay_harness: MiDaS depth mode (%s)\n",
                     depth_dir.c_str());
    }
    FramesIndex frames_index;
    if (have_frames && frame_tolerance_ns > 0) {
        frames_index = indexFramesDir(frames_dir);
        std::fprintf(stdout,
            "replay_harness: indexed %zu frames; tolerance %lld ms\n",
            frames_index.ts_ns.size(),
            (long long)(frame_tolerance_ns / 1'000'000LL));
    }

    std::ofstream csv(csv_path);
    if (!csv) {
        std::fprintf(stderr, "replay_harness: cannot open %s for write\n", csv_path.c_str());
        return 6;
    }
    csv.setf(std::ios::fixed);
    csv.precision(9);
    csv << "ts_ns,ekf_init,ekf_x,ekf_y,ekf_z,ekf_yaw_rad,"
        << "sigma_xx,sigma_xz,sigma_zz,"
        << "recorded_vx,recorded_vy,recorded_vz,recorded_vyaw_rad,"
        << "glat,glng,gacc_m,"
        << "inlier_count,tracked_count,total_count,mean_flow,"
        << "pose_flags,vision_valid,frame_loaded,keyframe_stored,vout_heading_rad,ekf_yaw_var_rad2,bias_yaw_var,cross_yaw,"
        // 2026-05-29 — the USER-FACING dot trajectory (Tracker::global_t_,
        // exposed as VisionOutput.t at Tracker.cpp:6231) and the UI speedometer
        // value (Tracker::getFusedSpeedMps). getPose()/ekf_x..z above is EKF
        // p_G_ — a DIFFERENT trajectory that the depth-flow speed work does NOT
        // drive. To score what the user actually sees on the map (and what the
        // velocity work changes), the scorer must read traj_x/traj_z, not ekf_*.
        << "traj_x,traj_y,traj_z,fused_speed_mps,gp_flow_speed_mps,"
        << "gp_valid,gp_scale,gp_conf,gp_hvio,gp_cand,gp_inl,gp_horizon_v\n";

    VioEngine engine;
    // Synthetic intrinsics (used as defaults when no frames are present).
    engine.setIntrinsics(static_cast<double>(kSyntheticWidth),
                         static_cast<double>(kSyntheticWidth),
                         kSyntheticWidth * 0.5,
                         kSyntheticHeight * 0.5);

    // 2026-05-20 — Load the ORB DBoW2 vocabulary so the Step 7 loop-closure
    // detector + all keyframe-time ORB descriptor / landmark / local-map
    // pipelines that gate on `loop_closure_.isReady()` actually fire.
    // Without this, Tracker.cpp:3703 skips the entire keyframe ORB block,
    // landmarks_added_total stays at 0, landmarks_matched_total stays at 0,
    // loop closure never fires, and Fix #9/#11/#12 have nothing to operate
    // on.
    if (!frames_dir.empty()) {
        const std::filesystem::path vocab_path = std::filesystem::path(
            __FILE__).parent_path() / "desktop_stubs" / "ORBvoc.txt";
        if (std::filesystem::exists(vocab_path)) {
            const bool ok = engine.loadLoopClosureVocabulary(vocab_path.string());
            std::cerr << "replay_harness: loaded ORBvoc.txt (145 MB), ok="
                      << (ok ? "true" : "false") << "\n";
        } else {
            std::cerr << "replay_harness: ORBvoc.txt not found at "
                      << vocab_path << " — landmark+LC counters will stay 0\n";
        }
    }

    // 2026-05-20 — Critical: shortcut the InertialInitializer's
    // WAIT_STATIONARY gate via loadStoredCalibration. Without this,
    // Tracker::processFrame returns early at line 1053 (!initialized_),
    // so out.valid stays at default false, so EKF never gets visual
    // updates, so vision_valid=0 / all counters=0 across the replay.
    //
    // R_GtoI derived from the first IMU sample's accel direction.
    // Stationary or near-stationary phone reports specific_force ≈ -g
    // in body frame. Body-frame "up" direction = a / |a|. That vector
    // IS column 2 of R_GtoI (world Z expressed in body). Construct
    // the orthonormal frame via Gram-Schmidt against world X for
    // columns 0 and 1 (yaw=0 assumption — the magnetometer would set
    // the real yaw, but we don't have it; the first visual update
    // will refine).
    //
    // This replaces the earlier "identity R_GtoI" stub which left the
    // EKF thinking the phone was lying flat with screen up, causing
    // every SLAM promotion's reprojection RMS to fail the 1.5 px gate
    // because projected SLAM features landed in the wrong half of the
    // image plane.
    size_t kMadgwickWarmup = 0;  // Set inside the block below; used by main loop to skip already-fed IMU samples.
    if (!samples.empty()) {
        const auto& s0 = samples.front();
        const double ax = s0.ax, ay = s0.ay, az = s0.az;
        const double a_norm = std::sqrt(ax * ax + ay * ay + az * az);
        float R_GtoI[9] = {1, 0, 0,  0, 1, 0,  0, 0, 1};  // identity fallback
        if (a_norm > 1e-6 && std::isfinite(a_norm)) {
            // Column 2 = body-frame up (= specific force direction).
            const double c2x = ax / a_norm;
            const double c2y = ay / a_norm;
            const double c2z = az / a_norm;
            // Gram-Schmidt: pick world X = (1,0,0), project onto plane
            // perpendicular to col2, normalize → col0.
            const double dot_x = c2x;  // (1,0,0) · col2 = c2x
            double c0x = 1.0 - dot_x * c2x;
            double c0y = -dot_x * c2y;
            double c0z = -dot_x * c2z;
            double c0n = std::sqrt(c0x * c0x + c0y * c0y + c0z * c0z);
            if (c0n < 1e-3) {
                // World X is nearly parallel to up — use world Y instead.
                c0x = -c2y * c2x;
                c0y = 1.0 - c2y * c2y;
                c0z = -c2y * c2z;
                c0n = std::sqrt(c0x * c0x + c0y * c0y + c0z * c0z);
            }
            c0x /= c0n; c0y /= c0n; c0z /= c0n;
            // col1 = col2 × col0
            const double c1x = c2y * c0z - c2z * c0y;
            const double c1y = c2z * c0x - c2x * c0z;
            const double c1z = c2x * c0y - c2y * c0x;
            R_GtoI[0] = static_cast<float>(c0x); R_GtoI[1] = static_cast<float>(c1x); R_GtoI[2] = static_cast<float>(c2x);
            R_GtoI[3] = static_cast<float>(c0y); R_GtoI[4] = static_cast<float>(c1y); R_GtoI[5] = static_cast<float>(c2y);
            R_GtoI[6] = static_cast<float>(c0z); R_GtoI[7] = static_cast<float>(c1z); R_GtoI[8] = static_cast<float>(c2z);
            std::cerr << "replay_harness: R_GtoI derived from first IMU sample, "
                      << "up_body=(" << c2x << "," << c2y << "," << c2z << ") "
                      << "|a|=" << a_norm << "\n";
        } else {
            std::cerr << "replay_harness: first IMU sample has |a|=0, "
                      << "falling back to identity R_GtoI\n";
        }
        const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
        // 2026-05-20 — Prefer the saved on-device calibration over the
        // sample[0]-derived seed. The runtime app saves R_GtoI +
        // gyro_bias + accel_bias on first-run InertialInitializer
        // success into shared_prefs/navsight_calibration.xml and
        // reads it on every subsequent app launch. Without these,
        // the EKF starts with zero biases → IMU integration drifts
        // a few °/s per walk → MSCKF chi² rejects ~50% of updates →
        // EKF state diverges → SLAM promotion's two-view
        // triangulation produces 60-160 px reprojection error →
        // all candidates rejected by the 1.5 px RMS gate.
        //
        // The XML was pulled from device via:
        //   adb shell "run-as com.example.navsight1 cat \
        //     /data/user/0/com.example.navsight1/shared_prefs/navsight_calibration.xml"
        // saved at tests/cpp/desktop_stubs/navsight_calibration.xml
        float R_GtoI_use[9] = {
            R_GtoI[0], R_GtoI[1], R_GtoI[2],
            R_GtoI[3], R_GtoI[4], R_GtoI[5],
            R_GtoI[6], R_GtoI[7], R_GtoI[8],
        };
        float gyro_bias_use[3] = {0.0f, 0.0f, 0.0f};
        float accel_bias_use[3] = {0.0f, 0.0f, 0.0f};
        bool loaded_xml = false;
        {
            const std::filesystem::path calib_xml = std::filesystem::path(
                __FILE__).parent_path() / "desktop_stubs" / "navsight_calibration.xml";
            std::ifstream xml_in(calib_xml);
            if (xml_in) {
                std::string xml((std::istreambuf_iterator<char>(xml_in)),
                                std::istreambuf_iterator<char>());
                // Quick-and-dirty XML extraction. Only need three known
                // <string name="X">comma,sep,values</string> entries.
                auto extract = [&](const std::string& key,
                                    std::vector<float>& out) -> bool {
                    const std::string marker = "name=\"" + key + "\">";
                    auto p = xml.find(marker);
                    if (p == std::string::npos) return false;
                    p += marker.size();
                    auto end = xml.find("<", p);
                    if (end == std::string::npos) return false;
                    std::string vals = xml.substr(p, end - p);
                    out.clear();
                    size_t i = 0;
                    while (i < vals.size()) {
                        size_t c = vals.find(',', i);
                        std::string tok = (c == std::string::npos)
                            ? vals.substr(i) : vals.substr(i, c - i);
                        try { out.push_back(std::stof(tok)); }
                        catch (...) { return false; }
                        if (c == std::string::npos) break;
                        i = c + 1;
                    }
                    return true;
                };
                std::vector<float> rot, gb, ab;
                if (extract("rotation",  rot) && rot.size() == 9 &&
                    extract("gyro_bias", gb)  && gb.size()  == 3 &&
                    extract("accel_bias",ab)  && ab.size()  == 3) {
                    for (int k = 0; k < 9; ++k) R_GtoI_use[k]   = rot[k];
                    for (int k = 0; k < 3; ++k) gyro_bias_use[k]  = gb[k];
                    for (int k = 0; k < 3; ++k) accel_bias_use[k] = ab[k];
                    loaded_xml = true;
                    std::cerr << "replay_harness: loaded saved calibration "
                              << "(gyro_bias="
                              << gb[0] << "," << gb[1] << "," << gb[2]
                              << " rad/s)\n";
                }
            }
        }
        engine.loadStoredCalibration(R_GtoI_use, gyro_bias_use, accel_bias_use);
        if (!loaded_xml) {
            std::cerr << "replay_harness: navsight_calibration.xml missing; "
                      << "using sample-derived R_GtoI + zero biases (drift will "
                      << "dominate)\n";
        }

        // 2026-05-20 — Bootstrap Madgwick so EKF::initializeFull's
        // imu.getRotationGtoI() returns non-empty on the first frame.
        // Order matters:
        //   1. seedMadgwickYaw FIRST so Madgwick init (triggered by
        //      the first addAccelData in the warmup loop) consumes
        //      the right yaw seed. setInitialMadgwickYaw sets
        //      pending_madgwick_yaw_nav_ which is consumed once at
        //      tryInitMadgwickLocked. Calling it after the warmup
        //      is too late — Madgwick already inited with yaw=0.
        //   2. Warmup 200 IMU samples through Madgwick.
        //   3. EKF initializeFull will fire on the first processFrame
        //      with Madgwick's full roll/pitch/yaw.
        double seed_yaw = 0.0;
        if (sim_json.contains("points") && sim_json["points"].is_array() &&
            !sim_json["points"].empty()) {
            const auto& p0 = sim_json["points"][0];
            double h = 0.0;
            if (p0.contains("hdg") && jsonNumber(p0["hdg"], h)) {
                seed_yaw = h;
            }
        }
        engine.seedMadgwickYaw(seed_yaw);
        // Warmup: 200 samples ≈ 1 sec of IMU. Enough for Madgwick to
        // converge on roll/pitch from accel without pre-integrating
        // too much real walking motion. Skip these in the main loop
        // to avoid double-feeding. Increasing to 1000 includes
        // walking motion → Madgwick's quaternion absorbs the wrong
        // rotation → reprojection RMS skyrockets.
        kMadgwickWarmup = std::min<size_t>(200, samples.size());
        for (size_t i = 0; i < kMadgwickWarmup; ++i) {
            const auto& s = samples[i];
            engine.addAccelData(s.ts_ns, s.ax, s.ay, s.az);
            engine.addGyroData (s.ts_ns, s.gx, s.gy, s.gz);
        }
        std::cerr << "replay_harness: seeded Madgwick yaw="
                  << (seed_yaw * 180.0 / 3.14159265) << " deg, then warmed with "
                  << kMadgwickWarmup << " IMU samples\n";
    }

    // 2026-05-20 — initialise the runtime-side setters the Android app
    // calls but the harness historically skipped. Without these, EKF
    // never reaches full_initialized_=true → vision_valid stays 0 →
    // every SLAM/landmark/LC counter remains 0 across the entire replay.
    //
    // Critical: initial heading comes from the recorded sim's first
    // `hdg` field (device magnetometer-derived azimuth at the start of
    // the walk, CW-positive nav convention, radians). Without this,
    // every replay's EKF "thinks" yaw=0 (arbitrary North) while the
    // recording's actual scene is rotated by the real heading. The two-
    // view triangulation then puts world points behind the camera
    // (chirality fail) or far enough off that reprojection RMS = 160 px
    // — the 2026-05-20 dominant rejection mode for SLAM promotion.
    if (!frames_dir.empty()) {
        // R_bc rear-camera. NOTE: NavSight's default has CHANGED over
        // time. Pre-2026-05-19 walks (v54_two_loop, etc.) were made
        // with diag(1, -1, -1) (still visible in EKFState.cpp's
        // getExtrinsicsAngleDeg R_bc_initial constant). Post-2026-05-19
        // the EKFState ctor switched to ((0,-1,0),(-1,0,0),(0,0,-1)).
        // The two differ by 90°. Using the wrong one for an old
        // recording causes systematic 160-px reprojection RMS, killing
        // SLAM promotion via the 1.5 px gate.
        //
        // Detection: read the recording's startTime. Before 2026-05-19
        // (epoch ms < ~1779192000000) use diag; after, use the new
        // matrix. Recordings near the cutover are ambiguous and may
        // need manual override.
        // 2026-05-19 12:00 UTC in epoch ms. Walks recorded earlier used
        // the diag(1,-1,-1) R_bc; later walks use the new convention.
        constexpr int64_t kRbcCutoverEpochMs = 1779278400000LL;
        const int64_t start_ms =
            sim_json.contains("startTime") && sim_json["startTime"].is_number()
                ? sim_json["startTime"].get<int64_t>() : kRbcCutoverEpochMs;
        const bool is_old_rbc_era = start_ms < kRbcCutoverEpochMs;
        float R_bc_flat[9];
        if (is_old_rbc_era) {
            // Pre-2026-05-19 — diag(1, -1, -1)
            R_bc_flat[0] = 1.0f; R_bc_flat[1] = 0.0f; R_bc_flat[2] = 0.0f;
            R_bc_flat[3] = 0.0f; R_bc_flat[4] = -1.0f; R_bc_flat[5] = 0.0f;
            R_bc_flat[6] = 0.0f; R_bc_flat[7] = 0.0f; R_bc_flat[8] = -1.0f;
            std::cerr << "replay_harness: using PRE-2026-05-19 R_bc = diag(1,-1,-1)\n";
        } else {
            // Current
            R_bc_flat[0] = 0.0f; R_bc_flat[1] = -1.0f; R_bc_flat[2] = 0.0f;
            R_bc_flat[3] = -1.0f; R_bc_flat[4] = 0.0f; R_bc_flat[5] = 0.0f;
            R_bc_flat[6] = 0.0f; R_bc_flat[7] = 0.0f; R_bc_flat[8] = -1.0f;
            std::cerr << "replay_harness: using POST-2026-05-19 R_bc = ((0,-1,0),(-1,0,0),(0,0,-1))\n";
        }
        engine.setExtrinsicsRotation(R_bc_flat);
        engine.setUserHeight(1.7f);

        // Pull recorded initial heading from points[0].hdg (radians,
        // CW-positive nav per project convention). Fall back to 0 if
        // the field is missing or null (e.g., compass unavailable
        // during recording).
        double initial_heading = 0.0;
        if (sim_json.contains("points") && sim_json["points"].is_array() &&
            !sim_json["points"].empty()) {
            const auto& p0 = sim_json["points"][0];
            double h = 0.0;
            if (p0.contains("hdg") && jsonNumber(p0["hdg"], h)) {
                initial_heading = h;
            }
        }
        engine.setInitialHeading(initial_heading);
        std::cerr << "replay_harness: applied runtime defaults "
                  << "(R_bc rear-cam, height=1.7m, heading="
                  << (initial_heading * 180.0 / 3.14159265) << " deg)\n";
    }

    // 2026-05-20 — when recorded frames are present, load device camera
    // calibration so EKF intrinsics match the recording. Without this,
    // synthetic (320×240) intrinsics misalign against real 640×480
    // frames → vision_valid stays at 0 → EKF never initialises → all
    // SLAM/landmark/LC counters remain 0 in the replay output.
    //
    // The calibration JSON is checked in at
    // `tests/cpp/desktop_stubs/camera_calib_s21u.json` and was pulled
    // from the S21 Ultra device's in-app calibration via run-as.
    // RMS reprojection error 0.106 px — good calibration.
    if (!frames_dir.empty()) {
        const std::filesystem::path calib_path = std::filesystem::path(
            __FILE__).parent_path() / "desktop_stubs" / "camera_calib_s21u.json";
        std::ifstream calib_in(calib_path);
        if (calib_in) {
            try {
                nlohmann::json cj;
                calib_in >> cj;
                const double fx = cj.value("fx", 0.0);
                const double fy = cj.value("fy", 0.0);
                const double cx = cj.value("cx", 0.0);
                const double cy = cj.value("cy", 0.0);
                if (fx > 0 && fy > 0 && cx > 0 && cy > 0) {
                    engine.setIntrinsics(fx, fy, cx, cy);
                    std::cerr << "replay_harness: loaded calibration "
                              << "fx=" << fx << " fy=" << fy
                              << " cx=" << cx << " cy=" << cy << "\n";
                }
                if (cj.contains("dist")) {
                    const auto& dj = cj["dist"];
                    const double k1 = dj.value("k1", 0.0);
                    const double k2 = dj.value("k2", 0.0);
                    const double k3 = dj.value("k3", 0.0);
                    const double k4 = dj.value("k4", 0.0);
                    const double k5 = dj.value("k5", 0.0);
                    const double k6 = dj.value("k6", 0.0);
                    const double p1 = dj.value("p1", 0.0);
                    const double p2 = dj.value("p2", 0.0);
                    engine.getTracker()->setDistortion(k1, k2, k3, k4, k5, k6, p1, p2);
                    std::cerr << "replay_harness: loaded distortion "
                              << "k1=" << k1 << " k2=" << k2 << "\n";
                }
            } catch (const std::exception& e) {
                std::cerr << "replay_harness: calibration load failed: "
                          << e.what() << " — falling back to synthetic\n";
            }
        } else {
            std::cerr << "replay_harness: no calibration at "
                      << calib_path << "; vision_valid may stay 0\n";
        }
    }

    // Reusable buffer for synthetic frames.
    const std::vector<uint8_t> synthetic_yuv(
        static_cast<size_t>(kSyntheticWidth) * kSyntheticHeight * 3 / 2,
        kSyntheticFill);
    // Reusable per-frame NV21 buffer for recorded frames (sized lazily).
    std::vector<uint8_t> recorded_nv21;
    size_t depth_frames_fed = 0;  // 2026-05-29 — count of MiDaS rasters fed

    // 2026-05-29 — warm-start K seed. On-device, midas_scale_K_/expansion_scale_K_
    // persist in SharedPreferences across launches; the harness starts cold
    // (K=-1) so the looming speed path bails until depth-flow calibrates K — but
    // depth-flow is gated on essential-matrix verification, which fails on slow
    // walks → deadlock. Seeding K emulates a prior session having calibrated it.
    if (camera_height > 0.0) {
        engine.getTracker()->setCameraHeight(camera_height);
        std::fprintf(stdout, "replay_harness: ground-plane camera height = %.3f m\n", camera_height);
    }
    if (seed_k > 0.0) {
        engine.getTracker()->setMidasScaleK(seed_k);
        std::fprintf(stdout, "replay_harness: seeded warm K = %.1f\n", seed_k);
    }

    // 2026-05-31 — verdict §4 autonomous-p_G_ A/B. Enable BEFORE processing so
    // the very first frame's setPosition is already gated. Default arm leaves
    // this off (byte-identical to the device build's setPosition behaviour).
    if (autonomous_pg) {
        engine.setAutonomousPgExperiment(true);
        std::fprintf(stdout, "replay_harness: AUTONOMOUS p_G_ experiment ON "
                             "(setPosition skipped; ekf_x/y/z = autonomous p_G_)\n");
    }

    // Skip samples already fed during Madgwick warmup (otherwise they
    // get integrated twice — double-rotates Madgwick + EKF state).
    for (size_t s_idx = kMadgwickWarmup; s_idx < samples.size(); ++s_idx) {
        const auto& s = samples[s_idx];
        // Feed IMU strictly in the recorded order; both calls use the same
        // timestamp so the preintegrator window covers a single sample.
        engine.addAccelData(s.ts_ns, s.ax, s.ay, s.az);
        engine.addGyroData (s.ts_ns, s.gx, s.gy, s.gz);

        // Try to load a recorded frame for this timestamp. Falls back to the
        // synthetic mid-grey buffer when there is no PNG on disk.
        cv::Mat grey;
        std::filesystem::path matched_frame;
        bool frame_loaded = tryLoadFrame(frames_dir, frames_index,
                                         s.ts_ns, frame_tolerance_ns, grey,
                                         matched_frame);

        // 2026-05-29 — feed MiDaS depth for this frame BEFORE processFrame, so
        // the depth-flow speed path (updateDepthFlowSpeed) sees the same depth
        // the on-device pipeline pushes via setDepthMap. Keyed by the matched
        // frame's stem (<stem>.f32). Must precede processFrame: the camera-
        // thread reads depth_map_ during processFrame.
        if (frame_loaded && !depth_dir.empty()) {
            std::vector<float> depth;
            int dside = 0;
            if (tryLoadDepthRaster(depth_dir, matched_frame, depth, dside)) {
                engine.setDepthMap(depth.data(), dside, dside);
                ++depth_frames_fed;
            }
        }

        VisionOutput vout;
        if (frame_loaded) {
            greyToNv21(grey, recorded_nv21);
            vout = engine.processFrame(recorded_nv21.data(),
                                       grey.cols, grey.rows, s.ts_ns);
        } else if (frames_dir.empty()) {
            // IMU-only mode: feed synthetic mid-grey. Visual front-end
            // will find nothing in a flat frame but won't crash.
            vout = engine.processFrame(synthetic_yuv.data(),
                                       kSyntheticWidth, kSyntheticHeight,
                                       s.ts_ns);
        }
        // 2026-05-20 — In recorded-frames mode, samples WITHOUT a matching
        // frame are SKIPPED entirely (do NOT fall back to synthetic).
        // Mixing real 640×480 frames with synthetic 320×240 mid-grey
        // crashes KLT at cv::calcOpticalFlowPyrLK (pyramid size mismatch
        // between consecutive frames). Skipping keeps KLT state intact;
        // the IMU is already fed above so the EKF still propagates.

        double x = std::nan(""), y = std::nan(""), z = std::nan(""), yaw = std::nan("");
        bool init = engine.getPose(x, y, z, yaw);
        double cov[3] = {0.0, 0.0, 0.0};
        engine.getPositionCovarianceXZ(cov);

        csv << s.ts_ns << ','
            << (init ? 1 : 0) << ','
            << x << ',' << y << ',' << z << ',' << yaw << ','
            << cov[0] << ',' << cov[1] << ',' << cov[2] << ','
            << s.vx << ',' << s.vy << ',' << s.vz << ',' << s.vyaw << ',';
        if (s.has_gps) {
            csv << s.glat << ',' << s.glng << ',' << s.gacc_m;
        } else {
            csv << ",,";
        }
        csv << ','
            << vout.inlierCount << ','
            << vout.trackedCount << ','
            << vout.totalCount << ','
            << vout.meanFlow << ','
            << vout.poseFlags << ','
            << (vout.valid ? 1 : 0) << ','
            << (frame_loaded ? 1 : 0) << ','
            << (vout.keyframe_stored ? 1 : 0) << ','
            << vout.heading << ',';
        // 2026-05-24 BISECT — yaw-cov columns disabled so the harness compiles
        // against the COMMITTED baseline too (getYawVariance/getYawCovDiag are
        // stash-only). Heading bisection only needs vout.heading above.
        double yawdiag[3] = { std::nan(""), std::nan(""), std::nan("") };
        csv << std::nan("") << ',' << yawdiag[1] << ',' << yawdiag[2] << ',';
        // 2026-05-29 — user-facing dot trajectory (global_t_) + UI speed.
        // vout.t is only populated on frames where the visual front-end ran
        // (frame_loaded && valid); on skipped/IMU-only ticks it is empty, so
        // guard and emit NaN to keep the column count stable.
        double tx = std::nan(""), ty = std::nan(""), tz = std::nan("");
        if (!vout.t.empty() && vout.t.rows >= 3 && vout.t.type() == CV_64F) {
            tx = vout.t.at<double>(0, 0);
            ty = vout.t.at<double>(1, 0);
            tz = vout.t.at<double>(2, 0);
        }
        const double fused_speed = engine.getTracker()->getFusedSpeedMps();
        const Tracker* tk = engine.getTracker();
        csv << tx << ',' << ty << ',' << tz << ',' << fused_speed << ','
            << tk->getGroundFlowSpeedMps() << ','
            << (tk->isGroundPlaneValid() ? 1 : 0) << ','
            << tk->getGroundPlaneScale() << ','
            << tk->getGroundPlaneConfidence() << ','
            << tk->getGroundPlaneHvio() << ','
            << tk->getGroundPlaneCandidates() << ','
            << tk->getGroundPlaneInliers() << ','
            << tk->getGroundPlaneHorizonV() << '\n';
    }
    csv.flush();
    if (!csv) {
        std::fprintf(stderr, "replay_harness: write failed for %s\n", csv_path.c_str());
        return 7;
    }

    // Sidecar EventCounters dump. Lock-free atomic snapshot per
    // EventCounters.h:271-375; the per-field reads are individually atomic
    // but the file as a whole is not snapshot-consistent. That's fine for
    // CI scoring — every counter monotonically increases over a run, so the
    // values at file-close time are valid lower bounds.
    const std::string sidecar_path = csv_path + ".event_counters.json";
    std::ofstream sidecar(sidecar_path);
    if (sidecar) {
        sidecar << navsight::eventCounters().serializeAsJsonString();
        sidecar.flush();
        if (!sidecar) {
            std::fprintf(stderr,
                "replay_harness: warning — failed to flush %s\n",
                sidecar_path.c_str());
        }
    } else {
        std::fprintf(stderr,
            "replay_harness: warning — cannot open %s for write\n",
            sidecar_path.c_str());
    }

    std::fprintf(stdout,
        "replay_harness: wrote %zu rows -> %s (sidecar: %s)\n",
        samples.size(), csv_path.c_str(), sidecar_path.c_str());
    if (!depth_dir.empty()) {
        std::fprintf(stdout, "replay_harness: fed %zu MiDaS depth rasters\n",
                     depth_frames_fed);
    }
    return 0;
}
