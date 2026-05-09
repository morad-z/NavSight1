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
                  cv::Mat& out) {
    auto p = resolveFramePath(frames_dir, idx, ts_ns, tolerance_ns);
    if (p.empty()) return false;
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

}  // namespace

int main(int argc, char** argv) {
    // Parse positional + flag args. Keeping it hand-rolled avoids pulling in
    // a CLI library for two flags.
    std::string sim_path;
    std::string csv_path;
    std::string frames_dir_override;
    int64_t frame_tolerance_ns = 0;
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
                "[--frames-dir <path>] [--frame-match-tolerance-ms <N>]\n",
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
        << "pose_flags,vision_valid,frame_loaded,keyframe_stored\n";

    VioEngine engine;
    // Synthetic intrinsics (used as defaults when no frames are present).
    // When recorded frames load, the size of the first loaded frame stays
    // consistent with what the recording used — the harness does NOT
    // re-derive intrinsics here; the runtime is expected to have published
    // calibration via setIntrinsics during recording, which is replayed
    // implicitly via the EKF's last-known state if you pre-seed via a
    // companion script. For the first CI fixture (synthetic), this default
    // matches the synthetic frame dimensions.
    engine.setIntrinsics(static_cast<double>(kSyntheticWidth),
                         static_cast<double>(kSyntheticWidth),
                         kSyntheticWidth * 0.5,
                         kSyntheticHeight * 0.5);

    // Reusable buffer for synthetic frames.
    const std::vector<uint8_t> synthetic_yuv(
        static_cast<size_t>(kSyntheticWidth) * kSyntheticHeight * 3 / 2,
        kSyntheticFill);
    // Reusable per-frame NV21 buffer for recorded frames (sized lazily).
    std::vector<uint8_t> recorded_nv21;

    for (const auto& s : samples) {
        // Feed IMU strictly in the recorded order; both calls use the same
        // timestamp so the preintegrator window covers a single sample.
        engine.addAccelData(s.ts_ns, s.ax, s.ay, s.az);
        engine.addGyroData (s.ts_ns, s.gx, s.gy, s.gz);

        // Try to load a recorded frame for this timestamp. Falls back to the
        // synthetic mid-grey buffer when there is no PNG on disk.
        cv::Mat grey;
        bool frame_loaded = tryLoadFrame(frames_dir, frames_index,
                                         s.ts_ns, frame_tolerance_ns, grey);

        VisionOutput vout;
        if (frame_loaded) {
            greyToNv21(grey, recorded_nv21);
            vout = engine.processFrame(recorded_nv21.data(),
                                       grey.cols, grey.rows, s.ts_ns);
        } else {
            vout = engine.processFrame(synthetic_yuv.data(),
                                       kSyntheticWidth, kSyntheticHeight,
                                       s.ts_ns);
        }

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
            << (vout.keyframe_stored ? 1 : 0)
            << '\n';
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
    return 0;
}
