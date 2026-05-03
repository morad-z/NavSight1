// Step 7 — Replay Harness
//
// Reads a NavSight simulation JSON (schema documented in
// scripts/analyze_sim.py and matched by the runtime sim recorder), drives the
// VIO pipeline at the recorded sample timestamps, and emits a CSV trace of
// EKF pose + horizontal-plane covariance per frame for downstream regression
// scoring (replay_scorer.py).
//
// Camera bytes are not present in the recordings; the harness drives an
// IMU-only replay. To trigger EKF propagation, processFrame is invoked with a
// synthetic uniform 320x240 grey YUV buffer once per sample. The visual layer
// will not produce features against a flat frame, so the EKF state evolves
// purely under IMU integration — exactly the deterministic regression channel
// we want to gate CI on.
//
// CLI: replay_harness <sim_input.json> <pose_output.csv>
// Exit codes: 0 success, non-zero failure.
//
// CSV columns:
//   ts_ns, ekf_init, ekf_x, ekf_y, ekf_z, ekf_yaw_rad,
//   sigma_xx, sigma_xz, sigma_zz,
//   recorded_vx, recorded_vy, recorded_vz, recorded_vyaw_rad,
//   glat, glng, gacc_m
//
// Determinism:
//   • Samples are sorted ascending by ts before streaming.
//   • IMU is fed strictly in timestamp order.
//   • The synthetic frame buffer is constant across runs.

#include "VioEngine.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr int kSyntheticWidth  = 320;
constexpr int kSyntheticHeight = 240;
constexpr uint8_t kSyntheticFill = 64;  // mid-grey, away from clipping

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

bool loadSim(const std::string& path, std::vector<SimSample>& out) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "replay_harness: cannot open %s\n", path.c_str());
        return false;
    }
    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "replay_harness: parse error: %s\n", e.what());
        return false;
    }
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

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr,
            "Usage: %s <sim_input.json> <pose_output.csv>\n",
            (argc > 0 ? argv[0] : "replay_harness"));
        return 2;
    }
    const std::string sim_path = argv[1];
    const std::string csv_path = argv[2];

    std::vector<SimSample> samples;
    if (!loadSim(sim_path, samples)) return 3;
    if (samples.empty()) {
        std::fprintf(stderr, "replay_harness: no usable samples in %s\n", sim_path.c_str());
        return 4;
    }

    std::ofstream csv(csv_path);
    if (!csv) {
        std::fprintf(stderr, "replay_harness: cannot open %s for write\n", csv_path.c_str());
        return 5;
    }
    csv.setf(std::ios::fixed);
    csv.precision(9);
    csv << "ts_ns,ekf_init,ekf_x,ekf_y,ekf_z,ekf_yaw_rad,"
        << "sigma_xx,sigma_xz,sigma_zz,"
        << "recorded_vx,recorded_vy,recorded_vz,recorded_vyaw_rad,"
        << "glat,glng,gacc_m\n";

    VioEngine engine;
    // Synthetic intrinsics; their absolute value is irrelevant when no
    // real features are tracked. They must be non-zero so the EKF wiring
    // does not short-circuit on a missing-K guard.
    engine.setIntrinsics(static_cast<double>(kSyntheticWidth),
                         static_cast<double>(kSyntheticWidth),
                         kSyntheticWidth * 0.5,
                         kSyntheticHeight * 0.5);

    const std::vector<uint8_t> yuv(
        static_cast<size_t>(kSyntheticWidth) * kSyntheticHeight * 3 / 2,
        kSyntheticFill);

    for (const auto& s : samples) {
        // Feed IMU strictly in the recorded order; both calls use the same
        // timestamp so the preintegrator window covers a single sample.
        engine.addAccelData(s.ts_ns, s.ax, s.ay, s.az);
        engine.addGyroData (s.ts_ns, s.gx, s.gy, s.gz);

        // Drive a frame so propagateIMU runs against the queued window.
        engine.processFrame(yuv.data(), kSyntheticWidth, kSyntheticHeight, s.ts_ns);

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
        csv << '\n';
    }
    csv.flush();
    if (!csv) {
        std::fprintf(stderr, "replay_harness: write failed for %s\n", csv_path.c_str());
        return 6;
    }
    std::fprintf(stdout, "replay_harness: wrote %zu rows -> %s\n",
                 samples.size(), csv_path.c_str());
    return 0;
}
