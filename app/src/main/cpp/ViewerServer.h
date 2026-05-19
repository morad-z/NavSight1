#pragma once

// ViewerServer — minimal HTTP server that exposes a 3D web viewer for the
// running NavSight VIO state.
//
// Usage:
//   PC opens http://<phone-ip>:8765/ in a browser
//   The page (single self-contained HTML) connects back to /state.json at 5 Hz
//   and renders:
//     - LandmarkMap point cloud (orange = observed-this-frame, gray = dormant)
//     - Current EKF camera pose (frustum)
//     - Trajectory trail (last ~500 positions accumulated client-side)
//     - Orbit / zoom mouse controls (Three.js OrbitControls)
//
// Architecture: single TCP listener thread + per-request short-lived response.
// No external HTTP library; ~250 lines of BSD-socket C++. Bound to 0.0.0.0:8765
// so any device on the same WiFi can connect. Stops on Tracker reset / app
// shutdown.
//
// Why not WebSocket: HTTP polling at 5 Hz is simpler, browser-native, no
// handshake state machine. The 200 ms polling rate matches the visual
// debugging cadence we need; bandwidth is ~10 KB/s for typical map sizes.

#include <atomic>
#include <thread>
#include <memory>

class Tracker;  // forward declaration

namespace navsight {

class ViewerServer {
public:
    ViewerServer();
    ~ViewerServer();

    // Start the HTTP listener on the given port. Non-blocking; spawns a
    // background thread. Idempotent — second start() is a no-op.
    void start(Tracker* tracker, int port = 8765);

    // Signal the server thread to stop and join. Idempotent.
    void stop();

    // Public so JNI can query liveness for diagnostics.
    bool isRunning() const { return running_.load(); }

private:
    void runServerLoop(int port);
    void handleClient(int client_fd);
    std::string buildStateJson() const;
    std::string buildIndexHtml() const;

    Tracker* tracker_{nullptr};
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    std::unique_ptr<std::thread> server_thread_;
};

}  // namespace navsight
