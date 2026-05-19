#include "ViewerServer.h"
#include "Tracker.h"
#include "EKFState.h"
#include "LandmarkMap.h"

#include <android/log.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <vector>

#define LOG_TAG "NavSight-Viewer"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace navsight {

ViewerServer::ViewerServer() = default;

ViewerServer::~ViewerServer() { stop(); }

void ViewerServer::start(Tracker* tracker, int port) {
    if (running_.load()) return;
    tracker_ = tracker;
    should_stop_.store(false);
    running_.store(true);
    server_thread_ = std::make_unique<std::thread>(
        &ViewerServer::runServerLoop, this, port);
    LOGI("ViewerServer starting on port %d", port);
}

void ViewerServer::stop() {
    should_stop_.store(true);
    if (server_thread_ && server_thread_->joinable()) {
        server_thread_->join();
    }
    server_thread_.reset();
    running_.store(false);
}

void ViewerServer::runServerLoop(int port) {
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        LOGE("socket() failed: %s", strerror(errno));
        running_.store(false);
        return;
    }
    int opt = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(static_cast<uint16_t>(port));

    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOGE("bind() failed on port %d: %s", port, strerror(errno));
        ::close(listen_fd);
        running_.store(false);
        return;
    }
    if (::listen(listen_fd, 4) < 0) {
        LOGE("listen() failed: %s", strerror(errno));
        ::close(listen_fd);
        running_.store(false);
        return;
    }
    LOGI("ViewerServer listening on 0.0.0.0:%d", port);

    // Non-blocking poll so we can check should_stop_ periodically.
    timeval tv{};
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (!should_stop_.load()) {
        sockaddr_in client_addr{};
        socklen_t   client_len = sizeof(client_addr);
        int client_fd = ::accept(listen_fd,
                                 reinterpret_cast<sockaddr*>(&client_addr),
                                 &client_len);
        if (client_fd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            LOGW("accept() failed: %s", strerror(errno));
            continue;
        }
        handleClient(client_fd);
        ::close(client_fd);
    }
    ::close(listen_fd);
    running_.store(false);
    LOGI("ViewerServer stopped");
}

void ViewerServer::handleClient(int client_fd) {
    // Read request line. Enough to distinguish GET / from GET /state.json.
    char buf[1024];
    ssize_t n = ::recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;
    buf[n] = '\0';

    bool wants_state = (std::strstr(buf, "GET /state.json") != nullptr);

    std::string body;
    std::string content_type;
    if (wants_state) {
        body = buildStateJson();
        content_type = "application/json";
    } else {
        body = buildIndexHtml();
        content_type = "text/html; charset=utf-8";
    }

    std::ostringstream resp;
    resp << "HTTP/1.1 200 OK\r\n"
         << "Content-Type: " << content_type << "\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Access-Control-Allow-Origin: *\r\n"
         << "Cache-Control: no-store\r\n"
         << "Connection: close\r\n\r\n"
         << body;
    const std::string out = resp.str();
    ::send(client_fd, out.data(), out.size(), 0);
}

std::string ViewerServer::buildStateJson() const {
    std::ostringstream out;
    out << "{";
    if (!tracker_) {
        out << "\"ready\":false}";
        return out.str();
    }
    const EKFState* ekf = tracker_->getEKF();
    if (!ekf || !ekf->isFullInitialized()) {
        out << "\"ready\":false}";
        return out.str();
    }
    cv::Mat p_G = ekf->getPosition();
    cv::Mat R   = ekf->getRotation();
    out << "\"ready\":true";

    // Camera pose. p in EKF Z-up (X=East, Y=North, Z=Up). We emit Z-up
    // directly and let the JS choose visualization axes.
    out << ",\"pose\":{\"p\":["
        << p_G.at<double>(0) << "," << p_G.at<double>(1) << "," << p_G.at<double>(2)
        << "],\"R\":[";
    for (int r = 0; r < 3; r++) {
        if (r > 0) out << ",";
        out << "[" << R.at<double>(r,0) << "," << R.at<double>(r,1) << "," << R.at<double>(r,2) << "]";
    }
    out << "]}";

    // Landmarks. Pull from LandmarkMap via Tracker accessor. Cap at 1000 to
    // keep payload bounded; pick closest by Euclidean distance to current p_G.
    const LandmarkMap& lm = tracker_->getLandmarkMap();
    const cv::Vec3d p_center(p_G.at<double>(0), p_G.at<double>(1), p_G.at<double>(2));
    std::vector<int> ids = lm.getLandmarksInRadius(p_center, 100.0, -1, -1);
    std::vector<int> observed_ids = tracker_->getLastObservedLandmarkIds();
    // Use a set-like membership check; observed_ids is short (~tens) so
    // linear is fine for under-tens count typical of a single keyframe.
    auto is_observed = [&observed_ids](int id) {
        for (int oid : observed_ids) if (oid == id) return true;
        return false;
    };

    // Dedup ids — getLandmarksInRadius can return the same id multiple
    // times (KD-tree may revisit nodes). Sort + unique.
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    const size_t cap = ids.size() < 1000 ? ids.size() : 1000;
    out << ",\"landmarks\":[";
    bool first = true;
    for (size_t i = 0; i < cap; i++) {
        Landmark lm_rec;
        if (!lm.getLandmark(ids[i], lm_rec)) continue;
        if (!first) out << ",";
        first = false;
        out << "[" << lm_rec.id
            << "," << lm_rec.p_world[0]
            << "," << lm_rec.p_world[1]
            << "," << lm_rec.p_world[2]
            << "," << lm_rec.times_observed
            << "," << (is_observed(lm_rec.id) ? 1 : 0)
            << "]";
    }
    out << "]";
    out << ",\"map_size\":" << lm.size();
    out << "}";
    return out.str();
}

// Embedded viewer HTML. Single page, Three.js from CDN, polls /state.json
// at 5 Hz. Accumulates trajectory client-side from the streamed pose.
std::string ViewerServer::buildIndexHtml() const {
    return R"HTMLPAGE(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>NavSight Live Viewer</title>
<style>
body{margin:0;background:#0a0a14;color:#9bd;font-family:monospace;overflow:hidden}
#hud{position:fixed;top:8px;left:8px;background:#0008;padding:8px;border-radius:4px;font-size:12px;line-height:1.5}
#canvas{display:block;width:100vw;height:100vh}
.k{color:#7af}.v{color:#fc6}
</style></head>
<body><div id="hud">
<div><span class="k">NavSight Live</span></div>
<div><span class="k">map_size:</span> <span class="v" id="msz">-</span></div>
<div><span class="k">in radius:</span> <span class="v" id="nr">-</span></div>
<div><span class="k">observed:</span> <span class="v" id="no">-</span></div>
<div><span class="k">pose:</span> <span class="v" id="pp">-</span></div>
<div><span class="k">trail:</span> <span class="v" id="tl">-</span></div>
<div style="margin-top:8px;color:#888">drag=orbit · wheel=zoom · right-drag=pan</div>
</div>
<canvas id="canvas"></canvas>
<script type="importmap">{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const canvas = document.getElementById('canvas');
const renderer = new THREE.WebGLRenderer({canvas, antialias:true});
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0a14);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.up.set(0,0,1);  // Z-up world
camera.position.set(8, 8, 6);
camera.lookAt(0,0,0);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0,0,0);

// Ground grid (Z-up: grid lies in XY plane)
const grid = new THREE.GridHelper(40, 40, 0x335577, 0x223344);
grid.rotation.x = Math.PI/2;  // make it XY
scene.add(grid);

// Axes
const axes = new THREE.AxesHelper(2);
scene.add(axes);

// Landmark cloud — two BufferGeometry pools, observed (orange) and dormant (gray)
const MAX_PTS = 2000;
function makeCloud(color, size) {
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_PTS*3), 3));
  geom.setDrawRange(0, 0);
  const mat = new THREE.PointsMaterial({color, size, sizeAttenuation:true});
  const pts = new THREE.Points(geom, mat);
  scene.add(pts);
  return {geom, mat, pts};
}
const cloudObserved = makeCloud(0xff8000, 0.25);
const cloudDormant  = makeCloud(0x666666, 0.12);

// Camera frustum — pyramid pointing along BODY +X (NavSight body frame
// convention: camera-forward = body +X for vertical-phone orientation).
// Apex at origin, rectangular base at x=fdepth, half-width fy, half-height fz.
// 2026-05-17 fix: previously pointed along local +Z, which made pitch
// rotations look inverted in the viewer (phone-look-up shown as look-down)
// because body +Z is the phone's "up" axis, not its forward direction.
const frustGeom = new THREE.BufferGeometry();
const fdepth = 0.5, fy = 0.3, fz = 0.2;
const frustVerts = new Float32Array([
  0,0,0,  fdepth, fy, fz,
  0,0,0,  fdepth, fy,-fz,
  0,0,0,  fdepth,-fy,-fz,
  0,0,0,  fdepth,-fy, fz,
  fdepth, fy, fz,  fdepth, fy,-fz,
  fdepth, fy,-fz,  fdepth,-fy,-fz,
  fdepth,-fy,-fz,  fdepth,-fy, fz,
  fdepth,-fy, fz,  fdepth, fy, fz
]);
frustGeom.setAttribute('position', new THREE.BufferAttribute(frustVerts, 3));
const frustMat = new THREE.LineBasicMaterial({color:0x66ddff, linewidth:2});
const frustum = new THREE.LineSegments(frustGeom, frustMat);
scene.add(frustum);

// Trajectory polyline (client-accumulated)
const MAX_TRAIL = 1000;
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_TRAIL*3), 3));
trailGeom.setDrawRange(0, 0);
const trailMat = new THREE.LineBasicMaterial({color:0xffaa00});
const trail = new THREE.Line(trailGeom, trailMat);
scene.add(trail);
const trailPos = trailGeom.attributes.position.array;
let trailN = 0;

function appendTrail(x,y,z) {
  if (trailN < MAX_TRAIL) {
    trailPos[trailN*3+0] = x;
    trailPos[trailN*3+1] = y;
    trailPos[trailN*3+2] = z;
    trailN++;
  } else {
    // shift (FIFO)
    trailPos.copyWithin(0, 3, MAX_TRAIL*3);
    trailPos[(MAX_TRAIL-1)*3+0] = x;
    trailPos[(MAX_TRAIL-1)*3+1] = y;
    trailPos[(MAX_TRAIL-1)*3+2] = z;
  }
  trailGeom.setDrawRange(0, trailN);
  trailGeom.attributes.position.needsUpdate = true;
}

// Build the camera-to-world matrix and apply to frustum.
function updateFrustum(p, R_rows) {
  // R_rows is world-to-body or body-to-world? In NavSight EKF state, R_GtoI
  // is world→body. The frustum visualizes the camera "looking out" in world
  // frame, so we want body→world rotation = R_GtoI.transpose(). We pass that
  // as a column-major mat4 to Three.js.
  // R_rows[r][c] is row r, col c of R_world→body. Transpose for body→world.
  const m = new THREE.Matrix4();
  m.set(
    R_rows[0][0], R_rows[1][0], R_rows[2][0], p[0],
    R_rows[0][1], R_rows[1][1], R_rows[2][1], p[1],
    R_rows[0][2], R_rows[1][2], R_rows[2][2], p[2],
    0,            0,            0,            1
  );
  frustum.matrix.copy(m);
  frustum.matrixAutoUpdate = false;
}

let lastTrailPoint = null;
async function poll() {
  try {
    const r = await fetch('/state.json', {cache:'no-store'});
    const s = await r.json();
    if (!s.ready) return;
    const p = s.pose.p, R = s.pose.R;
    updateFrustum(p, R);
    // Append to trail if moved > 5 cm.
    if (!lastTrailPoint ||
        Math.hypot(p[0]-lastTrailPoint[0], p[1]-lastTrailPoint[1], p[2]-lastTrailPoint[2]) > 0.05) {
      appendTrail(p[0], p[1], p[2]);
      lastTrailPoint = [p[0], p[1], p[2]];
    }
    // Split landmarks into observed/dormant
    let nO = 0, nD = 0;
    const oArr = cloudObserved.geom.attributes.position.array;
    const dArr = cloudDormant.geom.attributes.position.array;
    for (const lm of s.landmarks) {
      const [id, x, y, z, n_obs, observed] = lm;
      if (observed === 1 && nO < MAX_PTS) {
        oArr[nO*3+0]=x; oArr[nO*3+1]=y; oArr[nO*3+2]=z; nO++;
      } else if (nD < MAX_PTS) {
        dArr[nD*3+0]=x; dArr[nD*3+1]=y; dArr[nD*3+2]=z; nD++;
      }
    }
    cloudObserved.geom.setDrawRange(0, nO);
    cloudDormant.geom.setDrawRange(0, nD);
    cloudObserved.geom.attributes.position.needsUpdate = true;
    cloudDormant.geom.attributes.position.needsUpdate = true;

    document.getElementById('msz').textContent = s.map_size;
    document.getElementById('nr').textContent  = s.landmarks.length;
    document.getElementById('no').textContent  = nO;
    document.getElementById('pp').textContent  = p.map(v=>v.toFixed(2)).join(', ');
    document.getElementById('tl').textContent  = trailN;
  } catch(e) {
    // disconnected — retry next tick
  }
}
setInterval(poll, 200);  // 5 Hz

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth/window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
</script></body></html>
)HTMLPAGE";
}

}  // namespace navsight
