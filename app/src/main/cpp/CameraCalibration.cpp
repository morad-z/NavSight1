#include "CameraCalibration.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-CamCalib"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#endif

namespace {

// Read the entire file into a std::string. Returns false on any I/O error.
bool slurpFile(const std::string& path, std::string& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    if (std::fseek(f, 0, SEEK_END) != 0) { std::fclose(f); return false; }
    const long sz = std::ftell(f);
    if (sz < 0) { std::fclose(f); return false; }
    std::rewind(f);
    out.resize(static_cast<size_t>(sz));
    const size_t read =
        sz > 0 ? std::fread(out.data(), 1, static_cast<size_t>(sz), f) : 0;
    std::fclose(f);
    if (static_cast<long>(read) != sz) return false;
    return true;
}

// Tiny key search: locates the first occurrence of "key" (with surrounding
// quotes), then advances past the colon to the value. Returns std::string::npos
// if the key is absent. The schema is fixed and shallow — duplicates and
// nested key collisions are not a concern in practice for this loader.
size_t findKey(const std::string& s, const std::string& key, size_t from = 0) {
    const std::string needle = "\"" + key + "\"";
    size_t p = s.find(needle, from);
    if (p == std::string::npos) return std::string::npos;
    p += needle.size();
    // Skip whitespace and the colon.
    while (p < s.size() && (std::isspace(static_cast<unsigned char>(s[p])))) ++p;
    if (p >= s.size() || s[p] != ':') return std::string::npos;
    ++p;
    while (p < s.size() && (std::isspace(static_cast<unsigned char>(s[p])))) ++p;
    return p;
}

// Parse a JSON number starting at `pos`. Stores into `out` and returns true on
// success. Whitespace/punctuation after the number stops the scan.
bool parseNumber(const std::string& s, size_t pos, double& out) {
    if (pos >= s.size()) return false;
    char* end = nullptr;
    const char* start = s.c_str() + pos;
    const double v = std::strtod(start, &end);
    if (end == start) return false;
    out = v;
    return true;
}

// Convenience: read a top-level numeric field by key.
bool readNumber(const std::string& s, const std::string& key, double& out) {
    const size_t pos = findKey(s, key);
    if (pos == std::string::npos) {
        LOGE("loadCalibrationFromJson: missing key '%s'", key.c_str());
        return false;
    }
    if (!parseNumber(s, pos, out)) {
        LOGE("loadCalibrationFromJson: bad number for key '%s'", key.c_str());
        return false;
    }
    return true;
}

// Convenience: read a nested numeric field inside an object referenced by
// `parent_key`. The parser scopes to the parent object's '{...}' region so
// keys outside it cannot match.
bool readNestedNumber(const std::string& s, const std::string& parent_key,
                      const std::string& child_key, double& out) {
    const size_t parent_pos = findKey(s, parent_key);
    if (parent_pos == std::string::npos) {
        LOGE("loadCalibrationFromJson: missing parent '%s'", parent_key.c_str());
        return false;
    }
    if (parent_pos >= s.size() || s[parent_pos] != '{') {
        LOGE("loadCalibrationFromJson: parent '%s' is not an object",
             parent_key.c_str());
        return false;
    }
    // Find matching '}' with simple depth counting (no string escapes inside
    // numeric-only objects in this schema, so this is safe).
    int depth = 0;
    size_t end = parent_pos;
    for (; end < s.size(); ++end) {
        const char c = s[end];
        if (c == '{') ++depth;
        else if (c == '}') {
            --depth;
            if (depth == 0) break;
        }
    }
    if (depth != 0) {
        LOGE("loadCalibrationFromJson: unmatched braces in '%s'",
             parent_key.c_str());
        return false;
    }
    const std::string sub = s.substr(parent_pos, end - parent_pos + 1);
    const size_t child_pos = findKey(sub, child_key);
    if (child_pos == std::string::npos) {
        LOGE("loadCalibrationFromJson: missing '%s.%s'", parent_key.c_str(),
             child_key.c_str());
        return false;
    }
    if (!parseNumber(sub, child_pos, out)) {
        LOGE("loadCalibrationFromJson: bad number for '%s.%s'",
             parent_key.c_str(), child_key.c_str());
        return false;
    }
    return true;
}

// Read the two-element image_size array.
bool readImageSize(const std::string& s, int& w, int& h) {
    const size_t pos = findKey(s, "image_size");
    if (pos == std::string::npos) {
        LOGE("loadCalibrationFromJson: missing key 'image_size'");
        return false;
    }
    if (pos >= s.size() || s[pos] != '[') {
        LOGE("loadCalibrationFromJson: 'image_size' is not an array");
        return false;
    }
    size_t p = pos + 1;
    while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p]))) ++p;
    double dw = 0.0;
    if (!parseNumber(s, p, dw)) {
        LOGE("loadCalibrationFromJson: bad image_size[0]");
        return false;
    }
    // Skip the parsed number, whitespace, and comma.
    while (p < s.size() && s[p] != ',') ++p;
    if (p >= s.size()) return false;
    ++p;
    while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p]))) ++p;
    double dh = 0.0;
    if (!parseNumber(s, p, dh)) {
        LOGE("loadCalibrationFromJson: bad image_size[1]");
        return false;
    }
    w = static_cast<int>(dw);
    h = static_cast<int>(dh);
    return true;
}

}  // namespace

bool loadCalibrationFromJson(const std::string& path, CameraIntrinsics& out) {
    std::string contents;
    if (!slurpFile(path, contents)) {
        LOGI("loadCalibrationFromJson: cannot read '%s'", path.c_str());
        return false;
    }
    if (contents.empty()) {
        LOGE("loadCalibrationFromJson: empty file '%s'", path.c_str());
        return false;
    }

    CameraIntrinsics tmp;

    int w = 0, h = 0;
    if (!readImageSize(contents, w, h)) return false;
    if (w <= 0 || h <= 0) {
        LOGE("loadCalibrationFromJson: invalid image_size %dx%d", w, h);
        return false;
    }
    tmp.image_w = w;
    tmp.image_h = h;

    if (!readNumber(contents, "fx", tmp.fx)) return false;
    if (!readNumber(contents, "fy", tmp.fy)) return false;
    if (!readNumber(contents, "cx", tmp.cx)) return false;
    if (!readNumber(contents, "cy", tmp.cy)) return false;

    if (tmp.fx <= 0.0 || tmp.fy <= 0.0) {
        LOGE("loadCalibrationFromJson: fx/fy must be > 0 (fx=%.3f fy=%.3f)",
             tmp.fx, tmp.fy);
        return false;
    }
    if (tmp.cx < 0.0 || tmp.cx > static_cast<double>(w) ||
        tmp.cy < 0.0 || tmp.cy > static_cast<double>(h)) {
        LOGE("loadCalibrationFromJson: cx/cy out of image bounds "
             "(cx=%.2f cy=%.2f, image=%dx%d)", tmp.cx, tmp.cy, w, h);
        return false;
    }

    if (!readNestedNumber(contents, "dist", "k1", tmp.k1)) return false;
    if (!readNestedNumber(contents, "dist", "k2", tmp.k2)) return false;
    if (!readNestedNumber(contents, "dist", "p1", tmp.p1)) return false;
    if (!readNestedNumber(contents, "dist", "p2", tmp.p2)) return false;
    if (!readNestedNumber(contents, "dist", "k3", tmp.k3)) return false;
    // k4..k6 are part of the schema but default to 0.0 if absent (older
    // offline-script outputs predate the rational-model extension). We try
    // to read them but accept absence silently — the struct already has 0.0
    // defaults that match the schema's "k4": 0.0 baseline.
    {
        double tmp_v = 0.0;
        const std::string parent = "dist";
        // Reuse readNestedNumber but suppress the missing-key log by
        // pre-checking presence with findKey.
        const size_t parent_pos = findKey(contents, parent);
        if (parent_pos != std::string::npos &&
            parent_pos < contents.size() && contents[parent_pos] == '{') {
            int depth = 0;
            size_t end = parent_pos;
            for (; end < contents.size(); ++end) {
                if (contents[end] == '{') ++depth;
                else if (contents[end] == '}') {
                    --depth;
                    if (depth == 0) break;
                }
            }
            if (depth == 0) {
                const std::string sub =
                    contents.substr(parent_pos, end - parent_pos + 1);
                size_t cp = findKey(sub, "k4");
                if (cp != std::string::npos && parseNumber(sub, cp, tmp_v)) tmp.k4 = tmp_v;
                cp = findKey(sub, "k5");
                if (cp != std::string::npos && parseNumber(sub, cp, tmp_v)) tmp.k5 = tmp_v;
                cp = findKey(sub, "k6");
                if (cp != std::string::npos && parseNumber(sub, cp, tmp_v)) tmp.k6 = tmp_v;
            }
        }
    }

    if (!readNumber(contents, "rms_reprojection_error_px", tmp.rms_px)) {
        return false;
    }
    if (!(tmp.rms_px >= 0.0) || tmp.rms_px > 1.0) {
        LOGE("loadCalibrationFromJson: RMS %.3f px outside [0, 1.0]",
             tmp.rms_px);
        return false;
    }

    out = tmp;
    LOGI("loadCalibrationFromJson: ok path='%s' fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
         "k1=%.4f k2=%.4f p1=%.4f p2=%.4f k3=%.4f rms=%.3fpx size=%dx%d",
         path.c_str(), out.fx, out.fy, out.cx, out.cy,
         out.k1, out.k2, out.p1, out.p2, out.k3, out.rms_px,
         out.image_w, out.image_h);
    return true;
}
