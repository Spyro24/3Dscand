#include "bridge/bridge_api.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "overlay/overlay_scene.hpp"
#include "overlay/point_color.hpp"
#include "sensors/keyframe_prior.hpp"
#include "sensors/mahony_filter.hpp"
#include "sensors/tilt_orientation.hpp"

namespace scand3d::bridge {
namespace {
// Bump together with the app's versionName; the bridge stays a skeleton until Phase 1.
constexpr char kCoreVersion[] = "0.1.0";

constexpr double kFilterGain = 0.6;
// The gravity/magnetometer fallback has no gyro to integrate, so each sample only nudges the
// estimate towards the measured attitude.
constexpr double kFallbackSmoothing = 0.15;
constexpr double kMinDtS = 0.0005;
constexpr double kMaxDtS = 0.1;
constexpr std::int64_t kEarliestTimestampNs = 1;  // zero or below cannot be a clock reading

/// One live fusion session: the filter plus the timing state needed to turn sample timestamps
/// into integration steps.
struct FusionSession {
    explicit FusionSession(double sample_rate_hz)
        : filter(kFilterGain),
          default_dt_s(1.0 / (sample_rate_hz > 1.0 ? sample_rate_hz : 200.0)) {}

    sensors::MahonyFilter filter;
    double default_dt_s = 0.005;
    std::int64_t last_timestamp_ns = 0;
    bool has_orientation = false;
};

std::mutex g_mutex;
std::unordered_map<ImuHandle, std::unique_ptr<FusionSession>> g_sessions;
std::atomic<ImuHandle> g_next_handle{1};

// Overlay scenes are touched by the sensor thread (pose) and the UI thread (points), and a
// projection runs on every pose sample, so they keep their own lock instead of serialising
// against sensor fusion.
std::mutex g_overlay_mutex;
std::unordered_map<OverlayHandle, overlay::OverlayScene> g_overlay_scenes;
std::atomic<OverlayHandle> g_next_overlay_handle{1};

sensors::Quat quatFromArray(const double values[4]) {
    return {values[0], values[1], values[2], values[3]};
}

}  // namespace

const char* coreVersion() {
    return kCoreVersion;
}

ImuHandle imuCreate(double sample_rate_hz) {
    const ImuHandle handle = g_next_handle.fetch_add(1);
    std::lock_guard<std::mutex> lock(g_mutex);
    g_sessions.emplace(handle, std::make_unique<FusionSession>(sample_rate_hz));
    return handle;
}

void imuDestroy(ImuHandle handle) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_sessions.erase(handle);
}

void imuReset(ImuHandle handle) {
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto it = g_sessions.find(handle);
    if (it == g_sessions.end()) {
        return;
    }
    it->second->filter.reset();
    it->second->last_timestamp_ns = 0;
    it->second->has_orientation = false;
}

bool imuUpdate(ImuHandle handle, const double* values, int value_count) {
    if (values == nullptr || value_count < kImuSampleValueCount) {
        return false;
    }
    for (int i = 0; i < kImuSampleValueCount; ++i) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }

    sensors::ImuSample sample;
    if (values[0] <= static_cast<double>(kEarliestTimestampNs)) {
        return false;
    }
    sample.timestamp_ns = static_cast<std::int64_t>(values[0]);
    sample.gyro_rad_s = {values[1], values[2], values[3]};
    sample.accel_m_s2 = {values[4], values[5], values[6]};
    sample.mag_u_t = {values[7], values[8], values[9]};
    const int flags = static_cast<int>(values[10]);
    sample.has_gyro = (flags & 1) != 0;
    sample.has_accel = (flags & 2) != 0;
    sample.has_mag = (flags & 4) != 0;

    std::lock_guard<std::mutex> lock(g_mutex);
    const auto it = g_sessions.find(handle);
    if (it == g_sessions.end()) {
        return false;
    }
    FusionSession& session = *it->second;

    double dt_s = session.default_dt_s;
    if (session.last_timestamp_ns > 0 && sample.timestamp_ns > session.last_timestamp_ns) {
        dt_s = static_cast<double>(sample.timestamp_ns - session.last_timestamp_ns) * 1e-9;
        dt_s = std::min(std::max(dt_s, kMinDtS), kMaxDtS);
    }
    session.last_timestamp_ns = sample.timestamp_ns;

    bool updated = false;
    const sensors::Vec3* mag = sample.has_mag ? &sample.mag_u_t : nullptr;

    if (sample.has_gyro) {
        if (!session.has_orientation && sample.has_accel) {
            // Seed pitch and roll so a session that starts tilted does not sweep there first.
            sensors::Quat seed{};
            if (sensors::tiltOrientation(sample.accel_m_s2, mag, &seed)) {
                session.filter.setOrientation(seed);
            }
        }
        updated = session.filter.update(sample, dt_s);
    } else if (sample.has_accel) {
        // Fallback path (roadmap 2.5): gravity and north define the orientation outright.
        sensors::Quat target{};
        if (sensors::tiltOrientation(sample.accel_m_s2, mag, &target)) {
            const sensors::Quat smoothed = session.has_orientation
                ? sensors::quatNlerp(session.filter.orientation(), target, kFallbackSmoothing)
                : target;
            session.filter.setOrientation(smoothed);
            updated = true;
        }
    }

    if (updated) {
        session.has_orientation = true;
    }
    return updated;
}

bool imuOrientation(ImuHandle handle, double out_quaternion[4]) {
    if (out_quaternion == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto it = g_sessions.find(handle);
    if (it == g_sessions.end() || !it->second->has_orientation) {
        return false;
    }
    const sensors::Quat& q = it->second->filter.orientation();
    for (int i = 0; i < 4; ++i) {
        out_quaternion[i] = q[i];
    }
    return true;
}

double angularDisplacementRad(const double a[4], const double b[4]) {
    if (a == nullptr || b == nullptr) {
        return std::nan("");
    }
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
            return std::nan("");
        }
    }
    return sensors::quatAngleRad(quatFromArray(a), quatFromArray(b));
}

bool keyframeBaselineAccept(
    const double reference[4],
    const double candidate[4],
    double min_baseline_rad,
    double max_baseline_rad) {
    if (reference == nullptr || candidate == nullptr) {
        return false;
    }
    sensors::KeyframePriorPolicy policy;
    policy.min_baseline_rad = min_baseline_rad;
    policy.max_baseline_rad = max_baseline_rad;
    return sensors::acceptKeyframe(
        quatFromArray(reference), quatFromArray(candidate), policy);
}

OverlayHandle overlayCreate() {
    const OverlayHandle handle = g_next_overlay_handle.fetch_add(1);
    std::lock_guard<std::mutex> lock(g_overlay_mutex);
    g_overlay_scenes.emplace(handle, overlay::OverlayScene{});
    return handle;
}

void overlayDestroy(OverlayHandle handle) {
    std::lock_guard<std::mutex> lock(g_overlay_mutex);
    g_overlay_scenes.erase(handle);
}

bool overlaySetPoints(
    OverlayHandle handle,
    const double* positions_m,
    const float* rgb,
    int count) {
    if (count < 0 || (count > 0 && (positions_m == nullptr || rgb == nullptr))) {
        return false;
    }

    std::vector<overlay::OverlayPoint> points;
    points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        overlay::OverlayPoint point;
        for (int axis = 0; axis < 3; ++axis) {
            const double value = positions_m[3 * i + axis];
            const float colour = rgb[3 * i + axis];
            if (!std::isfinite(value) || !std::isfinite(colour)) {
                return false;
            }
            point.position_m[static_cast<std::size_t>(axis)] = value;
            point.rgb[axis] = colour < 0.0f ? 0.0f : (colour > 1.0f ? 1.0f : colour);
        }
        points.push_back(point);
    }

    std::lock_guard<std::mutex> lock(g_overlay_mutex);
    const auto it = g_overlay_scenes.find(handle);
    if (it == g_overlay_scenes.end()) {
        return false;
    }
    it->second.setPoints(std::move(points));
    return true;
}

int overlayPointCount(OverlayHandle handle) {
    std::lock_guard<std::mutex> lock(g_overlay_mutex);
    const auto it = g_overlay_scenes.find(handle);
    if (it == g_overlay_scenes.end()) {
        return 0;
    }
    return static_cast<int>(it->second.pointCount());
}

int overlayProject(
    OverlayHandle handle,
    const OverlayCamera& camera,
    float* out_vertices,
    int max_vertices) {
    if (out_vertices == nullptr || max_vertices <= 0) {
        return 0;
    }

    overlay::OverlayCamera input;
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(camera.rotation[i])) {
            return 0;
        }
        input.rotation[static_cast<std::size_t>(i)] = camera.rotation[i];
    }
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(camera.centre_m[i])) {
            return 0;
        }
        input.centre_m[static_cast<std::size_t>(i)] = camera.centre_m[i];
    }
    input.focal_px = camera.focal_px;
    input.cx_px = camera.cx_px;
    input.cy_px = camera.cy_px;
    input.image_width_px = camera.image_width_px;
    input.image_height_px = camera.image_height_px;

    std::vector<overlay::ColoredVertex> vertices;
    {
        std::lock_guard<std::mutex> lock(g_overlay_mutex);
        const auto it = g_overlay_scenes.find(handle);
        if (it == g_overlay_scenes.end()) {
            return 0;
        }
        vertices = it->second.project(input);
    }

    const int written = std::min(static_cast<int>(vertices.size()), max_vertices);
    for (int i = 0; i < written; ++i) {
        const overlay::ColoredVertex& vertex = vertices[static_cast<std::size_t>(i)];
        float* out = out_vertices + static_cast<std::ptrdiff_t>(i) * kOverlayVertexValueCount;
        out[0] = vertex.ndc[0];
        out[1] = vertex.ndc[1];
        out[2] = vertex.rgb[0];
        out[3] = vertex.rgb[1];
        out[4] = vertex.rgb[2];
    }
    return written;
}

int overlaySampleColors(
    OverlayHandle handle,
    const OverlayCamera& camera,
    const unsigned char* rgb,
    int image_width_px,
    int image_height_px) {
    if (rgb == nullptr || image_width_px <= 0 || image_height_px <= 0) {
        return 0;
    }

    overlay::RgbImage keyframe;
    keyframe.width_px = image_width_px;
    keyframe.height_px = image_height_px;
    const std::size_t pixel_count =
        static_cast<std::size_t>(image_width_px) * static_cast<std::size_t>(image_height_px);
    keyframe.rgb.assign(rgb, rgb + pixel_count * 3u);

    overlay::OverlayCamera input;
    for (int i = 0; i < 9; ++i) {
        input.rotation[static_cast<std::size_t>(i)] = camera.rotation[i];
    }
    for (int i = 0; i < 3; ++i) {
        input.centre_m[static_cast<std::size_t>(i)] = camera.centre_m[i];
    }
    input.focal_px = camera.focal_px;
    input.cx_px = camera.cx_px;
    input.cy_px = camera.cy_px;
    input.image_width_px = camera.image_width_px;
    input.image_height_px = camera.image_height_px;

    std::lock_guard<std::mutex> lock(g_overlay_mutex);
    const auto it = g_overlay_scenes.find(handle);
    if (it == g_overlay_scenes.end()) {
        return 0;
    }
    std::vector<overlay::OverlayPoint> points = it->second.points();
    const int sampled = overlay::samplePointColors(keyframe, input, points);
    if (sampled > 0) {
        it->second.setPoints(std::move(points));
    }
    return sampled;
}

}  // namespace scand3d::bridge
