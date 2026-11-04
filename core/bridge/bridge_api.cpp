#include "bridge/bridge_api.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "feature/gray_image.hpp"
#include "geometry/linalg.hpp"
#include "mvs/pipeline.hpp"
#include "overlay/overlay_scene.hpp"
#include "overlay/object_clusters.hpp"
#include "overlay/point_color.hpp"
#include "sfm/session.hpp"
#include "sfm/tracker.hpp"
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

// A reconstruction run takes seconds, so it keeps its own lock and never blocks the IMU or the
// overlay path.
std::mutex g_reconstruction_mutex;
std::unordered_map<ReconstructionHandle, std::unique_ptr<sfm::Session>> g_reconstructions;
std::atomic<ReconstructionHandle> g_next_reconstruction_handle{1};

// Trackers own a snapshot of a map, so they need a lock of their own: tracking runs on every
// preview frame while a reconstruction run holds its lock for seconds (D051).
std::mutex g_tracker_mutex;
std::unordered_map<TrackerHandle, std::shared_ptr<const sfm::Tracker>> g_trackers;
std::atomic<TrackerHandle> g_next_tracker_handle{1};

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

ReconstructionHandle reconstructionCreate() {
    const ReconstructionHandle handle = g_next_reconstruction_handle.fetch_add(1);
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    g_reconstructions.emplace(handle, std::make_unique<sfm::Session>());
    return handle;
}

void reconstructionDestroy(ReconstructionHandle handle) {
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    g_reconstructions.erase(handle);
}

int reconstructionAddView(
    ReconstructionHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px) {
    if (gray == nullptr) {
        return -1;
    }
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    const auto it = g_reconstructions.find(handle);
    if (it == g_reconstructions.end()) {
        return -1;
    }
    return it->second->addView(width_px, height_px, gray, focal_px, cx_px, cy_px);
}

int reconstructionRun(ReconstructionHandle handle) {
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    const auto it = g_reconstructions.find(handle);
    if (it == g_reconstructions.end()) {
        return -1;
    }
    return it->second->run().views_registered;
}

int reconstructionPointCount(ReconstructionHandle handle) {
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    const auto it = g_reconstructions.find(handle);
    if (it == g_reconstructions.end()) {
        return 0;
    }
    return it->second->pointCount();
}

int reconstructionCopyPoints(ReconstructionHandle handle, double* positions_m, int max_points) {
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    const auto it = g_reconstructions.find(handle);
    if (it == g_reconstructions.end()) {
        return 0;
    }
    return it->second->copyPoints(positions_m, max_points);
}

bool reconstructionCopyCamera(
    ReconstructionHandle handle,
    int index,
    double out_rotation[9],
    double out_centre_m[3]) {
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    const auto it = g_reconstructions.find(handle);
    if (it == g_reconstructions.end()) {
        return false;
    }
    return it->second->copyCamera(index, out_rotation, out_centre_m);
}

int reconstructionTrackPose(
    ReconstructionHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px,
    double out_rotation[9],
    double out_centre_m[3]) {
    if (gray == nullptr || out_rotation == nullptr || out_centre_m == nullptr) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
    const auto it = g_reconstructions.find(handle);
    if (it == g_reconstructions.end()) {
        return 0;
    }
    sfm::Pose pose;
    int inliers = 0;
    if (!it->second->trackPose(width_px, height_px, gray, focal_px, cx_px, cy_px, pose, inliers)) {
        return 0;
    }
    for (int i = 0; i < 9; ++i) {
        out_rotation[i] = pose.rotation[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 3; ++i) {
        out_centre_m[i] = pose.camera_centre_m[static_cast<std::size_t>(i)];
    }
    return inliers;
}

TrackerHandle trackerCreate(ReconstructionHandle handle) {
    std::shared_ptr<const sfm::Tracker> snapshot;
    {
        std::lock_guard<std::mutex> lock(g_reconstruction_mutex);
        const auto it = g_reconstructions.find(handle);
        if (it == g_reconstructions.end()) {
            return 0;
        }
        // Copied under the reconstruction lock on purpose: the snapshot has to be of one map, not
        // of a run that is halfway through rewriting it.
        snapshot = it->second->snapshot();
    }
    if (snapshot == nullptr || snapshot->empty()) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(g_tracker_mutex);
    const TrackerHandle tracker = g_next_tracker_handle++;
    g_trackers.emplace(tracker, std::move(snapshot));
    return tracker;
}

int trackerTrack(
    TrackerHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px,
    double out_rotation[9],
    double out_centre_m[3]) {
    if (gray == nullptr || out_rotation == nullptr || out_centre_m == nullptr) {
        return 0;
    }
    std::shared_ptr<const sfm::Tracker> tracker;
    {
        std::lock_guard<std::mutex> lock(g_tracker_mutex);
        const auto it = g_trackers.find(handle);
        if (it == g_trackers.end()) {
            return 0;
        }
        // Held by shared_ptr so a destroy on the UI thread cannot free it mid-frame.
        tracker = it->second;
    }

    sfm::Pose pose;
    int inliers = 0;
    if (!tracker->track(width_px, height_px, gray, focal_px, cx_px, cy_px, pose, inliers)) {
        return 0;
    }
    for (int i = 0; i < 9; ++i) {
        out_rotation[i] = pose.rotation[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 3; ++i) {
        out_centre_m[i] = pose.camera_centre_m[static_cast<std::size_t>(i)];
    }
    return inliers;
}

void trackerDestroy(TrackerHandle handle) {
    std::lock_guard<std::mutex> lock(g_tracker_mutex);
    g_trackers.erase(handle);
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
    // Then recolour per object: the photo colour keeps isolated detail readable, and the cluster
    // colour is what shows which points belong to which surface (D055).
    const overlay::ObjectClusterResult clusters = overlay::colourByObject(points);
    if (sampled > 0 || clusters.objects > 0) {
        it->second.setPoints(std::move(points));
    }
    return sampled;
}

namespace {

/// One dense session (roadmap 6). The run itself can take minutes, so the views, the cancel flag
/// and the progress live in a shared state that is reachable without the global handle lock: that
/// is what lets `denseCancel` and `denseProgress` answer while `denseRun` blocks.
struct DenseState {
    std::mutex views_mutex;
    std::vector<mvs::MvsViewInput> views;
    std::vector<geometry::Vec3> hints;
    std::vector<mvs::FusedPoint> points;
    std::atomic<bool> cancel{false};
    std::atomic<double> progress{0.0};
    std::mutex run_mutex;
};

std::mutex g_dense_mutex;
std::unordered_map<DenseHandle, std::shared_ptr<DenseState>> g_dense_sessions;
std::atomic<DenseHandle> g_next_dense_handle{1};

std::shared_ptr<DenseState> denseStateOf(DenseHandle handle) {
    std::lock_guard<std::mutex> lock(g_dense_mutex);
    const auto found = g_dense_sessions.find(handle);
    return found == g_dense_sessions.end() ? nullptr : found->second;
}

}  // namespace

DenseHandle denseCreate() {
    const DenseHandle handle = g_next_dense_handle.fetch_add(1);
    auto state = std::make_shared<DenseState>();
    std::lock_guard<std::mutex> lock(g_dense_mutex);
    g_dense_sessions.emplace(handle, std::move(state));
    return handle;
}

void denseDestroy(DenseHandle handle) {
    std::shared_ptr<DenseState> state;
    {
        std::lock_guard<std::mutex> lock(g_dense_mutex);
        const auto found = g_dense_sessions.find(handle);
        if (found == g_dense_sessions.end()) {
            return;
        }
        state = found->second;
        g_dense_sessions.erase(found);
    }
    // A run that is still going without its entry must stop instead of finishing into nothing.
    state->cancel.store(true);
}

int denseAddView(
    DenseHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px,
    const double rotation[9],
    const double centre_m[3]) {
    if (gray == nullptr || rotation == nullptr || centre_m == nullptr || width_px <= 0 ||
        height_px <= 0 || !(focal_px > 0.0)) {
        return -1;
    }
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    if (state == nullptr) {
        return -1;
    }

    mvs::MvsViewInput view;
    view.gray.width_px = width_px;
    view.gray.height_px = height_px;
    const std::size_t pixel_count =
        static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px);
    view.gray.pixels.assign(gray, gray + pixel_count);
    view.camera = mvs::CameraModel{focal_px, cx_px, cy_px};
    for (int i = 0; i < 9; ++i) {
        view.pose.rotation[static_cast<std::size_t>(i)] = rotation[i];
    }
    for (int i = 0; i < 3; ++i) {
        view.pose.camera_centre_m[static_cast<std::size_t>(i)] = centre_m[i];
    }

    std::lock_guard<std::mutex> lock(state->views_mutex);
    state->views.push_back(std::move(view));
    return static_cast<int>(state->views.size()) - 1;
}

void denseSetHints(DenseHandle handle, const double* positions_m, int count) {
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    if (state == nullptr || count < 0 || (count > 0 && positions_m == nullptr)) {
        return;
    }
    std::vector<geometry::Vec3> hints;
    hints.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double* point = positions_m + static_cast<std::ptrdiff_t>(i) * 3;
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
            continue;
        }
        hints.push_back(geometry::Vec3{point[0], point[1], point[2]});
    }
    std::lock_guard<std::mutex> lock(state->views_mutex);
    state->hints = std::move(hints);
}

int denseRun(
    DenseHandle handle,
    int working_width_px,
    double min_depth_m,
    double max_depth_m) {
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    if (state == nullptr) {
        return -1;
    }
    std::lock_guard<std::mutex> run_lock(state->run_mutex);
    state->cancel.store(false);
    state->progress.store(0.0);

    std::vector<mvs::MvsViewInput> views;
    std::vector<geometry::Vec3> hints;
    {
        std::lock_guard<std::mutex> lock(state->views_mutex);
        views = state->views;
        hints = state->hints;
    }
    if (views.size() < 2) {
        state->progress.store(1.0);
        return 0;
    }

    mvs::MvsOptions options;
    options.working_width_px = working_width_px > 0 ? working_width_px : 320;
    if (min_depth_m > 0.0 && max_depth_m > min_depth_m) {
        options.patch_match.min_depth_m = min_depth_m;
        options.patch_match.max_depth_m = max_depth_m;
    }
    options.depth_hint_points_m = std::move(hints);

    mvs::MvsControl control;
    control.cancel = &state->cancel;
    control.progress = [&state](double fraction) { state->progress.store(fraction); };

    const mvs::MvsResult result = mvs::reconstructDense(views, options, control);
    if (result.cancelled) {
        state->progress.store(0.0);
        return 0;
    }

    std::lock_guard<std::mutex> lock(state->views_mutex);
    state->points = result.points;
    state->progress.store(1.0);
    return static_cast<int>(state->points.size());
}

int densePointCount(DenseHandle handle) {
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    if (state == nullptr) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(state->views_mutex);
    return static_cast<int>(state->points.size());
}

int denseCopyPoints(DenseHandle handle, double* positions_m, int max_points) {
    if (positions_m == nullptr || max_points <= 0) {
        return 0;
    }
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    if (state == nullptr) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(state->views_mutex);
    const int written =
        std::min(static_cast<int>(state->points.size()), max_points);
    for (int i = 0; i < written; ++i) {
        const mvs::FusedPoint& point = state->points[static_cast<std::size_t>(i)];
        for (int axis = 0; axis < 3; ++axis) {
            positions_m[3 * i + axis] = point.position_m[static_cast<std::size_t>(axis)];
        }
    }
    return written;
}

double denseProgress(DenseHandle handle) {
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    return state == nullptr ? 0.0 : state->progress.load();
}

void denseCancel(DenseHandle handle) {
    const std::shared_ptr<DenseState> state = denseStateOf(handle);
    if (state != nullptr) {
        state->cancel.store(true);
    }
}

}  // namespace scand3d::bridge
