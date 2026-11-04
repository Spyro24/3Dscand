#pragma once

#include <cstdint>

namespace scand3d::bridge {

/// Version of the native core. Single source of truth for the value the UI displays and the
/// native smoke test checks.
const char* coreVersion();

/// Opaque handle to one sensor-fusion session. 0 is never a valid handle.
using ImuHandle = std::uint64_t;

/// Number of doubles one sample occupies:
/// [timestamp_ns, gx, gy, gz, ax, ay, az, mx, my, mz, flags]
/// with flags bit0 = gyroscope, bit1 = accelerometer, bit2 = magnetometer present.
constexpr int kImuSampleValueCount = 11;

/// Starts a fusion session for the given sensor rate. The rate only seeds the first time step;
/// afterwards the sample timestamps decide dt, so a jittery sensor does not skew integration.
ImuHandle imuCreate(double sample_rate_hz);

/// Frees a session. Destroying an unknown or already-destroyed handle is a no-op.
void imuDestroy(ImuHandle handle);

/// Returns the session to its initial state, e.g. when the camera restarts.
void imuReset(ImuHandle handle);

/// Feeds one sample. Returns false when the handle is unknown or the values are unusable, in
/// which case the previous orientation is kept.
bool imuUpdate(ImuHandle handle, const double* values, int value_count);

/// Copies the orientation as {w, x, y, z} into `out_quaternion`. False until a first usable
/// orientation exists.
bool imuOrientation(ImuHandle handle, double out_quaternion[4]);

/// Rotation angle between two quaternions in radians, or NaN for non-finite input.
double angularDisplacementRad(const double a[4], const double b[4]);

/// True when the rotation between the two orientations falls inside the keyframe window.
bool keyframeBaselineAccept(
    const double reference[4],
    const double candidate[4],
    double min_baseline_rad,
    double max_baseline_rad);

/// Opaque handle to one overlay scene. 0 is never a valid handle.
using OverlayHandle = std::uint64_t;

/// Floats per vertex written by overlayProject: {x_ndc, y_ndc, r, g, b} (roadmap 5.1).
constexpr int kOverlayVertexValueCount = 5;

/// Camera and image the overlay is projected into. `rotation` is camera-to-world, row-major 3x3;
/// the pinhole model is in displayed preview pixels, and the image size is the one those
/// intrinsics describe (D031).
struct OverlayCamera {
    double rotation[9];
    double centre_m[3];
    double focal_px;
    double cx_px;
    double cy_px;
    int image_width_px;
    int image_height_px;
};

/// Creates an empty scene. The caller owns the handle and must destroy it.
OverlayHandle overlayCreate();

/// Frees a scene. Destroying an unknown or already-destroyed handle is a no-op.
void overlayDestroy(OverlayHandle handle);

/// Replaces every point: `positions_m` is xyz per point, `rgb` is 0..1 per point. False on
/// non-finite or negative input, which leaves the scene untouched.
bool overlaySetPoints(
    OverlayHandle handle,
    const double* positions_m,
    const float* rgb,
    int count);

/// Number of points in the scene, or 0 for an unknown handle.
int overlayPointCount(OverlayHandle handle);

/// Writes up to `max_vertices` visible vertices, far to near. Returns the number written.
int overlayProject(
    OverlayHandle handle,
    const OverlayCamera& camera,
    float* out_vertices,
    int max_vertices);

}  // namespace scand3d::bridge
