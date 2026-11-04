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

}  // namespace scand3d::bridge
