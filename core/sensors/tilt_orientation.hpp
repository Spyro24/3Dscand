#pragma once

#include "sensors/imu_types.hpp"

namespace scand3d::sensors {

/// Orientation from gravity alone (plus magnetic north when available). This is the fallback for
/// devices without a usable gyroscope (roadmap 2.5): pitch and roll are exact, yaw is either
/// measured from the magnetometer or reported as zero because there is nothing that observes it.
///
/// `accel_m_s2` is the measured specific force; at rest it points along world up. `mag_u_t` may be
/// null, and a magnetic vector parallel to gravity is ignored rather than producing a NaN basis.
bool tiltOrientation(const Vec3& accel_m_s2, const Vec3* mag_u_t, Quat* out_quaternion);

}  // namespace scand3d::sensors
