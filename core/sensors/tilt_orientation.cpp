#include "sensors/tilt_orientation.hpp"

#include <cmath>

namespace scand3d::sensors {
namespace {

/// Any unit vector perpendicular to `up`, used when there is no magnetic heading to observe.
/// Projecting world X onto the horizontal plane first means a level phone keeps yaw at zero
/// instead of starting from an arbitrary heading.
Vec3 perpendicularTo(const Vec3& up) {
    Vec3 reference{1.0, 0.0, 0.0};
    if (std::fabs(dot(up, reference)) > 0.9) {
        reference = {0.0, 1.0, 0.0};
    }
    Vec3 north = reference - up * dot(up, reference);
    normalize(north);
    return north;
}

}  // namespace

bool tiltOrientation(const Vec3& accel_m_s2, const Vec3* mag_u_t, Quat* out_quaternion) {
    if (out_quaternion == nullptr) {
        return false;
    }

    Vec3 up_body = accel_m_s2;
    if (!normalize(up_body)) {
        return false;  // free fall: gravity says nothing about orientation
    }

    Vec3 north_body = perpendicularTo(up_body);
    if (mag_u_t != nullptr) {
        Vec3 mag = *mag_u_t;
        if (normalize(mag)) {
            // Remove the vertical component: only the horizontal part carries heading.
            Vec3 horizontal = mag - up_body * dot(mag, up_body);
            if (normalize(horizontal)) {
                north_body = horizontal;
            }
        }
    }

    // Right-handed triad in the body frame: north, east, up with east = up x north.
    const Vec3 east_body = cross(up_body, north_body);

    // Columns are the world basis vectors expressed in body coordinates, so the transpose maps
    // body to world.
    const Mat3 rotation{
        north_body[0], north_body[1], north_body[2],
        east_body[0],  east_body[1],  east_body[2],
        up_body[0],    up_body[1],    up_body[2],
    };

    *out_quaternion = quatFromMatrix(rotation);
    return isFiniteQuat(*out_quaternion);
}

}  // namespace scand3d::sensors
