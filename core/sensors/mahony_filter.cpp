#include "sensors/mahony_filter.hpp"

#include <cmath>

namespace scand3d::sensors {
namespace {
// Magnetometer correction is scaled down: its heading is noisier than gravity, and a full-weight
// term makes yaw twitch whenever a metal object comes near the phone.
constexpr double kMagWeight = 0.5;
}  // namespace

MahonyFilter::MahonyFilter(double kp) : kp_(kp) {}

void MahonyFilter::reset() {
    q_ = identityQuat();
}

void MahonyFilter::setOrientation(const Quat& q) {
    q_ = isFiniteQuat(q) ? quatNormalize(q) : identityQuat();
}

bool MahonyFilter::update(const ImuSample& sample, double dt_s) {
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
        return false;
    }

    Vec3 omega{0.0, 0.0, 0.0};
    if (sample.has_gyro && std::isfinite(sample.gyro_rad_s[0]) &&
        std::isfinite(sample.gyro_rad_s[1]) && std::isfinite(sample.gyro_rad_s[2])) {
        omega = sample.gyro_rad_s;
    }

    Vec3 correction{0.0, 0.0, 0.0};

    if (sample.has_accel) {
        Vec3 measured = sample.accel_m_s2;
        if (normalize(measured)) {
            // Body-frame direction the accelerometer reports (it measures specific force, so at
            // rest it points along world up).
            const Vec3 expected_up_body = quatRotate(quatConjugate(q_), kWorldUp);
            const Vec3 error = cross(measured, expected_up_body);
            correction = correction + error;
        }
    }

    if (sample.has_accel && sample.has_mag) {
        Vec3 measured_mag = sample.mag_u_t;
        if (normalize(measured_mag)) {
            const Vec3 mag_world = quatRotate(q_, measured_mag);
            // Keep the measured inclination but drop the east component: only the horizontal
            // part of the field carries heading information.
            Vec3 desired{std::hypot(mag_world[0], mag_world[1]), 0.0, mag_world[2]};
            if (normalize(desired)) {
                const Vec3 error_world = cross(mag_world, desired);
                const Vec3 error_body = quatRotate(quatConjugate(q_), error_world);
                correction = correction + error_body * kMagWeight;
            }
        }
    }

    omega = omega + correction * kp_;

    // q_dot = 0.5 * q (x) [0, omega]
    const Quat omega_quat{0.0, omega[0], omega[1], omega[2]};
    const Quat q_dot = quatMultiply(q_, omega_quat);
    q_ = quatNormalize({
        q_[0] + 0.5 * q_dot[0] * dt_s,
        q_[1] + 0.5 * q_dot[1] * dt_s,
        q_[2] + 0.5 * q_dot[2] * dt_s,
        q_[3] + 0.5 * q_dot[3] * dt_s,
    });
    return true;
}

}  // namespace scand3d::sensors
