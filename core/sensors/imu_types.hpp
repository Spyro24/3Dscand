#pragma once

#include <array>
#include <cmath>
#include <cstdint>

// Shared math for the IMU modules. Conventions follow agents.md section 3: right-handed,
// radians, float64. A quaternion is {w, x, y, z} and rotates body (device) coordinates into
// world coordinates, so `quatRotate(q, v_body) == v_world`.
namespace scand3d::sensors {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major
using Quat = std::array<double, 4>;  // w, x, y, z

/// One fused sensor reading. Each block is optional: a device without a magnetometer still
/// produces usable samples, so the flags travel with the values instead of being inferred.
struct ImuSample {
    std::int64_t timestamp_ns = 0;
    Vec3 gyro_rad_s{0.0, 0.0, 0.0};
    Vec3 accel_m_s2{0.0, 0.0, 0.0};
    Vec3 mag_u_t{0.0, 0.0, 0.0};
    bool has_gyro = false;
    bool has_accel = false;
    bool has_mag = false;
};

inline constexpr Vec3 kWorldUp{0.0, 0.0, 1.0};
inline constexpr Vec3 kWorldNorth{1.0, 0.0, 0.0};

inline double length(const Vec3& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline bool normalize(Vec3& v) {
    const double n = length(v);
    if (!(n > 1e-12) || !std::isfinite(n)) {
        return false;
    }
    v = {v[0] / n, v[1] / n, v[2] / n};
    return true;
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline Vec3 operator*(const Vec3& v, double s) { return {v[0] * s, v[1] * s, v[2] * s}; }

inline Quat identityQuat() { return {1.0, 0.0, 0.0, 0.0}; }

inline Quat quatMultiply(const Quat& a, const Quat& b) {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
}

inline Quat quatConjugate(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }

inline Quat quatNormalize(Quat q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(n > 1e-12) || !std::isfinite(n)) {
        return identityQuat();
    }
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

inline Quat quatFromAxisAngle(const Vec3& axis_in, double angle_rad) {
    Vec3 axis = axis_in;
    if (!normalize(axis)) {
        return identityQuat();
    }
    const double half = 0.5 * angle_rad;
    const double s = std::sin(half);
    return {std::cos(half), axis[0] * s, axis[1] * s, axis[2] * s};
}

inline Vec3 quatRotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q[1], q[2], q[3]};
    const Vec3 t = cross(u, v) * 2.0;
    return v + t * q[0] + cross(u, t);
}

/// Rotation angle of `q` applied to a vector, in radians and in [0, pi].
inline double quatAngleRad(const Quat& a, const Quat& b) {
    const Quat relative = quatMultiply(quatConjugate(a), b);
    const double vector_norm =
        std::sqrt(relative[1] * relative[1] + relative[2] * relative[2] + relative[3] * relative[3]);
    return 2.0 * std::atan2(vector_norm, std::fabs(relative[0]));
}

/// Row-major rotation matrix (body to world) to quaternion.
inline Quat quatFromMatrix(const Mat3& m) {
    const double trace = m[0] + m[4] + m[8];
    Quat q{};
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q = {0.25 * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s};
    } else if (m[0] > m[4] && m[0] > m[8]) {
        const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
        q = {(m[7] - m[5]) / s, 0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s};
    } else if (m[4] > m[8]) {
        const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
        q = {(m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s};
    } else {
        const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
        q = {(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s};
    }
    return quatNormalize(q);
}

inline bool isFiniteQuat(const Quat& q) {
    return std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]);
}

/// Normalized linear interpolation, used to smooth the accelerometer/magnetometer fallback
/// where there is no gyro to integrate.
inline Quat quatNlerp(const Quat& a, const Quat& b, double alpha) {
    const double sign = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]) < 0.0 ? -1.0 : 1.0;
    const double t = alpha < 0.0 ? 0.0 : (alpha > 1.0 ? 1.0 : alpha);
    return quatNormalize({
        a[0] + (b[0] * sign - a[0]) * t,
        a[1] + (b[1] * sign - a[1]) * t,
        a[2] + (b[2] * sign - a[2]) * t,
        a[3] + (b[3] * sign - a[3]) * t,
    });
}

}  // namespace scand3d::sensors
