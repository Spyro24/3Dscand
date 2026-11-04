#pragma once

#include "sensors/imu_types.hpp"

namespace scand3d::sensors {

/// Mahony complementary AHRS. The gyroscope is integrated and pulled back towards gravity and
/// (when present) magnetic north by a proportional error term, so pitch and roll come from the
/// accelerometer and yaw from the magnetometer. agents.md section 7 leaves Madgwick or Mahony
/// open; Mahony is chosen because its correction step is short enough to audit, and the
/// per-sample cost stays well inside the sensor thread budget (DECISIONS.md D018).
class MahonyFilter {
public:
    explicit MahonyFilter(double kp = 0.6);

    void reset();

    /// Seeds the state without integrating, so a session started while the phone is tilted does
    /// not spend a second converging. Yaw is unknown without a magnetometer, so it stays zero.
    void setOrientation(const Quat& q);

    /// Advances by `dt_s`. Returns false for a non-positive or non-finite step, which happens
    /// on duplicate sensor timestamps; the caller keeps the previous orientation in that case.
    bool update(const ImuSample& sample, double dt_s);

    const Quat& orientation() const { return q_; }

private:
    double kp_;
    Quat q_ = identityQuat();
};

}  // namespace scand3d::sensors
