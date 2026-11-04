#pragma once

#include "sensors/imu_types.hpp"

namespace scand3d::sensors {

/// Rotation window a new keyframe must fall into. Too little rotation adds no triangulation
/// baseline; too much breaks feature overlap. The default window is a starting point for
/// Phase 4 (roadmap 4.3), which will tune it against the synthetic turntable.
struct KeyframePriorPolicy {
    double min_baseline_rad = 0.10;  // about 5.7 degrees
    double max_baseline_rad = 0.70;  // about 40 degrees
};

/// True when `candidate` is rotated far enough from `reference` for a usable baseline, and not
/// so far that overlap is likely lost. The IMU only knows rotation, not translation, so this is a
/// prior and never a final decision (roadmap 2.6).
bool acceptKeyframe(const Quat& reference, const Quat& candidate, const KeyframePriorPolicy& policy);

}  // namespace scand3d::sensors
