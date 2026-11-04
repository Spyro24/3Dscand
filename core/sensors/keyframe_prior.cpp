#include "sensors/keyframe_prior.hpp"

namespace scand3d::sensors {

bool acceptKeyframe(const Quat& reference, const Quat& candidate, const KeyframePriorPolicy& policy) {
    if (!isFiniteQuat(reference) || !isFiniteQuat(candidate)) {
        return false;
    }
    const double angle_rad = quatAngleRad(reference, candidate);
    return angle_rad >= policy.min_baseline_rad && angle_rad <= policy.max_baseline_rad;
}

}  // namespace scand3d::sensors
