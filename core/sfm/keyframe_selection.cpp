#include "sfm/keyframe_selection.hpp"

#include <algorithm>
#include <cmath>

namespace scand3d::sfm {

double rotationAngleRad(const Mat3& first, const Mat3& second) {
    // trace(R1^T R2) = 1 + 2 cos(angle)
    const Mat3 relative = geometry::matmul3(geometry::transpose3(first), second);
    const double trace = relative[0] + relative[4] + relative[8];
    const double cosine = std::clamp(0.5 * (trace - 1.0), -1.0, 1.0);
    return std::acos(cosine);
}

double baselineM(const Pose& first, const Pose& second) {
    const Vec3 delta{second.camera_centre_m[0] - first.camera_centre_m[0],
                     second.camera_centre_m[1] - first.camera_centre_m[1],
                     second.camera_centre_m[2] - first.camera_centre_m[2]};
    return std::sqrt(geometry::dot3(delta, delta));
}

KeyframeDecision evaluateKeyframe(
    double rotation_rad,
    double baseline_m,
    double overlap,
    const KeyframePolicy& policy) {
    KeyframeDecision decision;
    decision.rotation_rad = rotation_rad;
    decision.baseline_m = baseline_m;
    decision.overlap = overlap;

    if (!std::isfinite(rotation_rad) || !std::isfinite(baseline_m) || !std::isfinite(overlap)) {
        decision.reason = "non-finite measurement";
        return decision;
    }
    if (rotation_rad < policy.min_rotation_rad) {
        decision.reason = "too little rotation";
        return decision;
    }
    if (policy.max_rotation_rad > 0.0 && rotation_rad > policy.max_rotation_rad) {
        decision.reason = "too much rotation";
        return decision;
    }
    if (baseline_m < policy.min_baseline_m) {
        decision.reason = "baseline too short";
        return decision;
    }
    if (policy.max_baseline_m > 0.0 && baseline_m > policy.max_baseline_m) {
        decision.reason = "baseline too long";
        return decision;
    }
    if (overlap < policy.min_overlap) {
        decision.reason = "too little overlap";
        return decision;
    }

    decision.accept = true;
    decision.reason = "accepted";
    return decision;
}

}  // namespace scand3d::sfm
