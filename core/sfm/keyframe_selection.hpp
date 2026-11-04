#pragma once

#include "sfm/types.hpp"

namespace scand3d::sfm {

/// Rules for admitting a view into the reconstruction (roadmap 4.3). The rotation window is the
/// IMU prior's shape (D018); overlap and baseline come from the reconstructed map.
struct KeyframePolicy {
    double min_rotation_rad = 0.10;  // about 5.7 degrees
    double max_rotation_rad = 0.70;  // about 40 degrees
    double min_baseline_m = 0.01;
    double max_baseline_m = 0.0;  // 0 disables the upper bound
    double min_overlap = 0.20;    // fraction of the candidate's features that already have a point
};

struct KeyframeDecision {
    int view = -1;
    bool accept = false;
    double rotation_rad = 0.0;
    double baseline_m = 0.0;
    double overlap = 0.0;
    /// Why the candidate was rejected, for the reconstruction log.
    const char* reason = "";
};

/// Rotation angle between two camera-to-world rotations, in radians.
double rotationAngleRad(const Mat3& first, const Mat3& second);

/// Distance between two camera centres, in meters.
double baselineM(const Pose& first, const Pose& second);

/// Applies the policy. `overlap` is the fraction of the candidate's features whose track already
/// has a triangulated point; pass 1.0 when unknown.
KeyframeDecision evaluateKeyframe(
    double rotation_rad,
    double baseline_m,
    double overlap,
    const KeyframePolicy& policy);

}  // namespace scand3d::sfm
