#pragma once

#include "mvs/types.hpp"

namespace scand3d::mvs {

/// Forward-backward geometric consistency (roadmap 6.2): a depth survives only if the point it
/// describes lands on the neighbour's depth map with the same depth within a tolerance, and the
/// neighbour's own point comes back to the same pixel.
struct ConsistencyOptions {
    /// Relative depth agreement, e.g. 0.02 for 2 percent.
    double depth_tolerance_ratio = 0.02;
    /// Tolerance on the back-projection round trip, in reference pixels.
    double round_trip_px = 2.0;
};

struct ConsistencyReport {
    int checked = 0;
    int kept = 0;
};

/// Clears the depth of every pixel that the neighbour does not confirm. `reference` is modified in
/// place; the neighbour is only read.
ConsistencyReport filterByConsistency(
    DepthMap& reference,
    const DepthMap& neighbour,
    const CameraModel& reference_camera,
    const CameraModel& neighbour_camera,
    const Pose& reference_pose,
    const Pose& neighbour_pose,
    const ConsistencyOptions& options = {});

}  // namespace scand3d::mvs
