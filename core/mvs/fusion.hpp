#pragma once

#include <vector>

#include "mvs/types.hpp"

namespace scand3d::mvs {

/// Fusion of the per-view depth maps into one point cloud (roadmap 6.3).
struct FusionOptions {
    /// Edge length of an accumulation voxel. Positions are averaged inside a voxel, which is what
    /// suppresses the per-view depth noise without a second smoothing pass.
    double voxel_m = 0.005;
    /// Points seen by at least this many views are kept; 2 drops single-view outliers.
    int min_observations = 1;
};

struct FusionResult {
    std::vector<FusedPoint> points;
    int depth_pixels = 0;
    int fused_points = 0;
};

/// Back-projects every valid depth pixel of every view, accumulates the results per voxel and
/// returns one averaged point per voxel that reaches `min_observations`.
FusionResult fuseDepths(const std::vector<DepthView>& views, const FusionOptions& options = {});

}  // namespace scand3d::mvs
