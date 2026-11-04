#pragma once

#include <utility>
#include <vector>

#include "geometry/two_view.hpp"
#include "sfm/bundle_adjustment.hpp"
#include "sfm/keyframe_selection.hpp"
#include "sfm/pnp.hpp"
#include "sfm/report.hpp"
#include "sfm/types.hpp"

namespace scand3d::sfm {

/// One captured view: intrinsics plus the pixels of its features, in the caller's feature order.
struct FrameInput {
    CameraModel camera;
    std::vector<Vec2> pixels;
};

/// Feature matches between two views, indexed into their `pixels` arrays.
struct PairMatches {
    int view_a = -1;
    int view_b = -1;
    std::vector<std::pair<int, int>> matches;  // (feature in a, feature in b)
};

struct SfmOptions {
    geometry::RansacOptions two_view{};
    PnpOptions pnp{};
    BundleAdjustmentOptions bundle_adjustment{};
    KeyframePolicy keyframe{};
    /// Observations this far off are dropped after every registration (roadmap 4.5).
    double outlier_reprojection_px = 4.0;
    double min_parallax_rad = 0.017453292519943295;
    /// Minimum fraction of a candidate's correspondences that PnP must keep.
    double min_pnp_inlier_ratio = 0.3;
    /// Matches a view pair needs before it is worth an essential-matrix fit, and the strength
    /// gate on that fit's inlier set.
    int min_pair_matches = 12;
    /// Inliers the pair's essential matrix must keep. Separate from `min_pair_matches`: a dim
    /// frame may offer many candidate matches and still yield a poorly supported geometry, and
    /// that is exactly what has to be rejected.
    int min_pair_inliers = 12;
    /// Views this far apart in capture order are still matched. Sequential capture means the
    /// registration reference can be a few frames back, not only the immediate neighbour.
    int max_view_gap = 2;
    /// Lowe-style ratio gate for neighbour matching, in percent (lower is stricter).
    int match_ratio_percent = 80;
    /// Registered views the tracker matches each live frame against (D057).
    int track_reference_limit = 3;
    int bundle_adjust_every = 1;
};

struct SfmResult {
    Reconstruction reconstruction;
    ReconstructionReport report;
    /// Views in the order they were registered; the first entry is the seed camera.
    std::vector<int> registration_order;
    /// One line per registration attempt of the grown map: what matched and what stopped it.
    std::vector<std::string> growth_trace;
    std::vector<KeyframeDecision> decisions;
};

/// Incremental SfM: seed from the strongest pair, then register views by PnP against the map,
/// triangulate new tracks, run bundle adjustment and drop outliers (roadmap 4.1-4.6).
SfmResult reconstruct(
    const std::vector<FrameInput>& frames,
    const std::vector<PairMatches>& pairs,
    const SfmOptions& options = {});

}  // namespace scand3d::sfm
