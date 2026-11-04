#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "feature/feature_extractor.hpp"
#include "feature/features.hpp"
#include "sfm/pnp.hpp"
#include "sfm/types.hpp"

namespace scand3d::sfm {

class Session;

/// Immutable snapshot of a reconstructed map, taken so live frames can be tracked while the session
/// that produced it runs the next map. `Session::run` takes seconds and holds the session lock, so
/// tracking through the session would stall for the whole run and report poses for a camera that has
/// already moved; the snapshot owns everything PnP needs and needs no lock (D051).
class Tracker {
public:
    /// Extracts features from a live frame, matches them against the map's reference view and
    /// estimates the pose by PnP. False when too few correspondences survive; `out_inliers` is 0
    /// then, so a rejected frame never carries a stale pose.
    bool track(
        int width_px,
        int height_px,
        const unsigned char* gray,
        double focal_px,
        double cx_px,
        double cy_px,
        Pose& out_pose,
        int& out_inliers,
        int view_hint = -1) const;

    int viewCount() const { return static_cast<int>(features_.size()); }
    bool empty() const { return features_.empty(); }
    std::size_t pointCount() const { return point_positions_.size(); }

    /// Feature key used by the map: the view index in the high half, the feature index in the low.
    static std::uint64_t featureKey(int view, int feature) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(view)) << 32) |
               static_cast<std::uint32_t>(feature);
    }

private:
    friend class Session;

    Tracker() = default;

    std::vector<feature::FeatureSet> features_;
    /// Parallel to the map's points, including the invalid ones, because the feature index stores
    /// positions in the point array rather than in a compacted list.
    std::vector<Vec3> point_positions_;
    std::unordered_map<std::uint64_t, int> point_of_feature_;
    std::vector<bool> registered_;
    /// Registered views a live frame is matched against, nearest to the anchor first (D057).
    /// 1 keeps the single-reference behaviour.
    int reference_limit_ = 3;
    feature::FeatureParams feature_params_{};
    PnpOptions pnp_{};
};

}  // namespace scand3d::sfm
