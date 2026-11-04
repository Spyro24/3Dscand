#include "sfm/tracker.hpp"

#include <cstddef>
#include <vector>

#include "feature/matcher.hpp"

namespace scand3d::sfm {
namespace {

// Same floor Session::trackPose uses: below this the pose is a fit to noise.
constexpr int kMinPnpCorrespondences = 6;

// The reference frame's matches are the only ones a snapshot keeps, so the ratio can be tighter
// than the pairwise matching the map growth uses.
constexpr int kTrackingRatioPercent = 85;

}  // namespace

bool Tracker::track(
    int width_px,
    int height_px,
    const unsigned char* gray,
    double focal_px,
    double cx_px,
    double cy_px,
    Pose& out_pose,
    int& out_inliers,
    int view_hint) const {
    out_inliers = 0;
    if (features_.empty() || gray == nullptr || width_px <= 0 || height_px <= 0 ||
        !(focal_px > 0.0)) {
        return false;
    }

    feature::GrayImage image;
    image.width_px = width_px;
    image.height_px = height_px;
    image.pixels.assign(gray,
                        gray + static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px));
    const feature::FeatureSet live = feature::extractFeatures(image, feature_params_);
    if (live.empty()) {
        return false;
    }

    int reference = view_hint;
    if (reference < 0 || static_cast<std::size_t>(reference) >= features_.size()) {
        reference = -1;
        for (std::size_t index = features_.size(); index > 0; --index) {
            if (registered_[index - 1]) {
                reference = static_cast<int>(index - 1);
                break;
            }
        }
    }
    if (reference < 0) {
        return false;
    }

    feature::MatchOptions match_options;
    match_options.ratio_percent = kTrackingRatioPercent;
    match_options.cross_check = true;
    const std::vector<feature::FeatureMatch> matches = feature::matchApproximate(
        live, features_[static_cast<std::size_t>(reference)], match_options);

    std::vector<Vec3> object_points;
    std::vector<Vec2> image_points;
    object_points.reserve(matches.size());
    image_points.reserve(matches.size());
    for (const feature::FeatureMatch& match : matches) {
        const auto found = point_of_feature_.find(featureKey(reference, match.train_index));
        if (found == point_of_feature_.end()) {
            continue;
        }
        const std::size_t point_index = static_cast<std::size_t>(found->second);
        if (point_index >= point_positions_.size()) {
            continue;
        }
        object_points.push_back(point_positions_[point_index]);
        image_points.push_back(
            {live.keypoints[static_cast<std::size_t>(match.query_index)].x_px,
             live.keypoints[static_cast<std::size_t>(match.query_index)].y_px});
    }

    if (object_points.size() < static_cast<std::size_t>(kMinPnpCorrespondences)) {
        return false;
    }

    const PnpResult pnp = estimatePoseRansac(
        object_points, image_points, CameraModel{focal_px, cx_px, cy_px}, pnp_);
    if (!pnp.valid) {
        return false;
    }
    out_pose = pnp.pose;
    out_inliers = pnp.inliers;
    return true;
}

}  // namespace scand3d::sfm
