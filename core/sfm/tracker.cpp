#include "sfm/tracker.hpp"

#include <algorithm>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include "feature/matcher.hpp"

namespace scand3d::sfm {
namespace {

// Same floor Session::trackPose uses: below this the pose is a fit to noise.
constexpr int kMinPnpCorrespondences = 6;

// The reference frame's matches are the only ones a snapshot keeps, so the ratio can be tighter
// than the pairwise matching the map growth uses.
constexpr int kTrackingRatioPercent = 85;

// A frame supported this well needs no second reference view; further ones cost a full match.
constexpr int kConfidentInliers = 40;

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

    // A snapshot can carry more feature sets than it has registration flags (see
    // Session::snapshot); only the flagged prefix can be a reference view.
    const std::size_t view_count = std::min(features_.size(), registered_.size());
    int anchor = view_hint;
    if (anchor < 0 || static_cast<std::size_t>(anchor) >= view_count ||
        !registered_[static_cast<std::size_t>(anchor)]) {
        anchor = -1;
        for (std::size_t index = view_count; index > 0; --index) {
            if (registered_[index - 1]) {
                anchor = static_cast<int>(index - 1);
                break;
            }
        }
    }
    if (anchor < 0) {
        return false;
    }

    // One reference view only covers the part of the map it observed, so a live frame that has
    // moved past it loses its correspondences even though the neighbouring map view still sees
    // the same surface. Pooling matches from the nearest few registered views is what keeps such
    // a frame placed (D057).
    std::vector<int> references;
    const int limit = reference_limit_ > 0 ? reference_limit_ : 1;
    references.push_back(anchor);
    for (int offset = 1; static_cast<int>(references.size()) < limit &&
                         offset < static_cast<int>(view_count);
         ++offset) {
        for (const int candidate : {anchor - offset, anchor + offset}) {
            if (static_cast<int>(references.size()) >= limit) {
                break;
            }
            if (candidate >= 0 && candidate < static_cast<int>(view_count) &&
                registered_[static_cast<std::size_t>(candidate)]) {
                references.push_back(candidate);
            }
        }
    }

    feature::MatchOptions match_options;
    match_options.ratio_percent = kTrackingRatioPercent;
    match_options.cross_check = true;

    // Each reference view gets its own PnP fit and the best-supported one wins. Pooling their
    // correspondences was measured and is not monotone: a reference whose points are badly
    // triangulated drags the shared fit down, and 12 map views tracked 5 frames with one
    // reference against 3 with three pooled. Searching keeps the single-reference result as a
    // floor instead of hoping the extra correspondences help (D057).
    const PnpOptions pnp_options = pnp_;
    const CameraModel camera{focal_px, cx_px, cy_px};
    bool solved = false;
    int best_inliers = 0;
    Pose best_pose;

    for (const int reference : references) {
        // Corners first; the contour points are only tried when the corners alone cannot place
        // the frame (D055).
        const std::vector<feature::FeatureMatch> matches = feature::matchCornersThenContours(
            live, features_[static_cast<std::size_t>(reference)], match_options,
            static_cast<std::size_t>(kMinPnpCorrespondences) * 2);

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
            continue;
        }
        const PnpResult pnp = estimatePoseRansac(object_points, image_points, camera, pnp_options);
        if (!pnp.valid || pnp.inliers <= best_inliers) {
            continue;
        }
        best_inliers = pnp.inliers;
        best_pose = pnp.pose;
        solved = true;
        // A frame this well supported needs no second opinion; the neighbours cost a full match.
        if (best_inliers >= kConfidentInliers) {
            break;
        }
    }

    if (!solved) {
        return false;
    }
    out_pose = best_pose;
    out_inliers = best_inliers;
    return true;
}

}  // namespace scand3d::sfm
