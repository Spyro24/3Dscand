#include "feature/feature_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "feature/brief_descriptor.hpp"
#include "feature/fast_detector.hpp"

namespace scand3d::feature {

FeatureSet extractFeatures(const GrayImage& image, const FeatureParams& params) {
    FeatureSet features;
    if (image.empty()) {
        return features;
    }

    struct Candidate {
        Keypoint keypoint;
        Descriptor descriptor;
    };
    std::vector<Candidate> candidates;

    const auto pyramid = buildPyramid(image, params.pyramid_levels);
    for (const PyramidLevel& level : pyramid) {
        const auto corners = detectFastCorners(level.image, params.fast_threshold);
        if (corners.empty()) {
            continue;
        }

        // 0.5 on the first downsampled level; 2.0 as a magnification factor.
        const double magnification = 1.0 / level.scale;
        const int orientation_radius = std::max(
            2, static_cast<int>(std::lround(params.orientation_radius_px * level.scale)));

        candidates.reserve(candidates.size() + corners.size());
        for (const FastCorner& corner : corners) {
            Candidate candidate;
            candidate.keypoint.scale = magnification;
            // Level pixel centres map back to level-0 pixel centres, not to pixel corners.
            candidate.keypoint.x_px = (static_cast<double>(corner.x_px) + 0.5) * magnification - 0.5;
            candidate.keypoint.y_px = (static_cast<double>(corner.y_px) + 0.5) * magnification - 0.5;
            candidate.keypoint.response = corner.response;
            // The intensity-centroid angle is recorded as metadata, but the descriptor is not
            // steered by it. On this low-texture object the centroid angle moves between views,
            // and steering the pattern by it dropped verified inlier precision from >=95% to 78%
            // (see docs/DECISIONS.md). Rotation-invariant matching is left for a later task.
            candidate.keypoint.angle_rad =
                patchOrientation(level.image, corner.x_px, corner.y_px, orientation_radius);
            candidate.descriptor = describeBrief(level.image, corner.x_px, corner.y_px, 0.0);
            candidates.push_back(candidate);
        }
    }

    // Strongest first, so cross-scale suppression keeps the best-supported measurement.
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.keypoint.response > b.keypoint.response;
    });

    const double dedup_squared = params.dedup_radius_px * params.dedup_radius_px;
    for (const Candidate& candidate : candidates) {
        if (static_cast<int>(features.keypoints.size()) >= params.max_features) {
            break;
        }
        bool duplicate = false;
        for (const Keypoint& accepted : features.keypoints) {
            const double dx = accepted.x_px - candidate.keypoint.x_px;
            const double dy = accepted.y_px - candidate.keypoint.y_px;
            if (dx * dx + dy * dy < dedup_squared) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }
        features.keypoints.push_back(candidate.keypoint);
        features.descriptors.push_back(candidate.descriptor);
    }
    return features;
}

}  // namespace scand3d::feature
