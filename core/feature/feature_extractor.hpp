#pragma once

#include "feature/features.hpp"
#include "feature/gray_image.hpp"

namespace scand3d::feature {

struct FeatureParams {
    int pyramid_levels = 3;
    int fast_threshold = 20;

    /// Two corners this close (level-0 pixels) are the same physical point found at two scales;
    /// only the stronger one survives.
    double dedup_radius_px = 2.0;

    int max_features = 1200;
    int orientation_radius_px = 15;
};

/// Scale-space ORB-style extraction (roadmap 3.1 and 3.2): FAST-9 per pyramid level, one
/// keypoint per physical corner selected by response, intensity-centroid orientation, and a
/// steered BRIEF-256 descriptor. Deterministic for a given image and parameter set.
FeatureSet extractFeatures(const GrayImage& image, const FeatureParams& params = {});

}  // namespace scand3d::feature
