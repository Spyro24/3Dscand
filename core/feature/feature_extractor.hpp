#pragma once

#include "feature/edge_map.hpp"
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

    /// Points sampled along the edge map, per level (D053). Off by default: measured on the real
    /// scans, edge points cost match precision and map points instead of adding support (see the
    /// numbers in docs/DECISIONS.md D053), so a caller has to ask for them. A corner detector
    /// needs two gradient directions and a table edge or an object outline gives it none, which is
    /// the case these samples exist for.
    int max_edge_points_per_level = 0;

    /// Minimum distance between two edge samples, in level-0 pixels.
    double edge_spacing_px = 8.0;

    /// Sobel magnitude (0..255) an edge sample needs; below this the contour is noise.
    int edge_min_magnitude = 48;

    /// A pyramid level with fewer corners than this is treated as corner-poor and gets edge
    /// samples. On a textured frame the level has hundreds and its features stay unchanged.
    int edge_corner_floor = 120;

    /// How many of the finest pyramid levels may contribute edge samples. One keeps them at the
    /// resolution where their positions are exact.
    int edge_levels = 1;

    EdgeParams edge_params{};
};

/// Scale-space ORB-style extraction (roadmap 3.1 and 3.2): FAST-9 per pyramid level, one
/// keypoint per physical corner selected by response, intensity-centroid orientation, and a
/// steered BRIEF-256 descriptor. Deterministic for a given image and parameter set.
FeatureSet extractFeatures(const GrayImage& image, const FeatureParams& params = {});

}  // namespace scand3d::feature
