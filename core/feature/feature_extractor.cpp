#include "feature/feature_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "feature/brief_descriptor.hpp"
#include "feature/edge_map.hpp"
#include "feature/fast_detector.hpp"

namespace scand3d::feature {
namespace {

struct EdgeSample {
    int x_px = 0;
    int y_px = 0;
    int magnitude = 0;
};

/// Radius around every corner that needs no edge samples: another point there adds nothing the
/// corner does not already give, and a point on a straight edge between two corners is exactly the
/// ambiguous measurement that lowered match precision to 89% on the textured fixture.
std::vector<std::uint8_t> cornerCoverage(
    const std::vector<FastCorner>& corners, int width_px, int height_px, int radius_px) {
    std::vector<std::uint8_t> covered(
        static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px), 0);
    for (const FastCorner& corner : corners) {
        for (int dy = -radius_px; dy <= radius_px; ++dy) {
            for (int dx = -radius_px; dx <= radius_px; ++dx) {
                const int x = corner.x_px + dx;
                const int y = corner.y_px + dy;
                if (x < 0 || y < 0 || x >= width_px || y >= height_px) {
                    continue;
                }
                covered[static_cast<std::size_t>(y) * width_px + x] = 1;
            }
        }
    }
    return covered;
}

/// One sample per grid cell along the thinned contours, keeping the strongest pixel of each cell.
/// The cell grid spreads samples along a long contour without an O(n^2) spacing pass, and it stops
/// one high-contrast corner from filling the whole list.
std::vector<EdgeSample> sampleEdges(
    const EdgeMap& edges,
    const std::vector<std::uint8_t>& covered,
    int spacing_px,
    int min_magnitude,
    int limit) {
    std::vector<EdgeSample> samples;
    if (edges.empty() || limit <= 0) {
        return samples;
    }
    const int spacing = std::max(1, spacing_px);
    const int columns = (edges.width_px + spacing - 1) / spacing;
    const int rows = (edges.height_px + spacing - 1) / spacing;
    std::vector<EdgeSample> best(static_cast<std::size_t>(columns) * static_cast<std::size_t>(rows));
    std::vector<std::uint8_t> filled(best.size(), 0);
    for (int y = 0; y < edges.height_px; ++y) {
        for (int x = 0; x < edges.width_px; ++x) {
            if (!edges.isEdge(x, y)) {
                continue;
            }
            if (covered[static_cast<std::size_t>(y) * edges.width_px + x] != 0) {
                continue;
            }
            const int magnitude = edges.magnitudeAt(x, y);
            if (magnitude < min_magnitude) {
                continue;
            }
            const std::size_t cell =
                static_cast<std::size_t>(y / spacing) * static_cast<std::size_t>(columns) +
                static_cast<std::size_t>(x / spacing);
            if (filled[cell] != 0 && best[cell].magnitude >= magnitude) {
                continue;
            }
            best[cell] = {x, y, magnitude};
            filled[cell] = 1;
        }
    }
    samples.reserve(best.size() / 4 + 1);
    for (std::size_t cell = 0; cell < best.size(); ++cell) {
        if (filled[cell] != 0) {
            samples.push_back(best[cell]);
        }
    }
    std::sort(samples.begin(), samples.end(), [](const EdgeSample& a, const EdgeSample& b) {
        return a.magnitude > b.magnitude;
    });
    if (static_cast<int>(samples.size()) > limit) {
        samples.resize(static_cast<std::size_t>(limit));
    }
    return samples;
}

}  // namespace

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
    for (std::size_t level_index = 0; level_index < pyramid.size(); ++level_index) {
        const PyramidLevel& level = pyramid[level_index];
        // 0.5 on the first downsampled level; 2.0 as a magnification factor.
        const double magnification = 1.0 / level.scale;
        const int orientation_radius = std::max(
            2, static_cast<int>(std::lround(params.orientation_radius_px * level.scale)));

        auto add_candidate = [&](int x_px, int y_px, double response) {
            Candidate candidate;
            candidate.keypoint.scale = magnification;
            // Level pixel centres map back to level-0 pixel centres, not to pixel corners.
            candidate.keypoint.x_px = (static_cast<double>(x_px) + 0.5) * magnification - 0.5;
            candidate.keypoint.y_px = (static_cast<double>(y_px) + 0.5) * magnification - 0.5;
            candidate.keypoint.response = response;
            // The intensity-centroid angle is recorded as metadata, but the descriptor is not
            // steered by it. On this low-texture object the centroid angle moves between views,
            // and steering the pattern by it dropped verified inlier precision from >=95% to 78%
            // (see docs/DECISIONS.md). Rotation-invariant matching is left for a later task.
            candidate.keypoint.angle_rad =
                patchOrientation(level.image, x_px, y_px, orientation_radius);
            candidate.descriptor = describeBrief(level.image, x_px, y_px, 0.0);
            candidates.push_back(candidate);
        };

        const std::vector<FastCorner> corners =
            detectFastCorners(level.image, params.fast_threshold);
        for (const FastCorner& corner : corners) {
            add_candidate(corner.x_px, corner.y_px, corner.response);
        }

        // Corners alone leave a smooth or low-contrast surface without support: the corner
        // detector needs two gradient directions, and a table edge or an object outline has one.
        // The samples are only worth adding where the corner detector actually runs dry; on a
        // textured frame they cost match precision without adding support the corners lack (D053).
        const bool corner_poor =
            static_cast<int>(corners.size()) < params.edge_corner_floor;
        if (params.max_edge_points_per_level > 0 && corner_poor &&
            static_cast<int>(level_index) < params.edge_levels) {
            const EdgeMap edges = detectEdges(level.image, params.edge_params);
            const int spacing_px =
                std::max(2, static_cast<int>(std::lround(params.edge_spacing_px * level.scale)));
            // A corner already covers its neighbourhood, so only a contour further than the edge
            // spacing from every corner is worth a sample of its own.
            const int corner_radius_px = std::max(
                4, static_cast<int>(std::lround(params.edge_spacing_px * level.scale)));
            const std::vector<std::uint8_t> covered = cornerCoverage(
                corners, level.image.width_px, level.image.height_px, corner_radius_px);
            const auto samples = sampleEdges(
                edges, covered, spacing_px, params.edge_min_magnitude,
                params.max_edge_points_per_level);
            for (const EdgeSample& sample : samples) {
                add_candidate(sample.x_px, sample.y_px, static_cast<double>(sample.magnitude));
            }
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
