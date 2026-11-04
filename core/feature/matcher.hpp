#pragma once

#include <vector>

#include "feature/features.hpp"

namespace scand3d::feature {

struct FeatureMatch {
    int query_index = 0;
    int train_index = 0;
    int distance = 0;  // Hamming
};

struct MatchOptions {
    /// Best match must be clearly better than the second best, in percent (80 = 0.8).
    int ratio_percent = 80;

    /// Also require the reverse match to agree. Costs a second pass over the candidates.
    bool cross_check = false;

    /// Descriptor bits charged for a 90 degree disagreement between two edge keypoints' gradient
    /// directions (D054). A point on a contour is ambiguous along it; this breaks exactly those
    /// ties without rejecting a match over a camera rotation. Zero disables the term.
    int edge_direction_weight = 8;
};

/// Exhaustive Hamming search with Lowe's ratio test (roadmap 3.3). Used as the reference the
/// approximate matcher is measured against.
std::vector<FeatureMatch> matchBruteForce(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options = {});

/// Corners first, contours only as a fallback (D055): the corner matches are returned when there
/// are at least `min_matches` of them, and only otherwise is the search widened to the contour
/// points, which are ambiguous along their own edge. Match indices always refer to the input sets.
std::vector<FeatureMatch> matchCornersThenContours(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options,
    std::size_t min_matches);

/// Approximate nearest neighbour through multi-index hashing: the 256-bit descriptor is split
/// into eight 32-bit blocks, each indexed separately, and only descriptors sharing a block with
/// the query are compared. Queries that land in an over-full or nearly empty bucket fall back to
/// the exhaustive search, so the result never silently loses matches (roadmap 3.3).
std::vector<FeatureMatch> matchApproximate(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options = {});

}  // namespace scand3d::feature
