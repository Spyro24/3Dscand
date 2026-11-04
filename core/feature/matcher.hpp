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
};

/// Exhaustive Hamming search with Lowe's ratio test (roadmap 3.3). Used as the reference the
/// approximate matcher is measured against.
std::vector<FeatureMatch> matchBruteForce(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options = {});

/// Approximate nearest neighbour through multi-index hashing: the 256-bit descriptor is split
/// into eight 32-bit blocks, each indexed separately, and only descriptors sharing a block with
/// the query are compared. Queries that land in an over-full or nearly empty bucket fall back to
/// the exhaustive search, so the result never silently loses matches (roadmap 3.3).
std::vector<FeatureMatch> matchApproximate(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options = {});

}  // namespace scand3d::feature
