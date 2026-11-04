#include "feature/matcher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace scand3d::feature {
namespace {

constexpr int kBlocks = 8;         // 8 x 32 bits = the 256-bit descriptor
constexpr std::size_t kBlockBytes = 4;
constexpr std::size_t kMinCandidates = 6;
constexpr std::size_t kMaxCandidates = 400;

std::uint32_t readBlock(const Descriptor& descriptor, int block) {
    const auto offset = static_cast<std::size_t>(block) * kBlockBytes;
    return static_cast<std::uint32_t>(descriptor[offset]) |
           (static_cast<std::uint32_t>(descriptor[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(descriptor[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(descriptor[offset + 3]) << 24);
}

constexpr double kHalfPi = 1.5707963267948966;
constexpr double kPi = 3.14159265358979323846;

/// Hamming bits, plus a penalty when both keypoints are edge points whose gradient directions
/// disagree. Two points on the same straight contour have identical surroundings, so the direction
/// is the only thing that says which of them a query belongs to.
int matchDistance(
    const Keypoint& query_keypoint,
    const Descriptor& query,
    const Keypoint& train_keypoint,
    const Descriptor& train,
    int direction_weight) {
    int distance = hammingDistance(query, train);
    if (direction_weight > 0 && query_keypoint.is_edge && train_keypoint.is_edge) {
        double delta =
            std::fabs(query_keypoint.edge_direction_rad - train_keypoint.edge_direction_rad);
        if (delta > kHalfPi) {
            delta = std::fabs(kPi - delta);
        }
        distance += static_cast<int>(std::lround(direction_weight * delta / kHalfPi));
    }
    return distance;
}

/// Best and second-best among the given train indices. Returns false when there is no second
/// distinct match, because the ratio test would be meaningless.
bool bestTwo(
    const Keypoint& query_keypoint,
    const Descriptor& query,
    const FeatureSet& train,
    const std::vector<int>& candidates,
    int direction_weight,
    int& best_index,
    int& best_distance,
    int& second_distance) {
    best_index = -1;
    best_distance = 0;
    // A real best distance can be zero (identical descriptors), so the "no second candidate"
    // state needs its own sentinel instead of zero.
    second_distance = std::numeric_limits<int>::max();

    for (const int index : candidates) {
        const std::size_t train_index = static_cast<std::size_t>(index);
        const int distance = matchDistance(
            query_keypoint, query, train.keypoints[train_index], train.descriptors[train_index],
            direction_weight);
        if (best_index < 0) {
            best_distance = distance;
            best_index = index;
        } else if (distance < best_distance) {
            second_distance = best_distance;
            best_distance = distance;
            best_index = index;
        } else if (distance < second_distance) {
            second_distance = distance;
        }
    }
    return best_index >= 0 && second_distance != std::numeric_limits<int>::max();
}

bool passesRatio(int best_distance, int second_distance, int ratio_percent) {
    return best_distance * 100 < ratio_percent * second_distance;
}

}  // namespace

/// Approximate matching without the corner/contour split, shared by the public entry point and the
/// staged matcher below.
std::vector<FeatureMatch> matchApproximateSet(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options);

std::vector<FeatureMatch> matchBruteForce(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options) {
    std::vector<FeatureMatch> matches;
    if (query.empty() || train.empty()) {
        return matches;
    }

    std::vector<int> all_indices(train.descriptors.size());
    for (std::size_t i = 0; i < all_indices.size(); ++i) {
        all_indices[i] = static_cast<int>(i);
    }

    std::vector<int> reverse_best(query.size(), -1);
    if (options.cross_check) {
        // For each train descriptor, its best query; a match survives only if both agree.
        for (std::size_t j = 0; j < train.descriptors.size(); ++j) {
            int best_query = -1;
            int best_distance = 0;
            for (std::size_t i = 0; i < query.descriptors.size(); ++i) {
                const int distance = matchDistance(
                    query.keypoints[i], query.descriptors[i], train.keypoints[j],
                    train.descriptors[j], options.edge_direction_weight);
                if (best_query < 0 || distance < best_distance) {
                    best_query = static_cast<int>(i);
                    best_distance = distance;
                }
            }
            if (best_query >= 0) {
                reverse_best[static_cast<std::size_t>(best_query)] = static_cast<int>(j);
            }
        }
    }

    matches.reserve(query.size());
    for (std::size_t i = 0; i < query.descriptors.size(); ++i) {
        int best_index = -1;
        int best_distance = 0;
        int second_distance = 0;
        if (!bestTwo(query.keypoints[i], query.descriptors[i], train, all_indices,
                     options.edge_direction_weight, best_index, best_distance,
                     second_distance)) {
            continue;
        }
        if (!passesRatio(best_distance, second_distance, options.ratio_percent)) {
            continue;
        }
        if (options.cross_check && reverse_best[i] != best_index) {
            continue;
        }
        matches.push_back({static_cast<int>(i), best_index, best_distance});
    }
    return matches;
}

std::vector<FeatureMatch> matchApproximate(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options) {
    return matchApproximateSet(query, train, options);
}

std::vector<FeatureMatch> matchApproximateSet(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options) {
    std::vector<FeatureMatch> matches;
    if (query.empty() || train.empty()) {
        return matches;
    }

    std::array<std::unordered_map<std::uint32_t, std::vector<int>>, kBlocks> tables;
    for (std::size_t j = 0; j < train.descriptors.size(); ++j) {
        for (int block = 0; block < kBlocks; ++block) {
            tables[static_cast<std::size_t>(block)][readBlock(train.descriptors[j], block)]
                .push_back(static_cast<int>(j));
        }
    }

    std::vector<std::uint32_t> stamp(train.descriptors.size(), 0);
    std::uint32_t generation = 0;
    std::vector<int> candidates;
    std::vector<int> all_indices(train.descriptors.size());
    for (std::size_t i = 0; i < all_indices.size(); ++i) {
        all_indices[i] = static_cast<int>(i);
    }

    for (std::size_t i = 0; i < query.descriptors.size(); ++i) {
        ++generation;
        candidates.clear();
        for (int block = 0; block < kBlocks; ++block) {
            const auto& bucket =
                tables[static_cast<std::size_t>(block)][readBlock(query.descriptors[i], block)];
            for (const int index : bucket) {
                auto& seen = stamp[static_cast<std::size_t>(index)];
                if (seen != generation) {
                    seen = generation;
                    candidates.push_back(index);
                }
            }
        }

        // A bucket that is nearly empty or far too full means the hashing is not discriminating;
        // exhaustive search is both safer and cheap at these sizes.
        const std::vector<int>* pool = &candidates;
        if (candidates.size() < kMinCandidates || candidates.size() > kMaxCandidates) {
            pool = &all_indices;
        }

        int best_index = -1;
        int best_distance = 0;
        int second_distance = 0;
        if (!bestTwo(query.keypoints[i], query.descriptors[i], train, *pool,
                     options.edge_direction_weight, best_index, best_distance,
                     second_distance)) {
            continue;
        }
        if (!passesRatio(best_distance, second_distance, options.ratio_percent)) {
            continue;
        }
        // Cross-check is deliberately not offered here: it would require a second index over the
        // query set, and callers who need it should use the exhaustive matcher.
        matches.push_back({static_cast<int>(i), best_index, best_distance});
    }
    return matches;
}

std::vector<FeatureMatch> matchCornersThenContours(
    const FeatureSet& query,
    const FeatureSet& train,
    const MatchOptions& options,
    std::size_t min_matches) {
    std::vector<FeatureMatch> matches;
    if (query.empty() || train.empty()) {
        return matches;
    }

    // The corner-only subsets, plus the map back to the caller's indices.
    auto corners = [](const FeatureSet& features, FeatureSet& subset, std::vector<int>& indices) {
        indices.clear();
        for (std::size_t index = 0; index < features.size(); ++index) {
            if (features.keypoints[index].is_edge) {
                continue;
            }
            subset.keypoints.push_back(features.keypoints[index]);
            subset.descriptors.push_back(features.descriptors[index]);
            indices.push_back(static_cast<int>(index));
        }
    };
    FeatureSet query_corners;
    FeatureSet train_corners;
    std::vector<int> query_index;
    std::vector<int> train_index;
    corners(query, query_corners, query_index);
    corners(train, train_corners, train_index);

    matches = matchApproximateSet(query_corners, train_corners, options);
    if (matches.size() >= min_matches) {
        for (FeatureMatch& match : matches) {
            match.query_index = query_index[static_cast<std::size_t>(match.query_index)];
            match.train_index = train_index[static_cast<std::size_t>(match.train_index)];
        }
        return matches;
    }
    return matchApproximateSet(query, train, options);
}

}  // namespace scand3d::feature
