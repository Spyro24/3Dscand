#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace scand3d::feature {

struct Keypoint {
    double x_px = 0.0;  // level-0 image coordinates
    double y_px = 0.0;
    double scale = 1.0;      // detection scale relative to level 0
    double angle_rad = 0.0;  // intensity-centroid orientation, steers the descriptor
    double response = 0.0;   // corner strength, used for cross-scale suppression

    /// Gradient direction of an edge keypoint, in [0, pi). Two points on the same contour have the
    /// same direction, and that is what tells them apart along it (D054); meaningless unless
    /// `is_edge`.
    double edge_direction_rad = 0.0;

    /// True when the keypoint came from the edge map rather than the corner detector.
    bool is_edge = false;
};

/// One 256-bit BRIEF descriptor.
using Descriptor = std::array<std::uint8_t, 32>;

inline int hammingDistance(const Descriptor& a, const Descriptor& b) {
    int distance = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        auto difference = static_cast<std::uint8_t>(a[i] ^ b[i]);
        while (difference != 0) {
            distance += difference & 1;
            difference = static_cast<std::uint8_t>(difference >> 1);
        }
    }
    return distance;
}

/// Keypoints and descriptors always travel together and in the same order.
struct FeatureSet {
    std::vector<Keypoint> keypoints;
    std::vector<Descriptor> descriptors;

    std::size_t size() const { return keypoints.size(); }
    bool empty() const { return keypoints.empty(); }
};

}  // namespace scand3d::feature
