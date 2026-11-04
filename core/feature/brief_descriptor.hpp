#pragma once

#include <array>
#include <cstdint>

#include "feature/features.hpp"
#include "feature/gray_image.hpp"

namespace scand3d::feature {

/// 256 fixed comparison pairs inside a 31x31 patch. The pattern is generated once from a fixed
/// seed, so descriptors from two builds are comparable and the format never changes silently.
struct BriefPattern {
    std::array<std::array<int, 4>, 256> pairs{};  // x1, y1, x2, y2 offsets

    static const BriefPattern& standard();
};

/// Intensity-centroid orientation over a circular patch, in radians. Used to steer the
/// descriptor so a rotated patch still produces a similar bit string.
double patchOrientation(const GrayImage& image, double x_px, double y_px, int radius_px = 15);

/// BRIEF-256 at `(x_px, y_px)` on the given level, rotated by `angle_rad`.
Descriptor describeBrief(
    const GrayImage& level_image,
    double x_px,
    double y_px,
    double angle_rad,
    const BriefPattern& pattern = BriefPattern::standard());

}  // namespace scand3d::feature
