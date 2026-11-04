#pragma once

#include <vector>

#include "feature/gray_image.hpp"

namespace scand3d::feature {

struct FastCorner {
    int x_px = 0;
    int y_px = 0;
    double response = 0.0;
};

/// FAST-9 on one image level: 9 contiguous circle pixels brighter or darker than the centre by
/// at least `threshold`, then 3x3 non-maximum suppression. Scale-space is the caller's job; this
/// only reports corners in the image it was given.
std::vector<FastCorner> detectFastCorners(
    const GrayImage& image,
    int threshold = 20,
    int border_px = 4);

}  // namespace scand3d::feature
