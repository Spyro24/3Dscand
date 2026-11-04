#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "feature/features.hpp"
#include "feature/gray_image.hpp"
#include "feature/matcher.hpp"

namespace scand3d::tools {

struct OverlayImage {
    int width_px = 0;
    int height_px = 0;
    std::vector<std::uint8_t> rgb;  // row-major RGB, 3 bytes per pixel
};

/// Draws the two grayscale images side by side with one coloured line per match (roadmap 3.5).
/// Purely a debugging aid: the colours carry no meaning beyond distinguishing neighbouring lines.
OverlayImage renderMatchOverlay(
    const scand3d::feature::GrayImage& first,
    const scand3d::feature::GrayImage& second,
    const scand3d::feature::FeatureSet& first_features,
    const scand3d::feature::FeatureSet& second_features,
    const std::vector<scand3d::feature::FeatureMatch>& matches);

bool writePpm(const std::string& path, const OverlayImage& image);

}  // namespace scand3d::tools
