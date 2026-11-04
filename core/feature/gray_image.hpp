#pragma once

#include <cstdint>
#include <vector>

namespace scand3d::feature {

/// Single-channel 8-bit image. The camera path produces JPEG planes; tests produce synthetic
/// renders, and both meet here.
struct GrayImage {
    int width_px = 0;
    int height_px = 0;
    std::vector<std::uint8_t> pixels;

    bool empty() const { return width_px <= 0 || height_px <= 0 || pixels.empty(); }

    /// Bounds-checked read; anything outside the image is black, matching the renderer.
    std::uint8_t at(int x_px, int y_px) const;
};

/// Separable Gaussian with clamped edges. Feature detection needs a smoothed image or every
/// JPEG block boundary becomes a corner.
GrayImage gaussianBlur(const GrayImage& image, double sigma_px);

/// 2x2 box average. Used to build the pyramid; the average doubles as the anti-alias filter.
GrayImage downsampleHalf(const GrayImage& image);

struct PyramidLevel {
    GrayImage image;

    /// Size relative to level 0, e.g. 0.5 on the first downsampled level.
    double scale = 1.0;
};

/// Gaussian pyramid: level 0 is the blurred input, each later level has half the pixels
/// (agents.md section 7 leaves ORB-style scale-space as the default).
std::vector<PyramidLevel> buildPyramid(const GrayImage& base, int levels);

}  // namespace scand3d::feature
