#pragma once

#include <cstdint>
#include <vector>

#include "feature/gray_image.hpp"

namespace scand3d::feature {

/// Canny-style edge map: Sobel gradient, non-maximum suppression along the gradient direction, then
/// hysteresis thinning. Two stages use it: feature extraction samples points along the contours,
/// where a corner detector finds nothing on a smooth or low-contrast surface, and the dense pass
/// rejects depth on pixels whose gradient is too weak for the photometric cost to mean anything
/// (D053).
struct EdgeMap {
    int width_px = 0;
    int height_px = 0;

    /// Sobel gradient magnitude, divided by four and clamped to 255. A 100-level step edge reads
    /// about 100, so the thresholds below are in the same units as a pixel difference.
    std::vector<std::uint8_t> magnitude;

    /// 1 where the pixel survived non-maximum suppression and both hysteresis thresholds.
    std::vector<std::uint8_t> edge;

    bool empty() const { return width_px <= 0 || height_px <= 0 || magnitude.empty(); }

    /// Bounds-checked gradient magnitude; anything outside the image reads as 0.
    int magnitudeAt(int x_px, int y_px) const;

    bool isEdge(int x_px, int y_px) const;
};

struct EdgeParams {
    /// Hysteresis: below `low` a pixel never joins an edge, at or above `high` it starts one, and
    /// between the two it only joins an edge that is already connected to a strong pixel. This is
    /// what keeps noise from forming edges of its own while a weak contour still comes through.
    int low = 20;
    int high = 48;
};

/// Edges of one image. The input should already be smoothed (the pyramid's base is), because an
/// unsmoothed JPEG frame turns every block boundary into an edge.
EdgeMap detectEdges(const GrayImage& image, const EdgeParams& params = {});

/// Sobel gradient magnitude at one pixel, on the same 0..255 scale as `EdgeMap::magnitude`.
int gradientMagnitudeAt(const GrayImage& image, int x_px, int y_px);

}  // namespace scand3d::feature
