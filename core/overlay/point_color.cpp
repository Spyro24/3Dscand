#include "overlay/point_color.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace scand3d::overlay {
namespace {

// Patch half-size in pixels: 3x3 keeps the sample local without needing a mip pyramid.
constexpr int kPatchRadiusPx = 1;

unsigned char medianOf(std::array<unsigned char, 9>& values, int count) {
    std::sort(values.begin(), values.begin() + count);
    return values[static_cast<std::size_t>(count / 2)];
}

}  // namespace

int samplePointColors(
    const RgbImage& keyframe,
    const OverlayCamera& camera,
    std::vector<OverlayPoint>& points) {
    if (!keyframe.valid()) {
        return 0;
    }

    int sampled = 0;
    for (OverlayPoint& point : points) {
        double pixel_x = 0.0;
        double pixel_y = 0.0;
        double depth_m = 0.0;
        if (!projectToPixel(camera, point.position_m, pixel_x, pixel_y, depth_m)) {
            continue;
        }
        const int centre_x = static_cast<int>(std::lround(pixel_x));
        const int centre_y = static_cast<int>(std::lround(pixel_y));
        if (centre_x < 0 || centre_y < 0 || centre_x >= keyframe.width_px ||
            centre_y >= keyframe.height_px) {
            continue;
        }

        std::array<std::array<unsigned char, 9>, 3> channels{};
        int counts[3] = {0, 0, 0};
        for (int dy = -kPatchRadiusPx; dy <= kPatchRadiusPx; ++dy) {
            for (int dx = -kPatchRadiusPx; dx <= kPatchRadiusPx; ++dx) {
                const int x = centre_x + dx;
                const int y = centre_y + dy;
                if (x < 0 || y < 0 || x >= keyframe.width_px || y >= keyframe.height_px) {
                    continue;
                }
                const std::size_t offset =
                    (static_cast<std::size_t>(y) * static_cast<std::size_t>(keyframe.width_px) +
                     static_cast<std::size_t>(x)) *
                    3u;
                for (int channel = 0; channel < 3; ++channel) {
                    channels[static_cast<std::size_t>(channel)]
                            [static_cast<std::size_t>(counts[channel])] =
                        keyframe.rgb[offset + static_cast<std::size_t>(channel)];
                    ++counts[channel];
                }
            }
        }

        for (int channel = 0; channel < 3; ++channel) {
            if (counts[channel] > 0) {
                point.rgb[channel] =
                    static_cast<float>(medianOf(channels[static_cast<std::size_t>(channel)],
                                                counts[channel])) /
                    255.0f;
            }
        }
        ++sampled;
    }
    return sampled;
}

}  // namespace scand3d::overlay
