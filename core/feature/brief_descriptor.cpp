#include "feature/brief_descriptor.hpp"

#include <algorithm>
#include <cmath>

namespace scand3d::feature {
namespace {

/// Bilinear sample with clamped edges; the patch rotates, so integer sampling would alias.
double sampleBilinear(const GrayImage& image, double x_px, double y_px) {
    const int x0 = static_cast<int>(std::floor(x_px));
    const int y0 = static_cast<int>(std::floor(y_px));
    const double fx = x_px - static_cast<double>(x0);
    const double fy = y_px - static_cast<double>(y0);

    const double v00 = image.at(x0, y0);
    const double v10 = image.at(x0 + 1, y0);
    const double v01 = image.at(x0, y0 + 1);
    const double v11 = image.at(x0 + 1, y0 + 1);
    const double top = v00 + (v10 - v00) * fx;
    const double bottom = v01 + (v11 - v01) * fx;
    return top + (bottom - top) * fy;
}

BriefPattern makePattern() {
    BriefPattern pattern;
    // Fixed LCG: reproducible across runs and platforms, and independent of the standard library.
    std::uint32_t state = 0x5EED1234u;
    auto next = [&state]() {
        state = state * 1664525u + 1013904223u;
        return state;
    };
    auto offset = [&next]() { return static_cast<int>(next() % 31u) - 15; };

    for (auto& pair : pattern.pairs) {
        pair = {offset(), offset(), offset(), offset()};
        // A pair comparing a pixel with itself carries no information.
        while (pair[0] == pair[2] && pair[1] == pair[3]) {
            pair[2] = offset();
            pair[3] = offset();
        }
    }
    return pattern;
}

}  // namespace

const BriefPattern& BriefPattern::standard() {
    static const BriefPattern pattern = makePattern();
    return pattern;
}

double patchOrientation(const GrayImage& image, double x_px, double y_px, int radius_px) {
    double m01 = 0.0;
    double m10 = 0.0;
    const double radius_squared = static_cast<double>(radius_px * radius_px);

    for (int dy = -radius_px; dy <= radius_px; ++dy) {
        for (int dx = -radius_px; dx <= radius_px; ++dx) {
            if (static_cast<double>(dx * dx + dy * dy) > radius_squared) {
                continue;
            }
            const double intensity = image.at(
                static_cast<int>(std::lround(x_px)) + dx,
                static_cast<int>(std::lround(y_px)) + dy);
            m10 += static_cast<double>(dx) * intensity;
            m01 += static_cast<double>(dy) * intensity;
        }
    }
    return std::atan2(m01, m10);
}

Descriptor describeBrief(
    const GrayImage& level_image,
    double x_px,
    double y_px,
    double angle_rad,
    const BriefPattern& pattern) {
    Descriptor descriptor{};
    const double cos_angle = std::cos(angle_rad);
    const double sin_angle = std::sin(angle_rad);

    for (std::size_t i = 0; i < pattern.pairs.size(); ++i) {
        const auto& pair = pattern.pairs[i];
        // Rotate both offsets by the patch orientation, then sample.
        const double ax = static_cast<double>(pair[0]) * cos_angle -
                          static_cast<double>(pair[1]) * sin_angle;
        const double ay = static_cast<double>(pair[0]) * sin_angle +
                          static_cast<double>(pair[1]) * cos_angle;
        const double bx = static_cast<double>(pair[2]) * cos_angle -
                          static_cast<double>(pair[3]) * sin_angle;
        const double by = static_cast<double>(pair[2]) * sin_angle +
                          static_cast<double>(pair[3]) * cos_angle;

        const double first = sampleBilinear(level_image, x_px + ax, y_px + ay);
        const double second = sampleBilinear(level_image, x_px + bx, y_px + by);
        if (first < second) {
            descriptor[i / 8] |= static_cast<std::uint8_t>(1u << (i % 8));
        }
    }
    return descriptor;
}

}  // namespace scand3d::feature
