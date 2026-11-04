#include "tools/match_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace scand3d::tools {
namespace {

void putPixel(OverlayImage& image, int x_px, int y_px, std::uint8_t r, std::uint8_t g,
              std::uint8_t b) {
    if (x_px < 0 || y_px < 0 || x_px >= image.width_px || y_px >= image.height_px) {
        return;
    }
    const auto index = (static_cast<std::size_t>(y_px) * static_cast<std::size_t>(image.width_px) +
                        static_cast<std::size_t>(x_px)) * 3;
    image.rgb[index] = r;
    image.rgb[index + 1] = g;
    image.rgb[index + 2] = b;
}

void drawLine(OverlayImage& image, int x0, int y0, int x1, int y1, std::uint8_t r,
              std::uint8_t g, std::uint8_t b) {
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    const int step_x = x0 < x1 ? 1 : -1;
    const int step_y = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (true) {
        putPixel(image, x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int doubled = 2 * error;
        if (doubled >= dy) {
            error += dy;
            x0 += step_x;
        }
        if (doubled <= dx) {
            error += dx;
            y0 += step_y;
        }
    }
}

}  // namespace

OverlayImage renderMatchOverlay(
    const scand3d::feature::GrayImage& first,
    const scand3d::feature::GrayImage& second,
    const scand3d::feature::FeatureSet& first_features,
    const scand3d::feature::FeatureSet& second_features,
    const std::vector<scand3d::feature::FeatureMatch>& matches) {
    OverlayImage overlay;
    overlay.width_px = first.width_px + second.width_px;
    overlay.height_px = std::max(first.height_px, second.height_px);
    overlay.rgb.assign(
        static_cast<std::size_t>(overlay.width_px) * static_cast<std::size_t>(overlay.height_px) * 3,
        0);

    for (int y = 0; y < first.height_px; ++y) {
        for (int x = 0; x < first.width_px; ++x) {
            const std::uint8_t value = first.at(x, y);
            putPixel(overlay, x, y, value, value, value);
        }
    }
    for (int y = 0; y < second.height_px; ++y) {
        for (int x = 0; x < second.width_px; ++x) {
            const std::uint8_t value = second.at(x, y);
            putPixel(overlay, first.width_px + x, y, value, value, value);
        }
    }

    for (std::size_t i = 0; i < matches.size(); ++i) {
        const auto& match = matches[i];
        if (match.query_index < 0 ||
            static_cast<std::size_t>(match.query_index) >= first_features.keypoints.size() ||
            match.train_index < 0 ||
            static_cast<std::size_t>(match.train_index) >= second_features.keypoints.size()) {
            continue;
        }
        const auto& a = first_features.keypoints[static_cast<std::size_t>(match.query_index)];
        const auto& b = second_features.keypoints[static_cast<std::size_t>(match.train_index)];

        // Deterministic, well-spread colours: neighbouring lines must not look identical.
        const auto hue = static_cast<std::uint8_t>((i * 67u) % 256u);
        const auto r = static_cast<std::uint8_t>((hue * 2u) % 200u + 55u);
        const auto g = static_cast<std::uint8_t>((hue * 3u) % 200u + 55u);
        const auto ble = static_cast<std::uint8_t>((255u - hue) % 200u + 55u);

        const int ax = static_cast<int>(std::lround(a.x_px));
        const int ay = static_cast<int>(std::lround(a.y_px));
        const int bx = first.width_px + static_cast<int>(std::lround(b.x_px));
        const int by = static_cast<int>(std::lround(b.y_px));
        drawLine(overlay, ax, ay, bx, by, r, g, ble);
    }
    return overlay;
}

bool writePpm(const std::string& path, const OverlayImage& image) {
    if (image.width_px <= 0 || image.height_px <= 0 || image.rgb.empty()) {
        return false;
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    out << "P6\n" << image.width_px << " " << image.height_px << "\n255\n";
    out.write(reinterpret_cast<const char*>(image.rgb.data()),
              static_cast<std::streamsize>(image.rgb.size()));
    return out.good();
}

}  // namespace scand3d::tools
