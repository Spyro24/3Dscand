#include "feature/fast_detector.hpp"

#include <algorithm>
#include <cmath>

namespace scand3d::feature {
namespace {

// Circle of radius 3, in the order FAST expects (clockwise from the top).
constexpr int kCircleOffsets[16][2] = {
    {0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3},
    {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3},
};

/// True when nine consecutive circle pixels are all brighter or all darker than the centre.
bool isFastCorner(const GrayImage& image, int x_px, int y_px, int threshold, double* response) {
    const int centre = image.at(x_px, y_px);
    const int brighter_limit = centre + threshold;
    const int darker_limit = centre - threshold;

    bool brighter[16];
    bool darker[16];
    double difference_sum = 0.0;
    for (int i = 0; i < 16; ++i) {
        const int value = image.at(x_px + kCircleOffsets[i][0], y_px + kCircleOffsets[i][1]);
        brighter[i] = value > brighter_limit;
        darker[i] = value < darker_limit;
        difference_sum += std::abs(value - centre);
    }

    auto has_run = [](const bool states[16]) {
        // The circle wraps, so the scan runs over two periods.
        int run = 0;
        for (int i = 0; i < 32; ++i) {
            if (states[i % 16]) {
                ++run;
                if (run >= 9) {
                    return true;
                }
            } else {
                run = 0;
            }
        }
        return false;
    };

    if (!has_run(brighter) && !has_run(darker)) {
        return false;
    }
    if (response != nullptr) {
        *response = difference_sum;
    }
    return true;
}

}  // namespace

std::vector<FastCorner> detectFastCorners(const GrayImage& image, int threshold, int border_px) {
    std::vector<FastCorner> corners;
    if (image.empty()) {
        return corners;
    }

    const int border = std::max(border_px, 4);
    std::vector<FastCorner> candidates;
    for (int y = border; y < image.height_px - border; ++y) {
        for (int x = border; x < image.width_px - border; ++x) {
            double response = 0.0;
            if (isFastCorner(image, x, y, threshold, &response)) {
                candidates.push_back({x, y, response});
            }
        }
    }

    // Non-maximum suppression on corner strength keeps one response per physical corner.
    for (const FastCorner& candidate : candidates) {
        bool is_maximum = true;
        for (int dy = -1; dy <= 1 && is_maximum; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = candidate.x_px + dx;
                const int ny = candidate.y_px + dy;
                if (nx < border || ny < border || nx >= image.width_px - border ||
                    ny >= image.height_px - border) {
                    continue;
                }
                double neighbour_response = 0.0;
                if (isFastCorner(image, nx, ny, threshold, &neighbour_response) &&
                    neighbour_response > candidate.response) {
                    is_maximum = false;
                    break;
                }
            }
        }
        if (is_maximum) {
            corners.push_back(candidate);
        }
    }
    return corners;
}

}  // namespace scand3d::feature
