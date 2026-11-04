#include "feature/edge_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace scand3d::feature {
namespace {

/// Sobel response, kept separate because non-maximum suppression also needs the direction.
struct Gradient {
    int x = 0;
    int y = 0;
};

Gradient sobel(const GrayImage& image, int x_px, int y_px) {
    Gradient gradient;
    gradient.x = -image.at(x_px - 1, y_px - 1) + image.at(x_px + 1, y_px - 1) -
                 2 * image.at(x_px - 1, y_px) + 2 * image.at(x_px + 1, y_px) -
                 image.at(x_px - 1, y_px + 1) + image.at(x_px + 1, y_px + 1);
    gradient.y = -image.at(x_px - 1, y_px - 1) - 2 * image.at(x_px, y_px - 1) -
                 image.at(x_px + 1, y_px - 1) + image.at(x_px - 1, y_px + 1) +
                 2 * image.at(x_px, y_px + 1) + image.at(x_px + 1, y_px + 1);
    return gradient;
}

/// Sobel magnitude divided by four: the largest possible response is 4 * 255 per axis, so the
/// quotient stays inside a byte for any image.
std::uint8_t scaledMagnitude(const Gradient& gradient) {
    const double magnitude = std::hypot(static_cast<double>(gradient.x), static_cast<double>(gradient.y)) / 4.0;
    if (magnitude >= 255.0) {
        return 255;
    }
    return static_cast<std::uint8_t>(magnitude + 0.5);
}

/// True when this pixel wins against both neighbours across the gradient direction. The direction
/// is quantised to the four pixel axes; a diagonal maximum is within a pixel of the axis maximum,
/// and thinning only has to break the ridge into one pixel. A step edge has the same magnitude on
/// both of its pixels, so the tie is broken in favour of the earlier one in scan order; comparing
/// ">=" on both sides would keep a two-pixel band and double every edge count.
bool isRidgeMaximum(const GrayImage& image, int x_px, int y_px, const Gradient& gradient) {
    const int own = std::abs(gradient.x) + std::abs(gradient.y);
    const bool horizontal = std::abs(gradient.x) >= std::abs(gradient.y);
    const int dx = horizontal ? 1 : 0;
    const int dy = horizontal ? 0 : 1;
    const Gradient before = sobel(image, x_px - dx, y_px - dy);
    const Gradient after = sobel(image, x_px + dx, y_px + dy);
    return own >= std::abs(before.x) + std::abs(before.y) &&
           own > std::abs(after.x) + std::abs(after.y);
}

}  // namespace

int EdgeMap::magnitudeAt(int x_px, int y_px) const {
    if (x_px < 0 || y_px < 0 || x_px >= width_px || y_px >= height_px || magnitude.empty()) {
        return 0;
    }
    return magnitude[static_cast<std::size_t>(y_px) * width_px + x_px];
}

bool EdgeMap::isEdge(int x_px, int y_px) const {
    if (x_px < 0 || y_px < 0 || x_px >= width_px || y_px >= height_px || edge.empty()) {
        return false;
    }
    return edge[static_cast<std::size_t>(y_px) * width_px + x_px] != 0;
}

int gradientMagnitudeAt(const GrayImage& image, int x_px, int y_px) {
    if (image.empty() || x_px < 1 || y_px < 1 || x_px >= image.width_px - 1 ||
        y_px >= image.height_px - 1) {
        return 0;
    }
    return scaledMagnitude(sobel(image, x_px, y_px));
}

EdgeMap detectEdges(const GrayImage& image, const EdgeParams& params) {
    EdgeMap map;
    if (image.empty()) {
        return map;
    }
    map.width_px = image.width_px;
    map.height_px = image.height_px;
    const std::size_t count =
        static_cast<std::size_t>(image.width_px) * static_cast<std::size_t>(image.height_px);
    map.magnitude.assign(count, 0);
    map.edge.assign(count, 0);

    const int low = std::max(1, params.low);
    const int high = std::max(low, params.high);

    // Pass 1: magnitude everywhere, and the thinned ridge kept in `edge` as "weak" (2).
    for (int y = 0; y < image.height_px; ++y) {
        for (int x = 0; x < image.width_px; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * image.width_px + x;
            if (x < 1 || y < 1 || x >= image.width_px - 1 || y >= image.height_px - 1) {
                continue;
            }
            const Gradient gradient = sobel(image, x, y);
            map.magnitude[index] = scaledMagnitude(gradient);
            if (map.magnitude[index] < low || !isRidgeMaximum(image, x, y, gradient)) {
                continue;
            }
            map.edge[index] = map.magnitude[index] >= high ? 2 : 1;
        }
    }

    // Pass 2: hysteresis. Every strong pixel starts a contour, and a weak pixel joins only when it
    // touches one already in the contour, so a noisy patch on its own never survives.
    std::vector<int> stack;
    stack.reserve(count / 16 + 1);
    for (std::size_t index = 0; index < map.edge.size(); ++index) {
        if (map.edge[index] == 2) {
            stack.push_back(static_cast<int>(index));
        }
    }
    while (!stack.empty()) {
        const int index = stack.back();
        stack.pop_back();
        const int x = index % image.width_px;
        const int y = index / image.width_px;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= image.width_px || ny >= image.height_px) {
                    continue;
                }
                const std::size_t neighbour = static_cast<std::size_t>(ny) * image.width_px + nx;
                if (map.edge[neighbour] != 1) {
                    continue;
                }
                map.edge[neighbour] = 2;
                stack.push_back(static_cast<int>(neighbour));
            }
        }
    }
    for (std::size_t index = 0; index < map.edge.size(); ++index) {
        map.edge[index] = map.edge[index] == 2 ? 1 : 0;
    }
    return map;
}

}  // namespace scand3d::feature
