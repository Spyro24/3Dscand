#include "feature/gray_image.hpp"

#include <algorithm>
#include <cmath>

namespace scand3d::feature {

std::uint8_t GrayImage::at(int x_px, int y_px) const {
    if (x_px < 0 || y_px < 0 || x_px >= width_px || y_px >= height_px) {
        return 0;
    }
    return pixels[static_cast<std::size_t>(y_px) * static_cast<std::size_t>(width_px) +
                  static_cast<std::size_t>(x_px)];
}

GrayImage gaussianBlur(const GrayImage& image, double sigma_px) {
    if (image.empty() || !(sigma_px > 0.0)) {
        return image;
    }

    const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma_px)));
    std::vector<double> kernel(static_cast<std::size_t>(2 * radius + 1));
    double sum = 0.0;
    for (int i = 0; i < static_cast<int>(kernel.size()); ++i) {
        const double x = static_cast<double>(i - radius);
        kernel[static_cast<std::size_t>(i)] = std::exp(-0.5 * x * x / (sigma_px * sigma_px));
        sum += kernel[static_cast<std::size_t>(i)];
    }
    for (double& value : kernel) {
        value /= sum;
    }

    const int width = image.width_px;
    const int height = image.height_px;
    GrayImage horizontal = image;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double accumulator = 0.0;
            for (int k = -radius; k <= radius; ++k) {
                const int sample_x = std::clamp(x + k, 0, width - 1);
                accumulator += kernel[static_cast<std::size_t>(k + radius)] *
                               image.at(sample_x, y);
            }
            horizontal.pixels[static_cast<std::size_t>(y) * width + x] =
                static_cast<std::uint8_t>(std::clamp(accumulator, 0.0, 255.0));
        }
    }

    GrayImage blurred = horizontal;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double accumulator = 0.0;
            for (int k = -radius; k <= radius; ++k) {
                const int sample_y = std::clamp(y + k, 0, height - 1);
                accumulator += kernel[static_cast<std::size_t>(k + radius)] *
                               horizontal.at(x, sample_y);
            }
            blurred.pixels[static_cast<std::size_t>(y) * width + x] =
                static_cast<std::uint8_t>(std::clamp(accumulator, 0.0, 255.0));
        }
    }
    return blurred;
}

GrayImage downsampleHalf(const GrayImage& image) {
    GrayImage halved;
    halved.width_px = image.width_px / 2;
    halved.height_px = image.height_px / 2;
    if (halved.width_px <= 0 || halved.height_px <= 0) {
        return halved;
    }
    halved.pixels.assign(
        static_cast<std::size_t>(halved.width_px) * static_cast<std::size_t>(halved.height_px), 0);

    for (int y = 0; y < halved.height_px; ++y) {
        for (int x = 0; x < halved.width_px; ++x) {
            const int sum = static_cast<int>(image.at(2 * x, 2 * y)) +
                            static_cast<int>(image.at(2 * x + 1, 2 * y)) +
                            static_cast<int>(image.at(2 * x, 2 * y + 1)) +
                            static_cast<int>(image.at(2 * x + 1, 2 * y + 1));
            halved.pixels[static_cast<std::size_t>(y) * halved.width_px + x] =
                static_cast<std::uint8_t>((sum + 2) / 4);
        }
    }
    return halved;
}

std::vector<PyramidLevel> buildPyramid(const GrayImage& base, int levels) {
    std::vector<PyramidLevel> pyramid;
    if (base.empty() || levels <= 0) {
        return pyramid;
    }

    // Level 0 is blurred as well: ORB-style pipelines smooth every level before detecting, so a
    // corner found at one scale is the same kind of measurement as one found at another.
    GrayImage current = gaussianBlur(base, 1.2);
    pyramid.push_back({current, 1.0});

    for (int level = 1; level < levels; ++level) {
        current = downsampleHalf(current);
        if (current.width_px < 32 || current.height_px < 32) {
            break;
        }
        pyramid.push_back({gaussianBlur(current, 1.0), 1.0 / static_cast<double>(1 << level)});
    }
    return pyramid;
}

}  // namespace scand3d::feature
