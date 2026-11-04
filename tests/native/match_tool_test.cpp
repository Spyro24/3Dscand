#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "feature/features.hpp"
#include "feature/gray_image.hpp"
#include "tools/match_overlay.hpp"
#include "tools/pgm.hpp"

namespace {

using scand3d::feature::Descriptor;
using scand3d::feature::FeatureMatch;
using scand3d::feature::FeatureSet;
using scand3d::feature::GrayImage;
using scand3d::feature::Keypoint;

GrayImage makeImage(int width_px, int height_px, std::uint8_t value) {
    return {width_px, height_px,
            std::vector<std::uint8_t>(static_cast<std::size_t>(width_px) *
                                          static_cast<std::size_t>(height_px),
                                      value)};
}

}  // namespace

TEST(PgmIo, RoundTripsAGrayscaleImage) {
    const auto path = (std::filesystem::temp_directory_path() / "scand3d-pgm-roundtrip.pgm").string();
    const std::vector<std::uint8_t> pixels{0, 10, 20, 30, 40, 50};

    ASSERT_TRUE(scand3d::tools::writePgm(path, 3, 2, pixels));

    int width_px = 0;
    int height_px = 0;
    std::vector<std::uint8_t> loaded;
    ASSERT_TRUE(scand3d::tools::readPgm(path, width_px, height_px, loaded));
    EXPECT_EQ(3, width_px);
    EXPECT_EQ(2, height_px);
    EXPECT_EQ(pixels, loaded);

    std::filesystem::remove(path);
    EXPECT_FALSE(scand3d::tools::readPgm(path, width_px, height_px, loaded));
}

TEST(MatchOverlay, RendersBothImagesSideBySide) {
    const GrayImage first = makeImage(6, 4, 40);
    const GrayImage second = makeImage(6, 4, 120);

    FeatureSet first_features;
    first_features.keypoints = {Keypoint{1.0, 1.0, 1.0, 0.0, 1.0}};
    first_features.descriptors = {Descriptor{}};
    FeatureSet second_features;
    second_features.keypoints = {Keypoint{2.0, 2.0, 1.0, 0.0, 1.0}};
    second_features.descriptors = {Descriptor{}};

    const auto overlay = scand3d::tools::renderMatchOverlay(
        first, second, first_features, second_features,
        std::vector<FeatureMatch>{{0, 0, 10}});

    EXPECT_EQ(12, overlay.width_px);
    EXPECT_EQ(4, overlay.height_px);
    ASSERT_EQ(static_cast<std::size_t>(12 * 4 * 3), overlay.rgb.size());

    // The left half keeps the dark image, the right half the bright one.
    EXPECT_EQ(40, overlay.rgb[0]);
    const std::size_t right_index = static_cast<std::size_t>(6 * 3);
    EXPECT_EQ(120, overlay.rgb[right_index]);

    // The match line is coloured, so at least one pixel has unequal channels.
    bool coloured = false;
    for (std::size_t i = 0; i + 2 < overlay.rgb.size(); i += 3) {
        if (overlay.rgb[i] != overlay.rgb[i + 1] || overlay.rgb[i + 1] != overlay.rgb[i + 2]) {
            coloured = true;
            break;
        }
    }
    EXPECT_TRUE(coloured);
}

TEST(MatchOverlay, WritesAPpm) {
    const auto path = (std::filesystem::temp_directory_path() / "scand3d-overlay.ppm").string();
    const GrayImage image = makeImage(2, 2, 90);
    FeatureSet features;
    features.keypoints = {Keypoint{0.0, 0.0, 1.0, 0.0, 1.0}};
    features.descriptors = {Descriptor{}};

    const auto overlay = scand3d::tools::renderMatchOverlay(
        image, image, features, features, std::vector<FeatureMatch>{{0, 0, 4}});
    ASSERT_TRUE(scand3d::tools::writePpm(path, overlay));

    std::FILE* file = std::fopen(path.c_str(), "rb");
    ASSERT_NE(nullptr, file);
    char header[16] = {};
    ASSERT_GT(std::fread(header, 1, 15, file), 0u);
    std::fclose(file);
    EXPECT_EQ(std::string("P6"), std::string(header, 2));

    std::filesystem::remove(path);
}
