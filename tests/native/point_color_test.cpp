#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "overlay/point_color.hpp"

namespace {

using scand3d::overlay::OverlayCamera;
using scand3d::overlay::OverlayPoint;
using scand3d::overlay::RgbImage;
using scand3d::overlay::samplePointColors;

// 4x4 image, focal 4 px at 2 px/unit: a point one unit off axis moves two pixels.
OverlayCamera cameraFor(int width, int height) {
    OverlayCamera camera;
    camera.focal_px = 4.0;
    camera.cx_px = width / 2.0;
    camera.cy_px = height / 2.0;
    camera.image_width_px = width;
    camera.image_height_px = height;
    return camera;
}

RgbImage solidImage(int width, int height, unsigned char r, unsigned char g, unsigned char b) {
    RgbImage image;
    image.width_px = width;
    image.height_px = height;
    image.rgb.resize(static_cast<std::size_t>(width) * height * 3);
    for (std::size_t i = 0; i < image.rgb.size(); i += 3) {
        image.rgb[i] = r;
        image.rgb[i + 1] = g;
        image.rgb[i + 2] = b;
    }
    return image;
}

void setPixel(RgbImage& image, int x, int y, unsigned char r, unsigned char g, unsigned char b) {
    const std::size_t offset =
        (static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width_px) +
         static_cast<std::size_t>(x)) *
        3u;
    image.rgb[offset] = r;
    image.rgb[offset + 1] = g;
    image.rgb[offset + 2] = b;
}

OverlayPoint pointAt(double x, double y, double z) {
    OverlayPoint point;
    point.position_m = {x, y, z};
    point.rgb[0] = 0.0f;
    point.rgb[1] = 0.0f;
    point.rgb[2] = 0.0f;
    return point;
}

TEST(PointColor, SamplesWhereThePointProjects) {
    // Left half red, right half blue on an 8x8 keyframe. A point half a metre right of the axis
    // with focal 4 px projects to x = 6, so it must pick up the blue side.
    RgbImage image = solidImage(8, 8, 255, 0, 0);
    for (int y = 0; y < 8; ++y) {
        for (int x = 4; x < 8; ++x) {
            setPixel(image, x, y, 0, 0, 255);
        }
    }

    std::vector<OverlayPoint> points{pointAt(0.5, 0.0, 1.0)};

    EXPECT_EQ(samplePointColors(image, cameraFor(8, 8), points), 1);
    EXPECT_NEAR(points[0].rgb[0], 0.0f, 0.02f);
    EXPECT_NEAR(points[0].rgb[1], 0.0f, 0.02f);
    EXPECT_NEAR(points[0].rgb[2], 1.0f, 0.02f);
}

TEST(PointColor, MedianIgnoresASingleOutlierPixel) {
    // Eight of the nine pixels in the patch are grey; one is bright white.
    RgbImage image = solidImage(6, 6, 40, 40, 40);
    setPixel(image, 2, 2, 255, 255, 255);

    std::vector<OverlayPoint> points{pointAt(0.0, 0.0, 1.0)};

    ASSERT_EQ(samplePointColors(image, cameraFor(6, 6), points), 1);
    EXPECT_NEAR(points[0].rgb[0], 40.0f / 255.0f, 0.01f);
    EXPECT_NEAR(points[0].rgb[1], 40.0f / 255.0f, 0.01f);
}

TEST(PointColor, PointsOutsideTheKeyframeKeepTheirColour) {
    RgbImage image = solidImage(4, 4, 255, 0, 0);
    std::vector<OverlayPoint> points{pointAt(50.0, 0.0, 1.0)};

    EXPECT_EQ(samplePointColors(image, cameraFor(4, 4), points), 0);
    EXPECT_FLOAT_EQ(points[0].rgb[0], 0.0f);
}

TEST(PointColor, PointsBehindTheCameraAreNotSampled) {
    RgbImage image = solidImage(4, 4, 255, 0, 0);
    std::vector<OverlayPoint> points{pointAt(0.0, 0.0, -1.0)};

    EXPECT_EQ(samplePointColors(image, cameraFor(4, 4), points), 0);
}

TEST(PointColor, InvalidKeyframeIsIgnored) {
    RgbImage empty;
    std::vector<OverlayPoint> points{pointAt(0.0, 0.0, 1.0)};

    EXPECT_EQ(samplePointColors(empty, cameraFor(4, 4), points), 0);

    RgbImage short_buffer = solidImage(4, 4, 1, 2, 3);
    short_buffer.rgb.resize(5);
    EXPECT_FALSE(short_buffer.valid());
    EXPECT_EQ(samplePointColors(short_buffer, cameraFor(4, 4), points), 0);
}

}  // namespace
