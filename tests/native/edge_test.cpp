#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "feature/edge_map.hpp"
#include "feature/feature_extractor.hpp"
#include "feature/gray_image.hpp"

namespace {

using scand3d::feature::EdgeMap;
using scand3d::feature::FeatureParams;
using scand3d::feature::FeatureSet;
using scand3d::feature::GrayImage;

/// A flat rectangle on a flat background: the only structure is the contour, so a corner detector
/// finds the four corners and nothing else. This is the case real scans hit on a plain surface.
GrayImage flatRectangle(int width_px = 160, int height_px = 120, int inset_px = 30,
                        std::uint8_t background = 40, std::uint8_t shape = 150) {
    GrayImage image;
    image.width_px = width_px;
    image.height_px = height_px;
    image.pixels.assign(static_cast<std::size_t>(width_px) * height_px, background);
    for (int y = inset_px; y < height_px - inset_px; ++y) {
        for (int x = inset_px; x < width_px - inset_px; ++x) {
            image.pixels[static_cast<std::size_t>(y) * width_px + x] = shape;
        }
    }
    return image;
}

int countEdges(const EdgeMap& edges) {
    int count = 0;
    for (std::uint8_t value : edges.edge) {
        count += value != 0 ? 1 : 0;
    }
    return count;
}

}  // namespace

TEST(EdgeMap, FindsTheContourOfAFlatShape) {
    const GrayImage image = flatRectangle();
    const EdgeMap edges = scand3d::feature::detectEdges(image);

    ASSERT_EQ(image.width_px, edges.width_px);
    ASSERT_EQ(image.height_px, edges.height_px);

    // The contour is 100 x 60 in the middle of the image, so its perimeter is 320 pixels. Thinning
    // keeps one pixel across the edge, and the corners of the rectangle are counted once each, so
    // the count lands close to the perimeter rather than a band around it.
    const int contour = countEdges(edges);
    EXPECT_GT(contour, 250);
    EXPECT_LT(contour, 400);

    // Nothing inside the shape and nothing in the background: both are flat.
    EXPECT_FALSE(edges.isEdge(80, 60));
    EXPECT_FALSE(edges.isEdge(5, 5));
    // But the border is found on all four sides.
    EXPECT_TRUE(edges.isEdge(80, 30) || edges.isEdge(80, 31) || edges.isEdge(80, 29));
    EXPECT_TRUE(edges.isEdge(80, 89) || edges.isEdge(80, 90) || edges.isEdge(80, 88));
    EXPECT_TRUE(edges.isEdge(30, 60) || edges.isEdge(31, 60) || edges.isEdge(29, 60));
    EXPECT_TRUE(edges.isEdge(129, 60) || edges.isEdge(130, 60) || edges.isEdge(128, 60));
}

TEST(EdgeMap, HysteresisKeepsConnectedContoursAndDropsIsolatedNoise) {
    GrayImage image = flatRectangle();
    // One isolated pixel, 22 levels brighter than the background: above the low threshold and
    // below the high one, which is exactly the case hysteresis is there to settle.
    image.pixels[static_cast<std::size_t>(10) * image.width_px + 10] = 62;
    const EdgeMap edges = scand3d::feature::detectEdges(image);

    // The speckle thins to a ring of weak pixels with no strong pixel to connect to, so the
    // hysteresis pass leaves it out while the rectangle's contour, which has strong pixels
    // along its whole length, survives.
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            EXPECT_FALSE(edges.isEdge(10 + dx, 10 + dy))
                << "speckle survived at " << (10 + dx) << "," << (10 + dy);
        }
    }
    EXPECT_GT(countEdges(edges), 250);
}

TEST(EdgePoints, AddSupportWhereCornersAloneFindAlmostNothing) {
    const GrayImage image = flatRectangle();

    FeatureParams corners_only;
    corners_only.max_edge_points_per_level = 0;
    const FeatureSet without_edges = scand3d::feature::extractFeatures(image, corners_only);

    FeatureParams with_edges;
    with_edges.max_edge_points_per_level = 300;
    const FeatureSet with = scand3d::feature::extractFeatures(image, with_edges);

    // Four corners is all a flat rectangle gives FAST; the contour adds samples along all sides.
    EXPECT_LE(without_edges.size(), 8u);
    // The rectangle's contour is straight, so it is sampled on the wider straight-contour grid:
    // its 320-pixel perimeter at three times the 8-pixel spacing gives a dozen or so points.
    EXPECT_GE(with.size(), 10u);
    EXPECT_GT(with.size(), 2u * without_edges.size());

    // Every extra point sits on the contour, never in the flat interior or the background. The
    // tolerance covers the pyramid: a sample found on a level-2 image maps back to level 0 within
    // a couple of pixels of the contour it came from.
    for (const auto& keypoint : with.keypoints) {
        const double x = keypoint.x_px;
        const double y = keypoint.y_px;
        const bool on_contour = (x > 25.0 && x < 35.0) || (x > 125.0 && x < 135.0) ||
                                (y > 25.0 && y < 35.0) || (y > 85.0 && y < 95.0);
        EXPECT_TRUE(on_contour) << "edge point off the contour at " << x << "," << y;
    }
    EXPECT_EQ(with.keypoints.size(), with.descriptors.size());
}

TEST(EdgePoints, AreDeterministic) {
    const GrayImage image = flatRectangle();
    FeatureParams params;
    params.max_edge_points_per_level = 300;
    const FeatureSet first = scand3d::feature::extractFeatures(image, params);
    const FeatureSet second = scand3d::feature::extractFeatures(image, params);

    ASSERT_EQ(first.size(), second.size());
    for (std::size_t index = 0; index < first.size(); ++index) {
        EXPECT_DOUBLE_EQ(first.keypoints[index].x_px, second.keypoints[index].x_px);
        EXPECT_DOUBLE_EQ(first.keypoints[index].y_px, second.keypoints[index].y_px);
        EXPECT_EQ(first.descriptors[index], second.descriptors[index]);
    }
}
