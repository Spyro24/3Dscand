#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "overlay/object_clusters.hpp"

namespace {

using scand3d::overlay::ObjectClusterOptions;
using scand3d::overlay::ObjectClusterResult;
using scand3d::overlay::OverlayPoint;
using scand3d::overlay::colourByObject;

/// A solid block of points with `per_axis^3` members, spaced so neighbours are linked.
void addBlob(
    std::vector<OverlayPoint>& points, double centre_x, double centre_y, double centre_z,
    int per_axis, double spacing_m) {
    for (int z = 0; z < per_axis; ++z) {
        for (int y = 0; y < per_axis; ++y) {
            for (int x = 0; x < per_axis; ++x) {
                OverlayPoint point;
                point.position_m = {centre_x + x * spacing_m, centre_y + y * spacing_m,
                                    centre_z + z * spacing_m};
                points.push_back(point);
            }
        }
    }
}

double colourDistance(const OverlayPoint& a, const OverlayPoint& b) {
    return std::fabs(a.rgb[0] - b.rgb[0]) + std::fabs(a.rgb[1] - b.rgb[1]) +
           std::fabs(a.rgb[2] - b.rgb[2]);
}

}  // namespace

TEST(ObjectClusters, SeparatesTwoBlobsAndColoursThemDifferently) {
    std::vector<OverlayPoint> points;
    addBlob(points, 0.0, 0.0, 1.0, 3, 0.005);  // 27 points
    addBlob(points, 0.5, 0.0, 1.0, 3, 0.005);  // 27 points, half a metre away

    const ObjectClusterResult result = colourByObject(points);
    EXPECT_EQ(2, result.objects);
    EXPECT_EQ(27, result.largest_object);
    EXPECT_EQ(54u, result.coloured_points);

    // The two objects must not end up on the same shade, or the view would show one object.
    EXPECT_GT(colourDistance(points.front(), points.back()), 0.3);
}

TEST(ObjectClusters, LeavesGroupsBelowTheMinimumAlone) {
    std::vector<OverlayPoint> points;
    addBlob(points, 0.0, 0.0, 1.0, 2, 0.004);  // 8 points, below the default 12

    const ObjectClusterResult result = colourByObject(points);
    EXPECT_EQ(0, result.objects);
    EXPECT_EQ(0u, result.coloured_points);
    // Unclustered points keep the colour they arrived with, which is how photo detail survives.
    EXPECT_FLOAT_EQ(1.0f, points.front().rgb[0]);
}

TEST(ObjectClusters, LinksAChainIntoOneObject) {
    std::vector<OverlayPoint> points;
    for (int index = 0; index < 40; ++index) {
        OverlayPoint point;
        point.position_m = {0.01 * index, 0.0, 1.0};
        points.push_back(point);
    }

    const ObjectClusterResult result = colourByObject(points);
    EXPECT_EQ(1, result.objects);
    EXPECT_EQ(40, result.largest_object);
    EXPECT_EQ(40u, result.coloured_points);
}

TEST(ObjectClusters, RejectsUnusableOptions) {
    std::vector<OverlayPoint> points(20);
    ObjectClusterOptions no_link;
    no_link.link_distance_m = 0.0;
    EXPECT_EQ(0, colourByObject(points, no_link).objects);

    ObjectClusterOptions no_minimum;
    no_minimum.min_points = 0;
    EXPECT_EQ(0, colourByObject(points, no_minimum).objects);
}
