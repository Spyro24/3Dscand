#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "overlay/overlay_scene.hpp"

namespace {

using scand3d::geometry::identity3;
using scand3d::geometry::Mat3;
using scand3d::overlay::ColoredVertex;
using scand3d::overlay::OverlayCamera;
using scand3d::overlay::OverlayPoint;
using scand3d::overlay::OverlayScene;

// 100x80 image, 100 px focal length: one pixel of image is one percent of NDC in x.
OverlayCamera testCamera() {
    OverlayCamera camera;
    camera.focal_px = 100.0;
    camera.cx_px = 50.0;
    camera.cy_px = 40.0;
    camera.image_width_px = 100;
    camera.image_height_px = 80;
    return camera;
}

OverlayPoint pointAt(double x, double y, double z) {
    OverlayPoint point;
    point.position_m = {x, y, z};
    return point;
}

TEST(OverlayScene, PointOnOpticalAxisLandsInTheImageCentre) {
    OverlayScene scene;
    scene.setPoints({pointAt(0.0, 0.0, 2.0)});

    const auto vertices = scene.project(testCamera());

    ASSERT_EQ(vertices.size(), 1u);
    EXPECT_NEAR(vertices[0].ndc[0], 0.0, 1e-6);
    EXPECT_NEAR(vertices[0].ndc[1], 0.0, 1e-6);
    EXPECT_NEAR(vertices[0].depth_m, 2.0, 1e-6);
}

TEST(OverlayScene, CameraXAndYFollowTheCvConvention) {
    // Camera axes are +X right, +Y down (D012). A point right of the axis moves NDC +x; a point
    // below the axis moves NDC -y, because NDC y points up.
    OverlayScene scene;
    scene.setPoints({pointAt(0.5, 0.0, 1.0), pointAt(0.0, 0.4, 1.0)});

    const auto vertices = scene.project(testCamera());

    ASSERT_EQ(vertices.size(), 2u);
    // Right: pixel 100, i.e. NDC +1.0.
    EXPECT_NEAR(vertices[0].ndc[0], 1.0, 1e-6);
    EXPECT_NEAR(vertices[0].ndc[1], 0.0, 1e-6);
    // Down: pixel 80 on a 80 px tall image, i.e. the bottom edge, NDC -1.0.
    EXPECT_NEAR(vertices[1].ndc[0], 0.0, 1e-6);
    EXPECT_NEAR(vertices[1].ndc[1], -1.0, 1e-6);
}

TEST(OverlayScene, PointsBehindTheCameraAreDropped) {
    OverlayScene scene;
    scene.setPoints({pointAt(0.0, 0.0, -1.0), pointAt(0.0, 0.0, 0.0), pointAt(0.0, 0.0, 1.0)});

    const auto vertices = scene.project(testCamera());

    ASSERT_EQ(vertices.size(), 1u);
    EXPECT_NEAR(vertices[0].depth_m, 1.0, 1e-6);
}

TEST(OverlayScene, ColourIsCarriedThrough) {
    OverlayScene scene;
    OverlayPoint point = pointAt(0.0, 0.0, 1.0);
    point.rgb[0] = 0.25f;
    point.rgb[1] = 0.5f;
    point.rgb[2] = 0.75f;
    scene.setPoints({point});

    const auto vertices = scene.project(testCamera());

    ASSERT_EQ(vertices.size(), 1u);
    EXPECT_FLOAT_EQ(vertices[0].rgb[0], 0.25f);
    EXPECT_FLOAT_EQ(vertices[0].rgb[1], 0.5f);
    EXPECT_FLOAT_EQ(vertices[0].rgb[2], 0.75f);
}

TEST(OverlayScene, VerticesAreSortedFarToNear) {
    OverlayScene scene;
    scene.setPoints({pointAt(0.0, 0.0, 1.0), pointAt(0.0, 0.0, 5.0), pointAt(0.0, 0.0, 3.0)});

    const auto vertices = scene.project(testCamera());

    ASSERT_EQ(vertices.size(), 3u);
    EXPECT_NEAR(vertices[0].depth_m, 5.0, 1e-6);
    EXPECT_NEAR(vertices[1].depth_m, 3.0, 1e-6);
    EXPECT_NEAR(vertices[2].depth_m, 1.0, 1e-6);
}

TEST(OverlayScene, CameraPoseIsApplied) {
    // Camera turned 90 degrees about its own Y, so camera forward points along world +X; a point
    // two meters along +X must then sit on the optical axis at depth two.
    OverlayCamera camera = testCamera();
    camera.rotation = Mat3{0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0};

    OverlayScene scene;
    scene.setPoints({pointAt(2.0, 0.0, 0.0)});

    const auto vertices = scene.project(camera);

    ASSERT_EQ(vertices.size(), 1u);
    EXPECT_NEAR(vertices[0].depth_m, 2.0, 1e-6);
    EXPECT_NEAR(vertices[0].ndc[0], 0.0, 1e-6);
    EXPECT_NEAR(vertices[0].ndc[1], 0.0, 1e-6);
}

TEST(OverlayScene, CameraCentreMovesTheOrigin) {
    OverlayCamera camera = testCamera();
    camera.centre_m = {1.0, 2.0, 3.0};

    OverlayScene scene;
    scene.setPoints({pointAt(1.0, 2.0, 6.0)});

    const auto vertices = scene.project(camera);

    ASSERT_EQ(vertices.size(), 1u);
    EXPECT_NEAR(vertices[0].depth_m, 3.0, 1e-6);
    EXPECT_NEAR(vertices[0].ndc[0], 0.0, 1e-6);
}

TEST(OverlayScene, DegenerateCameraYieldsNothing) {
    OverlayScene scene;
    scene.setPoints({pointAt(0.0, 0.0, 1.0)});

    OverlayCamera no_image = testCamera();
    no_image.image_width_px = 0;
    EXPECT_TRUE(scene.project(no_image).empty());

    OverlayCamera no_focal = testCamera();
    no_focal.focal_px = 0.0;
    EXPECT_TRUE(scene.project(no_focal).empty());
}

TEST(OverlayScene, SetPointsReplacesThePreviousScene) {
    OverlayScene scene;
    scene.setPoints({pointAt(0.0, 0.0, 1.0), pointAt(0.0, 0.0, 2.0)});
    ASSERT_EQ(scene.pointCount(), 2u);

    scene.setPoints({pointAt(0.0, 0.0, 1.0)});

    EXPECT_EQ(scene.pointCount(), 1u);
    EXPECT_EQ(scene.project(testCamera()).size(), 1u);
}

TEST(OverlayScene, EmptySceneProjectsToNothing) {
    OverlayScene scene;

    EXPECT_EQ(scene.pointCount(), 0u);
    EXPECT_TRUE(scene.project(testCamera()).empty());
}

}  // namespace
