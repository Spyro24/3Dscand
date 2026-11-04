#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "bridge/bridge_api.hpp"
#include "synthetic/sfm_scan.hpp"
#include "synthetic/synthetic_scene.hpp"

namespace {

// The UI displays this string, so it must always be present and parseable.
TEST(BridgeApi, ReportsNonEmptyVersion) {
    const char* version = scand3d::bridge::coreVersion();
    ASSERT_NE(version, nullptr);
    EXPECT_GT(std::strlen(version), 0u);
}

TEST(BridgeApi, VersionIsSemantic) {
    const char* version = scand3d::bridge::coreVersion();
    ASSERT_NE(version, nullptr);

    int major = -1;
    int minor = -1;
    int patch = -1;
    EXPECT_EQ(3, std::sscanf(version, "%d.%d.%d", &major, &minor, &patch));
    EXPECT_GE(major, 0);
    EXPECT_GE(minor, 0);
    EXPECT_GE(patch, 0);
}

// The overlay scene is what the app renders every frame (roadmap 5.1), so the C API is the
// contract the JNI layer mirrors; a projection mistake here shows up as a misaligned overlay.
TEST(BridgeApi, OverlayProjectsPointsThroughTheScene) {
    const auto handle = scand3d::bridge::overlayCreate();
    ASSERT_NE(handle, 0u);
    EXPECT_EQ(scand3d::bridge::overlayPointCount(handle), 0);

    const double positions[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 2.0};
    const float colours[6] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    ASSERT_TRUE(scand3d::bridge::overlaySetPoints(handle, positions, colours, 2));
    EXPECT_EQ(scand3d::bridge::overlayPointCount(handle), 2);

    scand3d::bridge::OverlayCamera camera{};
    camera.rotation[0] = 1.0;
    camera.rotation[4] = 1.0;
    camera.rotation[8] = 1.0;
    camera.focal_px = 100.0;
    camera.cx_px = 50.0;
    camera.cy_px = 40.0;
    camera.image_width_px = 100;
    camera.image_height_px = 80;

    float vertices[2 * scand3d::bridge::kOverlayVertexValueCount] = {};
    const int written = scand3d::bridge::overlayProject(handle, camera, vertices, 2);
    ASSERT_EQ(written, 2);
    // Both points sit on the optical axis, the far one first.
    EXPECT_NEAR(vertices[0], 0.0f, 1e-6f);
    EXPECT_NEAR(vertices[1], 0.0f, 1e-6f);
    EXPECT_FLOAT_EQ(vertices[2], 0.0f);
    EXPECT_FLOAT_EQ(vertices[3], 1.0f);

    scand3d::bridge::overlayDestroy(handle);
    EXPECT_EQ(scand3d::bridge::overlayPointCount(handle), 0);
}

// Array-length mismatches are caught by the JNI layer, which is the only caller that knows the
// lengths; this C API is handed raw pointers and must reject the input it can see.
TEST(BridgeApi, OverlayRejectsUnusablePointsAndUnknownHandles) {
    const auto handle = scand3d::bridge::overlayCreate();
    ASSERT_NE(handle, 0u);

    const double positions[3] = {0.0, 0.0, 1.0};
    const float colours[3] = {1.0f, 1.0f, 1.0f};
    const double nan_positions[3] = {0.0, 0.0, std::nan("")};

    EXPECT_FALSE(scand3d::bridge::overlaySetPoints(handle, nan_positions, colours, 1));
    EXPECT_FALSE(scand3d::bridge::overlaySetPoints(handle, nullptr, colours, 1));
    EXPECT_FALSE(scand3d::bridge::overlaySetPoints(handle, positions, nullptr, 1));
    EXPECT_FALSE(scand3d::bridge::overlaySetPoints(handle, positions, colours, -1));
    EXPECT_EQ(scand3d::bridge::overlayPointCount(handle), 0);

    scand3d::bridge::overlayDestroy(handle);
    EXPECT_FALSE(scand3d::bridge::overlaySetPoints(handle, positions, colours, 1));
    EXPECT_EQ(scand3d::bridge::overlayPointCount(handle), 0);
}

// The dense stage reaches the app through this C API (roadmap 6): the app feeds the poses the
// sparse stage found and expects a fused cloud for the overlay. A bridge that fused nothing would
// leave the preview on the sparse map with no way to tell why.
TEST(BridgeApi, DenseRunFusesPosedViews) {
    const int view_count = 8;
    const auto scene = scand3d::synthetic::makeNoiseTexturedBoxScene(
        view_count, 1.5, 0.6, 320, 240, 0.75 * 320, 7u);
    const auto handle = scand3d::bridge::denseCreate();
    ASSERT_NE(handle, 0u);

    std::vector<double> hints;
    for (const auto& vertex : scene.mesh.vertices) {
        hints.push_back(vertex.position_m[0]);
        hints.push_back(vertex.position_m[1]);
        hints.push_back(vertex.position_m[2]);
    }
    scand3d::bridge::denseSetHints(handle, hints.data(), static_cast<int>(hints.size() / 3));

    for (int index = 0; index < view_count; ++index) {
        const auto image = scand3d::synthetic::render(scene, index);
        const auto& camera = scene.cameras[static_cast<std::size_t>(index)];
        const auto pose = scand3d::synthetic::poseOf(camera);
        const int added = scand3d::bridge::denseAddView(
            handle, image.pixels.data(), image.width_px, image.height_px, camera.focal_px,
            camera.cx_px, camera.cy_px, pose.rotation.data(), pose.camera_centre_m.data());
        EXPECT_EQ(added, index);
    }

    const int points = scand3d::bridge::denseRun(handle, 120, 0.05, 20.0);
    ASSERT_GT(points, 100);
    EXPECT_DOUBLE_EQ(scand3d::bridge::denseProgress(handle), 1.0);
    EXPECT_EQ(scand3d::bridge::densePointCount(handle), points);

    std::vector<double> positions(static_cast<std::size_t>(points) * 3);
    EXPECT_EQ(scand3d::bridge::denseCopyPoints(handle, positions.data(), points), points);
    // The cloud has to sit where the box is, not at the origin of an empty result.
    bool any_offset = false;
    for (int index = 0; index < points; ++index) {
        const double* point = positions.data() + index * 3;
        if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]) &&
            (std::abs(point[0]) > 0.05 || std::abs(point[1]) > 0.05)) {
            any_offset = true;
            break;
        }
    }
    EXPECT_TRUE(any_offset);

    scand3d::bridge::denseDestroy(handle);
    EXPECT_EQ(scand3d::bridge::densePointCount(handle), 0);
}

TEST(BridgeApi, DenseRejectsUnknownHandlesAndEmptyViews) {
    constexpr scand3d::bridge::DenseHandle unknown = 999999u;
    EXPECT_EQ(scand3d::bridge::denseRun(unknown, 160, 0.1, 10.0), -1);
    EXPECT_EQ(scand3d::bridge::densePointCount(unknown), 0);
    EXPECT_DOUBLE_EQ(scand3d::bridge::denseProgress(unknown), 0.0);
    EXPECT_EQ(
        -1, scand3d::bridge::denseAddView(
                unknown, nullptr, 0, 0, 0.0, 0.0, 0.0, nullptr, nullptr));
    scand3d::bridge::denseCancel(unknown);
    scand3d::bridge::denseDestroy(unknown);
}

}  // namespace
