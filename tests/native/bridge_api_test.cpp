#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "bridge/bridge_api.hpp"

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

}  // namespace
