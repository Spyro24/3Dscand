#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "synthetic/synthetic_scene.hpp"

namespace {

using scand3d::synthetic::Camera;
using scand3d::synthetic::Image;
using scand3d::synthetic::Mat4;
using scand3d::synthetic::Mesh;
using scand3d::synthetic::Projection;
using scand3d::synthetic::Scene;
using scand3d::synthetic::Vec3;

Mat4 identityPose() {
    return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Camera squareCamera(int size_px, double focal_px) {
    Camera camera;
    camera.width_px = size_px;
    camera.height_px = size_px;
    camera.focal_px = focal_px;
    camera.cx_px = 0.5 * static_cast<double>(size_px);
    camera.cy_px = 0.5 * static_cast<double>(size_px);
    camera.pose_camera_to_world = identityPose();
    return camera;
}

void addTriangle(Mesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& normal, std::uint8_t albedo) {
    const auto base = static_cast<int>(mesh.vertices.size());
    for (const Vec3& position : {a, b, c}) {
        scand3d::synthetic::Vertex vertex;
        vertex.position_m = position;
        vertex.normal = normal;
        vertex.albedo = albedo;
        mesh.vertices.push_back(vertex);
    }
    mesh.triangles.push_back({base + 0, base + 1, base + 2});
}

bool anyNonZeroNear(const Image& image, double u_px, double v_px) {
    const int centre_x = static_cast<int>(std::lround(u_px));
    const int centre_y = static_cast<int>(std::lround(v_px));
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (image.at(centre_x + dx, centre_y + dy) > 0) {
                return true;
            }
        }
    }
    return false;
}

TEST(TurntableObject, HasIndexedGeometryAndVaryingAlbedo) {
    const Mesh mesh = scand3d::synthetic::makeTurntableObject();

    ASSERT_GT(mesh.vertices.size(), 0u);
    ASSERT_GT(mesh.triangles.size(), 0u);
    EXPECT_EQ(0u, mesh.triangles.size() % 2u);  // quads split into pairs

    std::set<std::uint8_t> albedos;
    for (const auto& triangle : mesh.triangles) {
        for (int index : triangle.vertex_index) {
            ASSERT_GE(index, 0);
            ASSERT_LT(static_cast<std::size_t>(index), mesh.vertices.size());
        }
    }
    for (const auto& vertex : mesh.vertices) {
        albedos.insert(vertex.albedo);
    }
    EXPECT_GE(albedos.size(), 2u);  // one flat tone would make every view identical
}

TEST(TurntableScene, PlacesCamerasEvenlyOnARing) {
    const Scene scene = scand3d::synthetic::makeTurntableScene(8, 2.0, 0.6, 320, 240, 400.0);

    ASSERT_EQ(8u, scene.cameras.size());
    for (const Camera& camera : scene.cameras) {
        const double x_m = camera.pose_camera_to_world[3];
        const double y_m = camera.pose_camera_to_world[7];
        const double z_m = camera.pose_camera_to_world[11];
        EXPECT_NEAR(2.0, std::hypot(x_m, y_m), 1e-9);
        EXPECT_NEAR(0.6, z_m, 1e-9);
    }

    // Consecutive cameras must be the same angular step apart.
    const double expected_step_rad = 2.0 * std::acos(-1.0) / 8.0;
    for (std::size_t i = 0; i < scene.cameras.size(); ++i) {
        const auto& a = scene.cameras[i].pose_camera_to_world;
        const auto& b = scene.cameras[(i + 1) % scene.cameras.size()].pose_camera_to_world;
        const double angle_a = std::atan2(a[7], a[3]);
        const double angle_b = std::atan2(b[7], b[3]);
        EXPECT_NEAR(expected_step_rad, std::fmod(angle_b - angle_a + 2.0 * std::acos(-1.0),
                                                2.0 * std::acos(-1.0)),
                    1e-9);
    }
}

TEST(TurntableScene, CamerasLookAtTheObjectCentre) {
    const Scene scene = scand3d::synthetic::makeTurntableScene(6, 1.5, 0.4, 320, 240, 380.0);
    const Vec3 target_m{0.0, 0.0, 0.5};

    for (const Camera& camera : scene.cameras) {
        const Projection projection = scand3d::synthetic::projectPoint(camera, target_m);
        ASSERT_TRUE(projection.in_front);
        EXPECT_NEAR(camera.cx_px, projection.u_px, 1e-6);
        EXPECT_NEAR(camera.cy_px, projection.v_px, 1e-6);
    }
}

TEST(Projection, OneMetreAlongTheOpticalAxisLandsOnThePrincipalPoint) {
    const Camera camera = squareCamera(200, 200.0);

    const Projection ahead = scand3d::synthetic::projectPoint(camera, {0.0, 0.0, 1.0});
    ASSERT_TRUE(ahead.in_front);
    EXPECT_NEAR(1.0, ahead.depth_m, 1e-12);
    EXPECT_NEAR(camera.cx_px, ahead.u_px, 1e-9);
    EXPECT_NEAR(camera.cy_px, ahead.v_px, 1e-9);

    const Projection behind = scand3d::synthetic::projectPoint(camera, {0.0, 0.0, -1.0});
    EXPECT_FALSE(behind.in_front);
}

TEST(Projection, CameraYMapsDownTheImage) {
    const Camera camera = squareCamera(200, 200.0);
    // +Y is down in camera axes, so a point above the centre must land above it in pixels.
    const Projection projection = scand3d::synthetic::projectPoint(camera, {0.0, -0.5, 1.0});
    ASSERT_TRUE(projection.in_front);
    EXPECT_LT(projection.v_px, camera.cy_px);
}

TEST(Render, ZBufferKeepsTheNearerSurface) {
    Scene scene;
    scene.cameras.push_back(squareCamera(200, 200.0));

    // Dark triangle one metre in front, bright triangle half a metre behind it.
    addTriangle(scene.mesh, {-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0}, {0.0, 1.0, 1.0}, {0.0, 0.0, -1.0}, 40);
    addTriangle(scene.mesh, {-2.0, -2.0, 1.5}, {2.0, -2.0, 1.5}, {0.0, 2.0, 1.5}, {0.0, 0.0, -1.0}, 240);

    const Image both = scand3d::synthetic::render(scene, 0);
    const std::uint8_t centre = both.at(100, 100);
    EXPECT_GT(centre, 0);
    EXPECT_LT(centre, 40);  // shaded near triangle, not the brighter far one

    Scene far_only;
    far_only.cameras.push_back(squareCamera(200, 200.0));
    addTriangle(far_only.mesh, {-2.0, -2.0, 1.5}, {2.0, -2.0, 1.5}, {0.0, 2.0, 1.5}, {0.0, 0.0, -1.0}, 240);
    EXPECT_GT(scand3d::synthetic::render(far_only, 0).at(100, 100), 40);
}

TEST(Render, ObjectFillsTheSilhouetteAndBackgroundStaysEmpty) {
    const Scene scene = scand3d::synthetic::makeTurntableScene(6, 1.8, 0.5, 256, 256, 300.0);

    const Image image = scand3d::synthetic::render(scene, 0);
    ASSERT_EQ(256u * 256u, image.pixels.size());
    EXPECT_EQ(0, image.at(0, 0));
    EXPECT_GT(image.at(128, 128), 0);

    const std::size_t covered = static_cast<std::size_t>(
        std::count_if(image.pixels.begin(), image.pixels.end(), [](std::uint8_t p) { return p > 0; }));
    EXPECT_GT(covered, image.pixels.size() / 20u);  // at least 5 percent of the frame
    EXPECT_LT(covered, image.pixels.size());        // and never the whole frame
}

TEST(Render, IsDeterministic) {
    const Scene scene = scand3d::synthetic::makeTurntableScene(4, 2.0, 0.7, 128, 128, 180.0);

    const Image first = scand3d::synthetic::render(scene, 1);
    const Image second = scand3d::synthetic::render(scene, 1);
    EXPECT_EQ(first.pixels, second.pixels);

    // Different viewpoints must actually differ, otherwise the ground truth is useless.
    const Image other = scand3d::synthetic::render(scene, 3);
    EXPECT_NE(first.pixels, other.pixels);
}

TEST(VisibleVertices, ProjectInsideTheRenderedSilhouette) {
    const Scene scene = scand3d::synthetic::makeTurntableScene(6, 1.8, 0.5, 256, 256, 300.0);
    const Image image = scand3d::synthetic::render(scene, 0);

    const auto visible = scand3d::synthetic::visibleVertices(scene, 0);
    ASSERT_FALSE(visible.empty());
    for (const auto& [vertex_index, projection] : visible) {
        EXPECT_GE(vertex_index, 0);
        EXPECT_TRUE(anyNonZeroNear(image, projection.u_px, projection.v_px));
    }
}

}  // namespace
