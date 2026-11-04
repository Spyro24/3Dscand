#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>

#include "sfm/session.hpp"
#include "sfm/tracker.hpp"
#include "synthetic/sfm_scan.hpp"
#include "synthetic/synthetic_scene.hpp"

namespace {

namespace syn = scand3d::synthetic;

constexpr int kViewCount = 24;
constexpr int kWidthPx = 800;
constexpr int kHeightPx = 600;
constexpr double kFocalPx = 600.0;

std::string unusedDirectory() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return (std::filesystem::temp_directory_path() /
            ("scand3d-map-" + std::to_string(stamp)))
        .string();
}

}  // namespace

// A scan has to survive the process: storing the map and loading it into a fresh session must
// give back the same views and points, and a frame the map has seen must relocalise against it.
TEST(MapFile, RoundTripsAScanAndRelocalisesAgainstIt) {
    const syn::Scene scene = syn::makeTexturedBoxScene(kViewCount, 1.5, 0.45, kWidthPx,
                                                       kHeightPx, kFocalPx, 12, 20261104u);
    scand3d::feature::FeatureParams params;
    params.fast_threshold = 12;
    params.max_features = 2000;

    scand3d::sfm::Session session(scand3d::sfm::SfmOptions{}, params);
    for (int index = 0; index < 6; ++index) {
        const syn::Image image = syn::render(scene, index);
        const syn::Camera& camera = scene.cameras[static_cast<std::size_t>(index)];
        session.addView(image.width_px, image.height_px, image.pixels.data(), camera.focal_px,
                        camera.cx_px, camera.cy_px);
    }
    const scand3d::sfm::ReconstructionReport report = session.run();
    ASSERT_GE(report.views_registered, 2) << "nothing to store unless the synthetic scan builds";
    const int points = session.pointCount();
    ASSERT_GT(points, 0);

    const std::string directory = unusedDirectory();
    ASSERT_TRUE(session.saveMap(directory));

    scand3d::sfm::Session restored(scand3d::sfm::SfmOptions{}, params);
    ASSERT_TRUE(restored.loadMap(directory));
    EXPECT_EQ(restored.viewCount(), session.viewCount());
    EXPECT_EQ(restored.pointCount(), points);
    EXPECT_EQ(restored.report().views_registered, report.views_registered);

    const syn::Image image = syn::render(scene, 0);
    const syn::Camera& camera = scene.cameras[0];
    scand3d::sfm::Pose pose;
    int inliers = 0;
    const bool tracked = restored.trackPose(image.width_px, image.height_px, image.pixels.data(),
                                            camera.focal_px, camera.cx_px, camera.cy_px, pose,
                                            inliers);
    EXPECT_TRUE(tracked);
    EXPECT_GT(inliers, 0);

    std::filesystem::remove_all(directory);
}

TEST(MapFile, RefusesAnEmptyDirectory) {
    scand3d::sfm::Session session;
    EXPECT_FALSE(session.loadMap(unusedDirectory()));
}
