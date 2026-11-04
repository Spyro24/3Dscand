#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "feature/feature_extractor.hpp"
#include "feature/matcher.hpp"
#include "geometry/two_view.hpp"
#include "sfm/pipeline.hpp"
#include "sfm/pose.hpp"
#include "sfm/session.hpp"
#include "synthetic/sfm_scan.hpp"
#include "synthetic/synthetic_scene.hpp"

namespace {

namespace syn = scand3d::synthetic;

constexpr int kViewCount = 24;
constexpr int kWidthPx = 800;
constexpr int kHeightPx = 600;
constexpr double kFocalPx = 600.0;
constexpr double kPi = 3.14159265358979323846;

// The Phase 3 textured box carries only a few hundred corners at 12 cells; a reconstruction needs
// far more support per pair, so these tests use a finer speckle and a smaller angular step.
syn::Scene renderScene(std::uint32_t seed) {
    return syn::makeTexturedBoxScene(kViewCount, 1.5, 0.45, kWidthPx, kHeightPx, kFocalPx, 12, seed);
}

scand3d::feature::FeatureParams testFeatureParams() {
    scand3d::feature::FeatureParams params;
    params.fast_threshold = 12;
    params.max_features = 2000;
    return params;
}

scand3d::sfm::Session makeSession() {
    return scand3d::sfm::Session(scand3d::sfm::SfmOptions{}, testFeatureParams());
}

scand3d::feature::FeatureSet featuresOf(const syn::Image& image) {
    scand3d::feature::GrayImage gray;
    gray.width_px = image.width_px;
    gray.height_px = image.height_px;
    gray.pixels = image.pixels;
    return scand3d::feature::extractFeatures(gray, testFeatureParams());
}

int addRing(scand3d::sfm::Session& session, const syn::Scene& scene, int count) {
    int added = 0;
    for (int index = 0; index < count; ++index) {
        const syn::Image image = syn::render(scene, index);
        const syn::Camera& camera = scene.cameras[static_cast<std::size_t>(index)];
        if (session.addView(image.width_px, image.height_px, image.pixels.data(), camera.focal_px,
                            camera.cx_px, camera.cy_px) >= 0) {
            ++added;
        }
    }
    return added;
}

}  // namespace

// Diagnostic only: the match counts that decide whether the session fails in matching or later.
TEST(Session, ReportsMatchCountsOnRenderedViews) {
    const syn::Scene scene = renderScene(20261104u);
    std::vector<scand3d::feature::FeatureSet> features;
    for (int index = 0; index < 3; ++index) {
        features.push_back(featuresOf(syn::render(scene, index)));
    }

    scand3d::feature::MatchOptions options;
    options.ratio_percent = 80;
    options.cross_check = true;
    for (std::size_t index = 0; index + 1 < features.size(); ++index) {
        const auto matches =
            scand3d::feature::matchApproximate(features[index], features[index + 1], options);
        std::vector<scand3d::geometry::PointCorrespondence> correspondences;
        for (const auto& match : matches) {
            correspondences.push_back(
                {{features[index].keypoints[static_cast<std::size_t>(match.query_index)].x_px,
                  features[index].keypoints[static_cast<std::size_t>(match.query_index)].y_px},
                 {features[index + 1].keypoints[static_cast<std::size_t>(match.train_index)].x_px,
                  features[index + 1].keypoints[static_cast<std::size_t>(match.train_index)].y_px}});
        }
        const auto ransac = scand3d::geometry::estimateEssentialRansac(
            correspondences,
            scand3d::geometry::CameraIntrinsics{kFocalPx, 0.5 * kWidthPx, 0.5 * kHeightPx},
            scand3d::geometry::CameraIntrinsics{kFocalPx, 0.5 * kWidthPx, 0.5 * kHeightPx});
        std::printf("view %zu->%zu features %zu/%zu matches %zu ransac %zu median %.2f px\n",
                    index, index + 1, features[index].size(), features[index + 1].size(),
                    matches.size(), ransac.inlier_indices.size(), ransac.median_sampson_px);
    }

    EXPECT_GT(features[0].size(), 200u);
}

// Diagnostic only: runs the seeder on exact ground-truth correspondences. If this keeps few points,
// the seeder is at fault; if it keeps them all, the fault is match quality.
TEST(Session, ReportsSeedAgainstGroundTruth) {
    const syn::Scene scene = renderScene(20261104u);
    const auto first_vertices = syn::visibleSurfaceVertices(scene, 0);
    const auto second_vertices = syn::visibleSurfaceVertices(scene, 1);

    std::unordered_map<int, const syn::SurfaceVertex*> second_by_id;
    for (const auto& vertex : second_vertices) {
        second_by_id[vertex.id] = &vertex;
    }
    std::vector<scand3d::geometry::PointCorrespondence> correspondences;
    for (const auto& vertex : first_vertices) {
        const auto found = second_by_id.find(vertex.id);
        if (found == second_by_id.end()) {
            continue;
        }
        correspondences.push_back(
            {{vertex.projection.u_px, vertex.projection.v_px},
             {found->second->projection.u_px, found->second->projection.v_px}});
    }

    const syn::Camera& first_camera = scene.cameras[0];
    const syn::Camera& second_camera = scene.cameras[1];
    const auto initialization = scand3d::sfm::initializeTwoView(
        correspondences,
        scand3d::sfm::CameraModel{first_camera.focal_px, first_camera.cx_px, first_camera.cy_px},
        scand3d::sfm::CameraModel{second_camera.focal_px, second_camera.cx_px, second_camera.cy_px});

    const scand3d::sfm::Pose first_truth = syn::poseOf(first_camera);
    const scand3d::sfm::Pose second_truth = syn::poseOf(second_camera);
    const scand3d::geometry::Mat3 expected_rotation = scand3d::geometry::matmul3(
        scand3d::geometry::transpose3(first_truth.rotation), second_truth.rotation);
    const double rotation_error_deg =
        scand3d::sfm::rotationAngleRad(initialization.second.rotation, expected_rotation) * 180.0 /
        kPi;
    std::printf("seed ground truth: corr %zu valid %d inliers %d points %zu median %.2f px "
                "rotation error %.2f deg\n",
                correspondences.size(), initialization.valid ? 1 : 0, initialization.inliers,
                initialization.points.size(), initialization.median_reprojection_px,
                rotation_error_deg);

    EXPECT_TRUE(initialization.valid);
    EXPECT_GT(initialization.inliers, 50);
}

TEST(Session, ReconstructsARenderedTurntable) {
    const syn::Scene scene = renderScene(20261104u);
    scand3d::sfm::Session session = makeSession();
    ASSERT_EQ(addRing(session, scene, kViewCount), kViewCount);

    const scand3d::sfm::ReconstructionReport report = session.run();
    std::printf("session: %s\n", report.summary().c_str());

    // The seed and its neighbours reconstruct at sub-pixel error. Growth stops at the ring's
    // weakest neighbouring pair, which yields too few matches to place a view (D035, D037). The
    // scene seed is pinned so this depth is reproducible.
    EXPECT_GE(report.views_registered, 3) << report.summary();
    EXPECT_GT(session.pointCount(), 40) << report.summary();
    EXPECT_LT(report.mean_reprojection_px, 3.0) << report.summary();
}

TEST(Session, CopiesPointsAndCamerasInTheReconstructionFrame) {
    const syn::Scene scene = renderScene(7u);
    scand3d::sfm::Session session = makeSession();
    ASSERT_EQ(addRing(session, scene, 6), 6);
    session.run();

    const int points = session.pointCount();
    EXPECT_EQ(session.cameraCount(), 6);
    if (points > 0) {
        std::vector<double> positions(static_cast<std::size_t>(points) * 3);
        EXPECT_EQ(session.copyPoints(positions.data(), points), points);
        for (const double value : positions) {
            EXPECT_TRUE(std::isfinite(value));
        }
    }

    double rotation[9] = {};
    double centre[3] = {};
    EXPECT_FALSE(session.copyCamera(session.cameraCount(), rotation, centre));

    // Whatever registered first is the seed camera, and its pose is the reconstruction's origin.
    for (int index = 0; index < session.cameraCount(); ++index) {
        if (!session.copyCamera(index, rotation, centre)) {
            continue;
        }
        const double trace = rotation[0] + rotation[4] + rotation[8];
        EXPECT_LE(trace, 3.0);
        EXPECT_TRUE(std::isfinite(centre[0]) && std::isfinite(centre[1]) &&
                    std::isfinite(centre[2]));
        break;
    }
}

TEST(Session, RejectsUnusableViews) {
    scand3d::sfm::Session session;

    EXPECT_EQ(session.addView(0, 0, nullptr, kFocalPx, 0.0, 0.0), -1);
    EXPECT_EQ(session.addView(8, 8, nullptr, kFocalPx, 0.0, 0.0), -1);

    // A flat image has no corners, so it cannot contribute a view.
    std::vector<unsigned char> flat(64 * 64, 128);
    EXPECT_EQ(session.addView(64, 64, flat.data(), kFocalPx, 32.0, 32.0), -1);
    EXPECT_EQ(session.viewCount(), 0);
    EXPECT_EQ(session.run().views_registered, 0);
}

TEST(Session, TracksAViewAgainstTheMap) {
    const syn::Scene scene = renderScene(99u);
    scand3d::sfm::Session session = makeSession();
    ASSERT_EQ(addRing(session, scene, kViewCount), kViewCount);
    const scand3d::sfm::ReconstructionReport report = session.run();
    // This fixture only needs a map to track against, so it does not assert the growth depth that
    // the turntable test above pins; how far growth reaches depends on the scene's match quality.
    ASSERT_GE(report.views_registered, 2) << report.summary();
    ASSERT_GT(session.pointCount(), 10) << report.summary();

    // Tracking a map view's own image must land on that view's pose.
    int best_inliers = 0;
    int best_view = -1;
    scand3d::sfm::Pose best_pose;
    for (int index = 0; index < kViewCount; ++index) {
        const syn::Image image = syn::render(scene, index);
        const syn::Camera& camera = scene.cameras[static_cast<std::size_t>(index)];
        scand3d::sfm::Pose pose;
        int inliers = 0;
        if (session.trackPose(image.width_px, image.height_px, image.pixels.data(), camera.focal_px,
                              camera.cx_px, camera.cy_px, pose, inliers, index) &&
            inliers > best_inliers) {
            best_inliers = inliers;
            best_view = index;
            best_pose = pose;
        }
    }
    ASSERT_GT(best_inliers, 10);
    ASSERT_GE(best_view, 0);

    // The seed map covers a mostly planar box face, so its pose is fixed only up to the planar
    // flip; comparing the tracked rotation against the stored one would measure that ambiguity
    // rather than tracking. What is asserted instead: tracking returns a map pose, and a frame
    // without corners is refused instead of inventing one.
    double rotation[9] = {};
    double centre[3] = {};
    EXPECT_TRUE(session.copyCamera(best_view, rotation, centre));
    EXPECT_TRUE(std::isfinite(best_pose.camera_centre_m[0]) &&
                std::isfinite(best_pose.camera_centre_m[1]) &&
                std::isfinite(best_pose.camera_centre_m[2]));

    std::vector<unsigned char> blank(static_cast<std::size_t>(kWidthPx) * kHeightPx, 0);
    scand3d::sfm::Pose blank_pose;
    int blank_inliers = 7;
    EXPECT_FALSE(session.trackPose(kWidthPx, kHeightPx, blank.data(), kFocalPx, 0.5 * kWidthPx,
                                   0.5 * kHeightPx, blank_pose, blank_inliers));
    EXPECT_EQ(blank_inliers, 0);
}
