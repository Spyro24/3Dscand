#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "sfm/bundle_adjustment.hpp"
#include "sfm/keyframe_selection.hpp"
#include "sfm/map_store.hpp"
#include "sfm/pipeline.hpp"
#include "sfm/pnp.hpp"
#include "sfm/pose.hpp"
#include "sfm/report.hpp"
#include "sfm/triangulation.hpp"
#include "synthetic/sfm_scan.hpp"

namespace {

namespace syn = scand3d::synthetic;

using scand3d::sfm::BundleAdjustmentOptions;
using scand3d::sfm::MapStore;
using scand3d::sfm::PairMatches;
using scand3d::sfm::PnpOptions;
using scand3d::sfm::Pose;
using scand3d::sfm::Reconstruction;
using scand3d::sfm::SfmOptions;
using scand3d::sfm::Vec2;
using scand3d::sfm::Vec3;
using scand3d::sfm::ViewObservation;
using scand3d::sfm::ViewState;

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(Triangulation, RecoversAKnownPoint) {
    const syn::Camera first = syn::sfmCameraAt(0.0);
    const syn::Camera second = syn::sfmCameraAt(0.35);
    const Vec3 point{0.1, -0.05, 0.6};

    const syn::Projection a = syn::projectPoint(first, point);
    const syn::Projection b = syn::projectPoint(second, point);
    ASSERT_TRUE(a.in_front);
    ASSERT_TRUE(b.in_front);

    scand3d::sfm::TriangulationResult result;
    ASSERT_TRUE(scand3d::sfm::triangulateTwoView(
        syn::cameraModelOf(first), syn::poseOf(first), {a.u_px, a.v_px},
        syn::cameraModelOf(second), syn::poseOf(second), {b.u_px, b.v_px}, result));
    EXPECT_LT(std::abs(result.position_m[0] - point[0]), 1e-6);
    EXPECT_LT(std::abs(result.position_m[1] - point[1]), 1e-6);
    EXPECT_LT(std::abs(result.position_m[2] - point[2]), 1e-6);
    EXPECT_GT(result.max_parallax_rad, 0.1);
    EXPECT_LT(result.max_reprojection_px, 1e-6);
}

TEST(Triangulation, RejectsAZeroBaselinePair) {
    const syn::Camera camera = syn::sfmCameraAt(0.0);
    const syn::Projection projection = syn::projectPoint(camera, {0.0, 0.0, 0.6});

    scand3d::sfm::TriangulationResult result;
    EXPECT_FALSE(scand3d::sfm::triangulateTwoView(
        syn::cameraModelOf(camera), syn::poseOf(camera), {projection.u_px, projection.v_px},
        syn::cameraModelOf(camera), syn::poseOf(camera), {projection.u_px, projection.v_px},
        result));
}

TEST(TwoViewInitialization, RecoversTheCalibratedRelativePose) {
    const syn::SfmScan scan = syn::makeLatticeScan(12, 0.4);
    ASSERT_GE(scan.pairs.size(), 2u);

    const PairMatches& pair = scan.pairs.front();
    std::vector<scand3d::geometry::PointCorrespondence> correspondences;
    for (const auto& match : pair.matches) {
        correspondences.push_back(
            {scan.frames[static_cast<std::size_t>(pair.view_a)]
                 .pixels[static_cast<std::size_t>(match.first)],
             scan.frames[static_cast<std::size_t>(pair.view_b)]
                 .pixels[static_cast<std::size_t>(match.second)]});
    }

    const auto initialization = scand3d::sfm::initializeTwoView(
        correspondences, scan.frames[0].camera, scan.frames[1].camera);
    ASSERT_TRUE(initialization.valid);
    EXPECT_GE(initialization.inliers, 40);

    const Pose& first_truth = scan.ground_truth[0];
    const Pose& second_truth = scan.ground_truth[1];
    const scand3d::geometry::Mat3 expected_rotation = scand3d::geometry::matmul3(
        scand3d::geometry::transpose3(first_truth.rotation), second_truth.rotation);
    EXPECT_LT(scand3d::sfm::rotationAngleRad(initialization.second.rotation, expected_rotation) *
                  180.0 / kPi,
              1.0);

    const Vec3 delta{second_truth.camera_centre_m[0] - first_truth.camera_centre_m[0],
                     second_truth.camera_centre_m[1] - first_truth.camera_centre_m[1],
                     second_truth.camera_centre_m[2] - first_truth.camera_centre_m[2]};
    const Vec3 expected_centre = scand3d::geometry::matvec3(
        scand3d::geometry::transpose3(first_truth.rotation), delta);
    // The essential matrix only fixes the baseline up to scale, so the direction is what must match.
    const double estimated_norm = std::sqrt(scand3d::geometry::dot3(
        initialization.second.camera_centre_m, initialization.second.camera_centre_m));
    EXPECT_NEAR(1.0, estimated_norm, 0.01);
    EXPECT_LT(syn::directionErrorDeg(initialization.second.camera_centre_m, expected_centre), 1.0);
    EXPECT_LT(initialization.median_reprojection_px, 1.0);
}

TEST(PnP, RecoversAPoseWithOutliers) {
    const syn::Camera camera = syn::sfmCameraAt(0.8);
    const Pose truth = syn::poseOf(camera);
    const std::vector<Vec3> points = syn::latticePoints();

    std::mt19937 random(7);
    std::normal_distribution<double> noise(0.0, 0.3);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    std::vector<Vec3> object_points;
    std::vector<Vec2> image_points;
    for (const Vec3& point : points) {
        const syn::Projection projection = syn::projectPoint(camera, point);
        if (!projection.in_front) {
            continue;
        }
        object_points.push_back(point);
        image_points.push_back(uniform(random) < 0.2
                                   ? Vec2{projection.u_px + 40.0, projection.v_px - 35.0}
                                   : Vec2{projection.u_px + noise(random),
                                          projection.v_px + noise(random)});
    }
    ASSERT_GT(object_points.size(), 30u);

    PnpOptions options;
    options.threshold_px = 3.0;
    const auto result = scand3d::sfm::estimatePoseRansac(object_points, image_points,
                                                        syn::cameraModelOf(camera), options);
    ASSERT_TRUE(result.valid);
    EXPECT_LT(scand3d::sfm::rotationAngleRad(result.pose.rotation, truth.rotation) * 180.0 / kPi,
              0.5);
    EXPECT_LT(syn::directionErrorDeg(result.pose.camera_centre_m, truth.camera_centre_m), 1.0);
    EXPECT_LT(result.median_reprojection_px, 1.0);
}

TEST(KeyframeSelection, AppliesTheRotationAndOverlapWindow) {
    const scand3d::sfm::KeyframePolicy policy;
    EXPECT_FALSE(scand3d::sfm::evaluateKeyframe(0.02, 0.1, 0.9, policy).accept);
    EXPECT_FALSE(scand3d::sfm::evaluateKeyframe(0.9, 0.1, 0.9, policy).accept);
    EXPECT_FALSE(scand3d::sfm::evaluateKeyframe(0.3, 0.001, 0.9, policy).accept);
    EXPECT_FALSE(scand3d::sfm::evaluateKeyframe(0.3, 0.1, 0.05, policy).accept);
    EXPECT_TRUE(scand3d::sfm::evaluateKeyframe(0.3, 0.1, 0.6, policy).accept);

    const scand3d::geometry::Mat3 identity = scand3d::geometry::identity3();
    const scand3d::geometry::Mat3 turned =
        scand3d::sfm::rotationFromAxisAngle({0.0, 0.0, kPi / 2.0});
    EXPECT_NEAR(kPi / 2.0, scand3d::sfm::rotationAngleRad(identity, turned), 1e-9);
}

TEST(BundleAdjustment, ReducesTheCostOfANoisyReconstruction) {
    constexpr int kViews = 8;
    const syn::SfmScan scan = syn::makeLatticeScan(kViews, 0.6);

    Reconstruction reconstruction;
    for (int v = 0; v < kViews; ++v) {
        ViewState view;
        view.camera = scan.frames[static_cast<std::size_t>(v)].camera;
        view.registered = true;
        view.feature_count = scan.frames[static_cast<std::size_t>(v)].pixels.size();
        view.pose = scan.ground_truth[static_cast<std::size_t>(v)];
        reconstruction.views.push_back(view);
    }

    // Points start from noisy triangulations of the true poses, which is what BA has to clean up.
    for (std::size_t p = 0; p < scan.points.size(); ++p) {
        scand3d::sfm::MapPoint point;
        std::vector<ViewObservation> observations;
        for (int v = 0; v < kViews; ++v) {
            const int feature = scan.feature_of_point[static_cast<std::size_t>(v)][p];
            if (feature < 0) {
                continue;
            }
            const Vec2 pixel =
                scan.frames[static_cast<std::size_t>(v)].pixels[static_cast<std::size_t>(feature)];
            point.observations.push_back({v, feature, pixel});
            observations.push_back({reconstruction.views[static_cast<std::size_t>(v)].camera,
                                    reconstruction.views[static_cast<std::size_t>(v)].pose,
                                    pixel});
        }
        if (observations.size() < 2) {
            continue;
        }
        scand3d::sfm::TriangulationResult triangulated;
        if (!scand3d::sfm::triangulatePoint(observations, triangulated)) {
            continue;
        }
        point.position_m = triangulated.position_m;
        point.valid = true;
        reconstruction.points.push_back(std::move(point));
    }

    ASSERT_GT(reconstruction.points.size(), 40u);
    BundleAdjustmentOptions options;
    options.max_iterations = 30;
    const auto result = scand3d::sfm::optimize(reconstruction, options);
    EXPECT_LT(result.final_cost, result.initial_cost);
    EXPECT_LT(scand3d::sfm::meanReprojectionPx(reconstruction), 1.0);
}

TEST(MapStore, DropsOutlierObservationsAndMergesDuplicates) {
    // Triangulate from four consistent views, then add one wrong observation. This is the order the
    // pipeline uses: outlier removal runs on a point that is already supported.
    constexpr int kTrackViews = 4;
    constexpr int kViews = 5;
    std::vector<ViewState> views(kViews);
    for (int v = 0; v < kViews; ++v) {
        const syn::Camera camera = syn::sfmCameraAt(0.4 * v);
        views[static_cast<std::size_t>(v)].camera = syn::cameraModelOf(camera);
        views[static_cast<std::size_t>(v)].pose = syn::poseOf(camera);
        views[static_cast<std::size_t>(v)].registered = true;
    }
    MapStore store(views);

    const Vec3 point{0.0, 0.0, 0.6};
    std::vector<std::vector<scand3d::sfm::Observation>> tracks(1);
    for (int v = 0; v < kTrackViews; ++v) {
        const syn::Projection projection = syn::projectPoint(syn::sfmCameraAt(0.4 * v), point);
        tracks[0].push_back({v, 0, {projection.u_px, projection.v_px}});
    }

    store.setTracks(tracks);
    ASSERT_EQ(1, store.triangulateTracks(0.0));

    const syn::Projection outlier = syn::projectPoint(syn::sfmCameraAt(0.4 * 4), point);
    store.data().points[0].observations.push_back({4, 0, {outlier.u_px + 60.0, outlier.v_px}});

    EXPECT_EQ(1, store.removeOutlierObservations(2.0));
    ASSERT_EQ(1u, store.data().points.size());
    EXPECT_EQ(4u, store.data().points[0].observations.size());
    EXPECT_GE(store.pointForFeature(0, 0), 0);
    EXPECT_EQ(-1, store.pointForFeature(4, 0));

    // A point close enough to another that shares a view is a duplicate and gets invalidated.
    std::vector<std::vector<scand3d::sfm::Observation>> duplicates(2);
    duplicates[0] = {{0, 0, {0.0, 0.0}}, {1, 0, {0.0, 0.0}}};
    duplicates[1] = {{0, 1, {0.0, 0.0}}, {1, 1, {0.0, 0.0}}};
    store.setTracks(duplicates);
    store.data().points[0].position_m = {0.1, 0.2, 0.3};
    store.data().points[1].position_m = {0.105, 0.2, 0.3};
    store.data().points[0].valid = true;
    store.data().points[1].valid = true;
    EXPECT_EQ(1, store.mergeDuplicates(0.01));
    EXPECT_EQ(1, store.validPoints());
}

TEST(Pipeline, ReconstructsTheSyntheticTurntableScan) {
    const syn::SfmScan scan = syn::makeLatticeScan(16, 0.5);
    ASSERT_EQ(16u, scan.frames.size());

    SfmOptions options;
    options.two_view.threshold_px = 2.0;
    options.pnp.threshold_px = 3.0;
    options.bundle_adjustment.max_iterations = 30;
    options.keyframe.min_overlap = 0.2;
    options.outlier_reprojection_px = 4.0;

    const auto result = scand3d::sfm::reconstruct(scan.frames, scan.pairs, options);
    ASSERT_GE(result.registration_order.size(), 2u);

    EXPECT_GE(result.report.views_registered, 15) << result.report.summary();
    EXPECT_LT(result.report.mean_reprojection_px, 1.0) << result.report.summary();
    EXPECT_GT(result.report.points, 40);

    const int seed = result.registration_order.front();
    for (const int view : result.registration_order) {
        if (view == seed) {
            continue;
        }
        EXPECT_LT(
            syn::relativeRotationErrorDeg(result.reconstruction, scan.ground_truth, seed, view),
            1.0)
            << "view " << view;
        EXPECT_LT(syn::relativeCentreDirectionErrorDeg(result.reconstruction, scan.ground_truth,
                                                       seed, view),
                  1.0)
            << "view " << view;
    }
}

TEST(Report, SummarizesWhatWasEstimated) {
    const syn::SfmScan scan = syn::makeLatticeScan(12, 0.3);
    SfmOptions options;
    options.bundle_adjustment.max_iterations = 10;
    const auto result = scand3d::sfm::reconstruct(scan.frames, scan.pairs, options);

    EXPECT_EQ(static_cast<int>(scan.frames.size()), result.report.views_total);

    int observations = 0;
    for (const auto& point : result.reconstruction.points) {
        if (point.valid) {
            observations += static_cast<int>(point.observations.size());
        }
    }
    EXPECT_EQ(observations, result.report.observations);
    EXPECT_EQ(result.report.views_registered,
              static_cast<int>(std::count_if(
                  result.reconstruction.views.begin(), result.reconstruction.views.end(),
                  [](const ViewState& view) { return view.registered; })));
    EXPECT_FALSE(result.report.summary().empty());
    EXPECT_NE(std::string::npos, result.report.summary().find("views"));
}
