#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include "feature/gray_image.hpp"
#include "geometry/linalg.hpp"
#include "mvs/consistency.hpp"
#include "mvs/fusion.hpp"
#include "mvs/patchmatch.hpp"
#include "mvs/pipeline.hpp"
#include "mvs/types.hpp"
#include "sfm/types.hpp"
#include "synthetic/sfm_scan.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "tools/chamfer.hpp"

namespace {

namespace syn = scand3d::synthetic;
namespace mvs = scand3d::mvs;
namespace geometry = scand3d::geometry;
namespace feature = scand3d::feature;

constexpr int kSceneWidthPx = 480;
constexpr int kSceneHeightPx = 360;
constexpr double kRingRadiusM = 1.5;
constexpr double kRingHeightM = 0.6;
constexpr double kObjectCentreHeightM = 0.6;
constexpr double kPi = 3.14159265358979323846;
constexpr int kTurntableViews = 16;
constexpr std::uint32_t kTextureSeed = 7u;

feature::GrayImage grayOf(const syn::Image& image) {
    feature::GrayImage gray;
    gray.width_px = image.width_px;
    gray.height_px = image.height_px;
    gray.pixels = image.pixels;
    return gray;
}

mvs::MvsViewInput inputOf(const syn::Scene& scene, int index) {
    mvs::MvsViewInput input;
    input.gray = grayOf(syn::render(scene, index));
    input.camera = syn::cameraModelOf(scene.cameras[static_cast<std::size_t>(index)]);
    input.pose = syn::poseOf(scene.cameras[static_cast<std::size_t>(index)]);
    return input;
}

/// Turntable ring with a 22.5 degree step. Neighbouring cameras have to share most of the visible
/// surface for a stereo pair to match at all: a sparse ring (three views 120 degrees apart) sees
/// almost nothing twice and no pair can recover depth.
syn::Scene turntableScene(int view_count = kTurntableViews, int width_px = kSceneWidthPx) {
    syn::Scene scene = syn::makeNoiseTexturedBoxScene(
        view_count, kRingRadiusM, kRingHeightM, width_px, width_px * 3 / 4,
        0.75 * width_px, kTextureSeed);
    for (int index = 0; index < view_count; ++index) {
        const double angle = 2.0 * kPi * index / view_count;
        scene.cameras[static_cast<std::size_t>(index)].pose_camera_to_world = syn::lookAtCameraToWorld(
            {kRingRadiusM * std::cos(angle), kRingRadiusM * std::sin(angle), kRingHeightM},
            {0.0, 0.0, kObjectCentreHeightM}, {0.0, 0.0, 1.0});
    }
    return scene;
}

std::vector<scand3d::sfm::Pose> cameraPoses(const syn::Scene& scene) {
    std::vector<scand3d::sfm::Pose> poses;
    for (const syn::Camera& camera : scene.cameras) {
        poses.push_back(syn::poseOf(camera));
    }
    return poses;
}

std::vector<geometry::Vec3> worldPoints(const mvs::DepthView& view) {
    std::vector<geometry::Vec3> points;
    for (int y = 0; y < view.depth.height_px; ++y) {
        for (int x = 0; x < view.depth.width_px; ++x) {
            const double depth = view.depth.depthAt(x, y);
            if (!(depth > 0.0)) {
                continue;
            }
            const geometry::Vec3 point_camera{
                (x - view.camera.cx_px) / view.camera.focal_px * depth,
                (y - view.camera.cy_px) / view.camera.focal_px * depth,
                depth};
            points.push_back(scand3d::sfm::cameraToWorld(view.pose, point_camera));
        }
    }
    return points;
}

mvs::DepthMap planeDepthMap(int width_px, int height_px, double depth_m, int from_x, int to_x) {
    mvs::DepthMap map = mvs::DepthMap::sized(width_px, height_px);
    for (int y = 0; y < height_px; ++y) {
        for (int x = 0; x < width_px; ++x) {
            if (x < from_x || x >= to_x) {
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(y) * width_px + x;
            map.depth_m[index] = static_cast<float>(depth_m);
            map.normal_camera[index * 3 + 2] = -1.0f;
            map.cost[index] = 0.1f;
        }
    }
    return map;
}

struct PlaneRig {
    mvs::CameraModel camera{300.0, 50.0, 50.0};
    mvs::Pose first;
    mvs::Pose second;
};

PlaneRig planeRig() {
    PlaneRig rig;
    rig.first.rotation = geometry::identity3();
    rig.first.camera_centre_m = {0.0, 0.0, 0.0};
    rig.second.rotation = geometry::identity3();
    rig.second.camera_centre_m = {0.1, 0.0, 0.0};
    return rig;
}

}  // namespace

TEST(DepthMapTest, ASizedMapHasNoValidPixel) {
    const mvs::DepthMap map = mvs::DepthMap::sized(8, 6);
    EXPECT_EQ(map.width_px, 8);
    EXPECT_EQ(map.height_px, 6);
    EXPECT_EQ(map.depth_m.size(), 48u);
    EXPECT_EQ(map.normal_camera.size(), 144u);
    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 8; ++x) {
            EXPECT_FALSE(map.valid(x, y));
        }
    }
}

TEST(PatchMatchTest, RecoversTheSurfaceOfTheTurntableBox) {
    // A smaller render keeps the test near twenty seconds; the geometry is unchanged.
    const syn::Scene scene = turntableScene(kTurntableViews, 240);
    const std::vector<mvs::MvsViewInput> views = {inputOf(scene, 0), inputOf(scene, 1)};

    mvs::PatchMatchOptions options;
    options.iterations = 4;
    options.min_depth_m = 0.6;
    options.max_depth_m = 2.0;
    const mvs::DepthMap depth = mvs::estimateDepth(
        views[0].gray, views[1].gray, views[0].camera, views[1].camera,
        mvs::relativePose(views[0].pose, views[1].pose), options);

    int valid = 0;
    for (int y = 0; y < depth.height_px; ++y) {
        for (int x = 0; x < depth.width_px; ++x) {
            valid += depth.valid(x, y) ? 1 : 0;
        }
    }
    const int pixels = depth.width_px * depth.height_px;
    ASSERT_GT(valid, pixels / 20) << "fewer than 5 percent of the pixels got a depth";

    mvs::DepthView view;
    view.depth = depth;
    view.camera = views[0].camera;
    view.pose = views[0].pose;
    view.gray = views[0].gray;
    const std::vector<geometry::Vec3> points = worldPoints(view);
    const auto chamfer = scand3d::testutil::chamferToMesh(points, scene.mesh, 200);
    EXPECT_LT(chamfer.accuracy_m, 0.02) << "mean distance of the depth map to the box";
}

TEST(PatchMatchTest, AFlatImageYieldsNoDepth) {
    feature::GrayImage flat;
    flat.width_px = 160;
    flat.height_px = 120;
    flat.pixels.assign(static_cast<std::size_t>(160) * 120, 128);

    mvs::CameraModel camera{120.0, 80.0, 60.0};
    mvs::Pose left;
    mvs::Pose right;
    right.camera_centre_m = {0.1, 0.0, 0.0};

    mvs::PatchMatchOptions options;
    options.min_depth_m = 0.2;
    options.max_depth_m = 3.0;
    const mvs::DepthMap depth = mvs::estimateDepth(
        flat, flat, camera, camera, mvs::relativePose(left, right), options);

    int valid = 0;
    for (int y = 0; y < depth.height_px; ++y) {
        for (int x = 0; x < depth.width_px; ++x) {
            valid += depth.valid(x, y) ? 1 : 0;
        }
    }
    EXPECT_EQ(valid, 0) << "a textureless patch is not evidence of any depth";
}

TEST(ConsistencyTest, KeepsThePlaneBothViewsAgreeOnAndDropsTheRest) {
    const PlaneRig rig = planeRig();
    mvs::DepthMap reference = planeDepthMap(100, 100, 1.0, 0, 100);
    mvs::DepthMap neighbour = planeDepthMap(100, 100, 1.0, 0, 100);
    // Half of the reference claims a surface 20 percent closer than the neighbour sees it.
    mvs::DepthMap corrupted = planeDepthMap(100, 100, 1.2, 0, 50);
    for (int y = 0; y < 100; ++y) {
        for (int x = 50; x < 100; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * 100 + x;
            corrupted.depth_m[index] = reference.depth_m[index];
        }
    }

    const mvs::ConsistencyReport report = mvs::filterByConsistency(
        corrupted, neighbour, rig.camera, rig.camera, rig.first, rig.second);
    EXPECT_GT(report.checked, 9000);
    EXPECT_GT(report.kept, 4000);
    EXPECT_LT(report.kept, 5200) << "the 1.2 m half has to be rejected";

    int valid_left = 0;
    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 50; ++x) {
            valid_left += corrupted.valid(x, y) ? 1 : 0;
        }
    }
    EXPECT_EQ(valid_left, 0);
}

TEST(FusionTest, AveragesTwoDepthMapsIntoOnePlane) {
    const PlaneRig rig = planeRig();
    mvs::DepthView first;
    first.depth = planeDepthMap(100, 100, 1.0, 0, 100);
    first.camera = rig.camera;
    first.pose = rig.first;
    first.gray.width_px = 100;
    first.gray.height_px = 100;
    first.gray.pixels.assign(10000, 200);

    mvs::DepthView second = first;
    second.pose = rig.second;

    mvs::FusionOptions options;
    options.voxel_m = 0.005;
    const mvs::FusionResult fused = mvs::fuseDepths({first, second}, options);

    ASSERT_GT(fused.fused_points, 2000);
    for (const mvs::FusedPoint& point : fused.points) {
        EXPECT_NEAR(point.position_m[2], 1.0, 0.01);
        EXPECT_NEAR(point.gray, 200.0 / 255.0, 1e-3);
        EXPECT_GE(point.observations, 1);
    }

    mvs::FusionOptions strict = options;
    strict.min_observations = 2;
    const mvs::FusionResult both_views = mvs::fuseDepths({first, second}, strict);
    EXPECT_GT(both_views.fused_points, 0);
    EXPECT_LE(both_views.fused_points, fused.fused_points);
}

TEST(MvsPipelineTest, FusesTheTurntableIntoAPointCloud) {
    const syn::Scene scene = turntableScene();
    std::vector<mvs::MvsViewInput> views;
    for (int index = 0; index < kTurntableViews; ++index) {
        views.push_back(inputOf(scene, index));
    }

    mvs::MvsOptions options;
    options.working_width_px = 160;
    options.patch_match.min_depth_m = 0.6;
    options.patch_match.max_depth_m = 2.0;
    options.fusion.voxel_m = 0.004;

    const mvs::MvsResult result = mvs::reconstructDense(views, options);
    ASSERT_FALSE(result.cancelled);
    EXPECT_EQ(result.stats.processed_views, kTurntableViews);
    ASSERT_GT(result.stats.fused_points, 1000);

    std::vector<geometry::Vec3> points;
    points.reserve(result.points.size());
    for (const mvs::FusedPoint& point : result.points) {
        points.push_back(point.position_m);
    }
    // Completeness is measured over the sides only: a ring at the object centre never sees the top
    // or the underside, and counting them would score the capture geometry instead.
    const auto chamfer =
        scand3d::testutil::chamferToMesh(points, scene.mesh, cameraPoses(scene));
    EXPECT_NEAR(chamfer.measured_area_m2, 1.2, 0.05) << "four box sides, not the caps";
    EXPECT_LT(chamfer.accuracy_m, 0.02) << "fused points to the ground-truth box";
    EXPECT_LT(chamfer.completeness_m, 0.035) << "ground-truth surface to the fused cloud";
}

TEST(MvsPipelineTest, CancellingStopsBeforeFusionAndReportsProgress) {
    const syn::Scene scene = turntableScene();
    std::vector<mvs::MvsViewInput> views;
    for (int index = 0; index < kTurntableViews; ++index) {
        views.push_back(inputOf(scene, index));
    }

    mvs::MvsOptions options;
    options.working_width_px = 160;
    options.patch_match.min_depth_m = 0.6;
    options.patch_match.max_depth_m = 2.0;

    std::atomic<bool> cancel{false};
    int progress_calls = 0;
    mvs::MvsControl control;
    control.cancel = &cancel;
    control.progress = [&cancel, &progress_calls](double fraction) {
        ++progress_calls;
        EXPECT_GE(fraction, 0.0);
        EXPECT_LE(fraction, 1.0);
        if (fraction > 0.0) {
            cancel.store(true);
        }
    };

    const mvs::MvsResult result = mvs::reconstructDense(views, options, control);
    EXPECT_TRUE(result.cancelled);
    EXPECT_GE(progress_calls, 1);
    // Views run on a pool, so a few may finish before the cancel is seen; most must not.
    EXPECT_LT(result.stats.processed_views, kTurntableViews);
    EXPECT_TRUE(result.points.empty()) << "a cancelled run must not publish a partial cloud";
}

TEST(MvsPipelineTest, HigherQualityEstimatesMorePixels) {
    // Two neighbouring views are enough to show that the preset changes the working resolution.
    const syn::Scene scene = turntableScene();
    const std::vector<mvs::MvsViewInput> views = {inputOf(scene, 0), inputOf(scene, 1)};

    mvs::MvsOptions draft;
    draft.working_width_px = mvs::workingWidthPx(mvs::MvsQuality::Draft);
    draft.patch_match.min_depth_m = 0.6;
    draft.patch_match.max_depth_m = 2.0;

    mvs::MvsOptions high = draft;
    high.working_width_px = 240;

    const mvs::MvsResult draft_result = mvs::reconstructDense(views, draft);
    const mvs::MvsResult high_result = mvs::reconstructDense(views, high);

    EXPECT_EQ(draft_result.views[0].depth.width_px, 160);
    EXPECT_EQ(high_result.views[0].depth.width_px, 240);
    EXPECT_GT(high_result.stats.depth_pixels, 2 * draft_result.stats.depth_pixels);
}

TEST(MvsPipelineTest, SparseHintsNarrowTheDepthRange) {
    const syn::Scene scene = turntableScene();
    const std::vector<mvs::MvsViewInput> views = {inputOf(scene, 0), inputOf(scene, 1)};

    std::vector<geometry::Vec3> hints;
    for (const syn::Vertex& vertex : scene.mesh.vertices) {
        hints.push_back(vertex.position_m);
    }

    mvs::MvsOptions options;
    options.working_width_px = 200;
    options.patch_match.min_depth_m = 0.05;
    options.patch_match.max_depth_m = 20.0;
    options.depth_hint_points_m = hints;
    options.fusion.voxel_m = 0.004;

    const mvs::MvsResult result = mvs::reconstructDense(views, options);
    ASSERT_FALSE(result.cancelled);
    EXPECT_GT(result.stats.fused_points, 500) << "a hint range has to keep the sweep on the object";
}
