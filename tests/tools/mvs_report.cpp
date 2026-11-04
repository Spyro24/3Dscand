#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "mvs/pipeline.hpp"
#include "sfm/types.hpp"
#include "synthetic/sfm_scan.hpp"
#include "synthetic/synthetic_scene.hpp"
#include "tools/chamfer.hpp"

// Desktop-only Phase 6 report: runs the dense stage on the synthetic turntable, prints the Chamfer
// distance to the ground-truth mesh and the timing the acceptance line is judged on. Built only in
// `nativeTest` and never packaged into the APK.
int main(int argc, char** argv) {
    const int view_count = argc > 1 ? std::atoi(argv[1]) : 8;
    const int working_width = argc > 2 ? std::atoi(argv[2]) : scand3d::mvs::workingWidthPx(
                                                           scand3d::mvs::MvsQuality::Normal);
    const double ring_radius_m = argc > 3 ? std::atof(argv[3]) : 1.5;
    if (view_count < 2) {
        std::fprintf(stderr, "view count must be at least 2\n");
        return 2;
    }

    const int width_px = 480;
    const int height_px = 360;
    const scand3d::synthetic::Scene scene = scand3d::synthetic::makeNoiseTexturedBoxScene(
        view_count, ring_radius_m, 0.6, width_px, height_px, 0.75 * width_px, 7u);

    std::vector<scand3d::mvs::MvsViewInput> views;
    views.reserve(static_cast<std::size_t>(view_count));
    for (int index = 0; index < view_count; ++index) {
        const auto& camera = scene.cameras[static_cast<std::size_t>(index)];
        const scand3d::synthetic::Image image = scand3d::synthetic::render(scene, index);
        scand3d::mvs::MvsViewInput input;
        input.gray.width_px = image.width_px;
        input.gray.height_px = image.height_px;
        input.gray.pixels = image.pixels;
        input.camera = scand3d::synthetic::cameraModelOf(camera);
        input.pose = scand3d::synthetic::poseOf(camera);
        views.push_back(input);
    }

    // Optional fourth argument: the gradient floor the depth pass keeps (D053). Passing 0 turns it
    // off, which is how its effect on the Chamfer distance is measured.
    const int min_gradient =
        argc > 4 ? std::atoi(argv[4]) : scand3d::mvs::PatchMatchOptions{}.min_reference_gradient;

    scand3d::mvs::MvsOptions options;
    options.working_width_px = working_width;
    options.patch_match.min_reference_gradient = min_gradient;
    options.patch_match.min_depth_m = 0.6;
    options.patch_match.max_depth_m = 2.0;
    options.fusion.voxel_m = 0.004;

    std::vector<scand3d::geometry::Vec3> hints;
    for (const auto& vertex : scene.mesh.vertices) {
        hints.push_back(vertex.position_m);
    }
    options.depth_hint_points_m = hints;

    int progress_samples = 0;
    double last_progress = 0.0;
    scand3d::mvs::MvsControl control;
    control.progress = [&progress_samples, &last_progress](double fraction) {
        ++progress_samples;
        last_progress = fraction;
    };

    const auto started = std::chrono::steady_clock::now();
    const scand3d::mvs::MvsResult result =
        scand3d::mvs::reconstructDense(views, options, control);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();

    std::vector<scand3d::geometry::Vec3> points;
    points.reserve(result.points.size());
    for (const auto& point : result.points) {
        points.push_back(point.position_m);
    }
    std::vector<scand3d::sfm::Pose> camera_poses;
    for (const auto& camera : scene.cameras) {
        camera_poses.push_back(scand3d::synthetic::poseOf(camera));
    }
    // Completeness only over the sides some camera sees: the ring sits at the object centre, so the
    // top and underside are never captured and would otherwise dominate the number.
    const auto chamfer = scand3d::testutil::chamferToMesh(points, scene.mesh, camera_poses);
    const int depth_map_pixels = result.views.empty()
                                     ? 0
                                     : result.views[0].depth.width_px * result.views[0].depth.height_px;

    std::printf("views             %d\n", view_count);
    std::printf("working width     %d px (%d px per depth map)\n", working_width, depth_map_pixels);
    std::printf("processed         %d\n", result.stats.processed_views);
    std::printf("depth pixels      %d\n", result.stats.depth_pixels);
    std::printf("consistent        %d\n", result.stats.consistent_pixels);
    std::printf("fused points      %d\n", result.stats.fused_points);
    std::printf("chamfer accuracy  %.4f m\n", chamfer.accuracy_m);
    std::printf("chamfer complete  %.4f m\n", chamfer.completeness_m);
    std::printf("chamfer symmetric %.4f m\n", chamfer.symmetric_m);
    std::printf("scored area       %.3f m2\n", chamfer.measured_area_m2);
    std::printf("elapsed ms        %.1f (%.1f per view)\n", elapsed_ms,
                elapsed_ms / view_count);
    std::printf("ms per megapixel  %.1f\n",
                depth_map_pixels > 0
                    ? elapsed_ms / (depth_map_pixels * view_count / 1e6)
                    : 0.0);
    std::printf("progress calls    %d, last %.3f\n", progress_samples, last_progress);
    std::printf("cancelled         %s\n", result.cancelled ? "yes" : "no");

    return result.points.empty() ? 1 : 0;
}
