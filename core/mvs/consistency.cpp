#include "mvs/consistency.hpp"

#include <cmath>

#include "sfm/types.hpp"

namespace scand3d::mvs {
namespace {

Vec3 backProject(
    const CameraModel& camera,
    const Pose& pose,
    int x_px,
    int y_px,
    double depth_m) {
    const Vec3 point_camera{
        (static_cast<double>(x_px) - camera.cx_px) / camera.focal_px * depth_m,
        (static_cast<double>(y_px) - camera.cy_px) / camera.focal_px * depth_m,
        depth_m};
    return sfm::cameraToWorld(pose, point_camera);
}

void invalidate(DepthMap& map, int x_px, int y_px) {
    const std::size_t index = static_cast<std::size_t>(y_px) * map.width_px + x_px;
    map.depth_m[index] = 0.0f;
    map.cost[index] = 2.0f;
}

}  // namespace

ConsistencyReport filterByConsistency(
    DepthMap& reference,
    const DepthMap& neighbour,
    const CameraModel& reference_camera,
    const CameraModel& neighbour_camera,
    const Pose& reference_pose,
    const Pose& neighbour_pose,
    const ConsistencyOptions& options) {
    ConsistencyReport report;
    if (reference.empty() || neighbour.empty()) {
        return report;
    }
    if (!(reference_camera.focal_px > 0.0) || !(neighbour_camera.focal_px > 0.0)) {
        return report;
    }
    if (!(options.depth_tolerance_ratio > 0.0) || !(options.round_trip_px > 0.0)) {
        return report;
    }

    for (int y = 0; y < reference.height_px; ++y) {
        for (int x = 0; x < reference.width_px; ++x) {
            if (!reference.valid(x, y)) {
                continue;
            }
            ++report.checked;
            const double depth = reference.depthAt(x, y);
            const Vec3 point_world = backProject(reference_camera, reference_pose, x, y, depth);

            geometry::Vec2 neighbour_pixel{};
            double expected_depth = 0.0;
            if (!sfm::projectPoint(
                    neighbour_camera, neighbour_pose, point_world, neighbour_pixel,
                    expected_depth)) {
                invalidate(reference, x, y);
                continue;
            }
            const int neighbour_x = static_cast<int>(std::lround(neighbour_pixel[0]));
            const int neighbour_y = static_cast<int>(std::lround(neighbour_pixel[1]));
            if (!neighbour.valid(neighbour_x, neighbour_y)) {
                invalidate(reference, x, y);
                continue;
            }
            const double neighbour_depth = neighbour.depthAt(neighbour_x, neighbour_y);
            if (!(std::abs(neighbour_depth - expected_depth) <=
                  options.depth_tolerance_ratio * expected_depth)) {
                invalidate(reference, x, y);
                continue;
            }

            // The neighbour's own point has to come back to this pixel; that rejects a pair of
            // maps agreeing on a wrong plane, which a one-way check cannot see.
            const Vec3 neighbour_world = backProject(
                neighbour_camera, neighbour_pose, neighbour_x, neighbour_y, neighbour_depth);
            geometry::Vec2 back_pixel{};
            double back_depth = 0.0;
            if (!sfm::projectPoint(
                    reference_camera, reference_pose, neighbour_world, back_pixel, back_depth)) {
                invalidate(reference, x, y);
                continue;
            }
            const double dx = back_pixel[0] - x;
            const double dy = back_pixel[1] - y;
            if (!(std::sqrt(dx * dx + dy * dy) <= options.round_trip_px)) {
                invalidate(reference, x, y);
                continue;
            }
            ++report.kept;
        }
    }
    return report;
}

}  // namespace scand3d::mvs
