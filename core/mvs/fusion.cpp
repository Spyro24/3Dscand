#include "mvs/fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

#include "sfm/types.hpp"

namespace scand3d::mvs {
namespace {

struct Accumulator {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_gray = 0.0;
    int count = 0;
};

/// Packs a voxel coordinate into one integer; the 21-bit per-axis window is far larger than any
/// scan this module can produce and keeps the key exact for negative coordinates.
std::int64_t voxelKey(int x, int y, int z) {
    return (static_cast<std::int64_t>(x) & 0x1FFFFF) << 42 |
           (static_cast<std::int64_t>(y) & 0x1FFFFF) << 21 |
           (static_cast<std::int64_t>(z) & 0x1FFFFF);
}

}  // namespace

FusionResult fuseDepths(const std::vector<DepthView>& views, const FusionOptions& options) {
    FusionResult result;
    if (views.empty()) {
        return result;
    }
    const double voxel = options.voxel_m > 1e-6 ? options.voxel_m : 1e-3;

    std::unordered_map<std::int64_t, Accumulator> voxels;
    for (const DepthView& view : views) {
        if (view.depth.empty() || !(view.camera.focal_px > 0.0)) {
            continue;
        }
        for (int y = 0; y < view.depth.height_px; ++y) {
            for (int x = 0; x < view.depth.width_px; ++x) {
                const double depth = view.depth.depthAt(x, y);
                if (!(depth > 0.0)) {
                    continue;
                }
                ++result.depth_pixels;
                const Vec3 point_camera{
                    (static_cast<double>(x) - view.camera.cx_px) / view.camera.focal_px * depth,
                    (static_cast<double>(y) - view.camera.cy_px) / view.camera.focal_px * depth,
                    depth};
                const Vec3 point_world = sfm::cameraToWorld(view.pose, point_camera);
                if (!std::isfinite(point_world[0]) || !std::isfinite(point_world[1]) ||
                    !std::isfinite(point_world[2])) {
                    continue;
                }
                const int cell_x = static_cast<int>(std::floor(point_world[0] / voxel));
                const int cell_y = static_cast<int>(std::floor(point_world[1] / voxel));
                const int cell_z = static_cast<int>(std::floor(point_world[2] / voxel));
                Accumulator& accumulator = voxels[voxelKey(cell_x, cell_y, cell_z)];
                accumulator.sum_x += point_world[0];
                accumulator.sum_y += point_world[1];
                accumulator.sum_z += point_world[2];
                accumulator.sum_gray +=
                    view.gray.empty() ? 1.0 : static_cast<double>(view.gray.at(x, y)) / 255.0;
                ++accumulator.count;
            }
        }
    }

    // Sorted by voxel key so the same input produces the same point order every run.
    std::vector<std::pair<std::int64_t, Accumulator>> ordered(voxels.begin(), voxels.end());
    std::sort(
        ordered.begin(), ordered.end(),
        [](const auto& first, const auto& second) { return first.first < second.first; });

    const int minimum = std::max(1, options.min_observations);
    result.points.reserve(ordered.size());
    for (const auto& entry : ordered) {
        const Accumulator& accumulator = entry.second;
        if (accumulator.count <= 0 || accumulator.count < minimum) {
            continue;
        }
        const double inverse_count = 1.0 / accumulator.count;
        FusedPoint point;
        point.position_m = {
            accumulator.sum_x * inverse_count,
            accumulator.sum_y * inverse_count,
            accumulator.sum_z * inverse_count};
        point.gray = static_cast<float>(accumulator.sum_gray * inverse_count);
        point.observations = accumulator.count;
        result.points.push_back(point);
    }
    result.fused_points = static_cast<int>(result.points.size());
    return result;
}

}  // namespace scand3d::mvs
