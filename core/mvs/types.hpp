#pragma once

#include <cstdint>
#include <vector>

#include "feature/gray_image.hpp"
#include "geometry/linalg.hpp"
#include "sfm/types.hpp"

// Dense reconstruction (roadmap Phase 6). One module owns the dense depth estimation, the
// cross-view consistency filter and the fusion into a coloured point cloud. It takes poses and
// cameras that Phase 4 produced, so it never re-estimates geometry and depends only on
// core/feature (gray images), core/geometry and core/sfm types.
namespace scand3d::mvs {

using geometry::Mat3;
using geometry::Vec3;
using sfm::CameraModel;
using sfm::Pose;

/// Per-pixel depth and surface normal in the owning camera's frame. `depth_m` is 0 where the
/// pixel has no estimate; a normal is only meaningful where it does.
struct DepthMap {
    int width_px = 0;
    int height_px = 0;
    std::vector<float> depth_m;
    /// Three floats per pixel: unit normal in the camera frame (+z forward).
    std::vector<float> normal_camera;
    /// Matching cost, 1 - zero-mean normalised cross correlation; 2 means "no evidence".
    std::vector<float> cost;

    bool empty() const { return width_px <= 0 || height_px <= 0 || depth_m.empty(); }

    bool valid(int x_px, int y_px) const {
        if (x_px < 0 || y_px < 0 || x_px >= width_px || y_px >= height_px) {
            return false;
        }
        return depth_m[static_cast<std::size_t>(y_px) * width_px + x_px] > 0.0f;
    }

    double depthAt(int x_px, int y_px) const {
        if (!valid(x_px, y_px)) {
            return 0.0;
        }
        return depth_m[static_cast<std::size_t>(y_px) * width_px + x_px];
    }

    Vec3 normalAt(int x_px, int y_px) const {
        const std::size_t offset =
            (static_cast<std::size_t>(y_px) * width_px + x_px) * 3u;
        return {normal_camera[offset], normal_camera[offset + 1], normal_camera[offset + 2]};
    }

    /// Allocates storage for a map of this size, every pixel invalid.
    static DepthMap sized(int width_px, int height_px);
};

/// One view's dense result: the depth map plus what is needed to place it in the world.
struct DepthView {
    DepthMap depth;
    CameraModel camera;
    Pose pose;
    feature::GrayImage gray;
};

/// A fused world-space point with the gray value averaged from the views that saw it.
struct FusedPoint {
    Vec3 position_m{};
    float gray = 1.0f;
    int observations = 0;
};

/// Intrinsics scaled by a factor, for estimating at a reduced resolution (roadmap 6.4).
CameraModel scaledCamera(const CameraModel& camera, double factor);

}  // namespace scand3d::mvs
