#pragma once

#include <vector>

#include "sfm/types.hpp"

namespace scand3d::sfm {

/// One view of a candidate point, for triangulation.
struct ViewObservation {
    CameraModel camera;
    Pose pose;
    Vec2 pixel{};
};

struct TriangulationResult {
    Vec3 position_m{};
    /// Largest reprojection error over the observations, in pixels.
    double max_reprojection_px = 0.0;
    /// Smallest camera depth at which the point was seen; negative means a camera saw it behind.
    double min_depth_m = 0.0;
    /// Largest angle between any two viewing rays, in radians. Small values mean the point is
    /// poorly conditioned no matter how small the reprojection error looks.
    double max_parallax_rad = 0.0;
    bool valid = false;
};

/// Triangulates from two or more views: linear DLT in calibrated coordinates, then Gauss-Newton
/// on the reprojection error. Rejects points behind any camera or below `min_parallax_rad`
/// (roadmap 4.1 and 4.2).
bool triangulatePoint(
    const std::vector<ViewObservation>& observations,
    TriangulationResult& result,
    double min_parallax_rad = 0.017453292519943295);  // 1 degree

/// Two-view convenience wrapper.
bool triangulateTwoView(
    const CameraModel& first_camera,
    const Pose& first_pose,
    const Vec2& first_pixel,
    const CameraModel& second_camera,
    const Pose& second_pose,
    const Vec2& second_pixel,
    TriangulationResult& result,
    double min_parallax_rad = 0.017453292519943295);

}  // namespace scand3d::sfm
