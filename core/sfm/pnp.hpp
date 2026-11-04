#pragma once

#include <cstdint>
#include <vector>

#include "sfm/types.hpp"

namespace scand3d::sfm {

struct PnpOptions {
    /// Inlier threshold on the reprojection error, in pixels. 3 px on 480 px dark phone frames
    /// refused views the map could have used; 5 px registers more of a real scan and tracks more
    /// live frames (D056).
    double threshold_px = 5.0;
    int max_iterations = 500;
    double confidence = 0.999;
    std::uint32_t seed = 17;
};

struct PnpResult {
    bool valid = false;
    Pose pose;
    std::vector<int> inlier_indices;
    double median_reprojection_px = 0.0;
    int inliers = 0;
};

/// Pose from at least six 3D/2D correspondences: linear DLT, then Gauss-Newton on reprojection.
bool solvePoseDlt(
    const std::vector<Vec3>& object_points,
    const std::vector<Vec2>& image_points,
    const CameraModel& camera,
    Pose& pose);

/// PnP inside RANSAC with a six-point minimal sample, refit on the inliers (roadmap 4.2).
PnpResult estimatePoseRansac(
    const std::vector<Vec3>& object_points,
    const std::vector<Vec2>& image_points,
    const CameraModel& camera,
    const PnpOptions& options = {});

}  // namespace scand3d::sfm
