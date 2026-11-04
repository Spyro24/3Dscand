#pragma once

#include <vector>

#include "geometry/two_view.hpp"
#include "sfm/types.hpp"

namespace scand3d::sfm {

/// Result of two-view initialisation (roadmap 4.1). `first` is the identity pose by definition;
/// `second` is expressed relative to it.
struct TwoViewInitialization {
    bool valid = false;
    Pose first{};
    Pose second{};
    std::vector<Vec3> points;
    /// Correspondence index that produced each entry of `points`.
    std::vector<int> source_index;
    double median_reprojection_px = 0.0;
    int inliers = 0;
};

/// Estimates the essential matrix, recovers the relative pose that puts the most points in front
/// of both cameras, and triangulates those points.
TwoViewInitialization initializeTwoView(
    const std::vector<geometry::PointCorrespondence>& correspondences,
    const CameraModel& first_camera,
    const CameraModel& second_camera,
    const geometry::RansacOptions& options = {});

}  // namespace scand3d::sfm
