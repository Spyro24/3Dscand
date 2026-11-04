#pragma once

#include <vector>

#include "sfm/pipeline.hpp"
#include "sfm/types.hpp"
#include "synthetic/synthetic_scene.hpp"

// Synthetic scan for the Phase 4 tests and the reconstruction report tool: a non-coplanar lattice
// of known 3D points observed by a turntable ring, with pixel noise and a match graph between
// neighbouring views. A monocular reconstruction only fixes the cameras up to a similarity, so the
// error helpers compare rotations and baseline directions instead of absolute positions.
namespace scand3d::synthetic {

struct SfmScan {
    std::vector<sfm::FrameInput> frames;
    std::vector<sfm::PairMatches> pairs;
    std::vector<sfm::Pose> ground_truth;
    std::vector<sfm::Vec3> points;
    /// feature_of_point[view][point] is the feature index of that point in the view, or -1.
    std::vector<std::vector<int>> feature_of_point;
};

/// Camera of the turntable ring at `angle_rad`, looking at the object centre.
Camera sfmCameraAt(double angle_rad, double radius_m = 1.6, double height_m = 0.5);

sfm::CameraModel cameraModelOf(const Camera& camera);

sfm::Pose poseOf(const Camera& camera);

/// 4 x 4 x 4 lattice in front of the ring: non-coplanar, so triangulation and PnP are well posed.
std::vector<sfm::Vec3> latticePoints();

SfmScan makeLatticeScan(int view_count, double noise_px);

/// Angle between two directions, in degrees.
double directionErrorDeg(const sfm::Vec3& first, const sfm::Vec3& second);

/// Gauge-free rotation error between view `index` and the seed view, in degrees.
double relativeRotationErrorDeg(
    const sfm::Reconstruction& reconstruction,
    const std::vector<sfm::Pose>& ground_truth,
    int seed,
    int index);

/// Gauge-free camera-centre direction error between view `index` and the seed view, in degrees.
double relativeCentreDirectionErrorDeg(
    const sfm::Reconstruction& reconstruction,
    const std::vector<sfm::Pose>& ground_truth,
    int seed,
    int index);

}  // namespace scand3d::synthetic
