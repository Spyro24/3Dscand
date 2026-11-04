#pragma once

#include <atomic>

#include "feature/gray_image.hpp"
#include "geometry/linalg.hpp"
#include "mvs/types.hpp"

namespace scand3d::mvs {

/// PatchMatch stereo for one reference view against one neighbour (roadmap 6.1). The search is
/// over a plane per pixel (depth plus normal), initialised by a log-spaced plane sweep and refined
/// by spatial propagation and random search.
struct PatchMatchOptions {
    /// Window radius of the correlation patch: 2 means 5x5.
    int patch_radius_px = 2;
    /// Propagation/refinement sweeps over the image.
    int iterations = 4;
    /// Depths sampled by the initial plane sweep; 0 skips the sweep and starts from random planes.
    int depth_samples = 32;
    /// Random plane trials per pixel and iteration.
    int random_trials = 6;
    double min_depth_m = 0.1;
    double max_depth_m = 10.0;
    /// Spread of the initial random normals around the viewing ray, in degrees.
    double initial_normal_deg = 35.0;
    /// A pixel whose best cost is above this keeps no depth: 1 - ZNCC, so 0.5 means ZNCC 0.5.
    double max_cost = 0.5;

    /// Sobel magnitude (0..255) the reference pixel needs before its depth is kept (D053). On a
    /// flat patch the photometric cost matches anything, so the depth there is noise whatever the
    /// cost says; this drops those pixels instead of fusing blobs. Zero disables the check.
    int min_reference_gradient = 4;
};

/// Relative transform from the reference camera to the neighbour: X_neighbour = R * X_ref + t.
struct RelativePose {
    geometry::Mat3 rotation = geometry::identity3();
    Vec3 translation_m{};
};

/// Estimates a depth map for `reference`. Both images must be at the resolution their camera
/// models describe. Returns a map with every pixel invalid when the input is unusable; a cancelled
/// run returns the pixels estimated so far.
DepthMap estimateDepth(
    const feature::GrayImage& reference,
    const feature::GrayImage& neighbour,
    const CameraModel& reference_camera,
    const CameraModel& neighbour_camera,
    const RelativePose& relative,
    const PatchMatchOptions& options = {},
    const std::atomic<bool>* cancel = nullptr);

/// Relative pose between two camera-to-world poses.
RelativePose relativePose(const Pose& reference, const Pose& neighbour);

}  // namespace scand3d::mvs
