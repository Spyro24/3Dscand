#pragma once

#include "sfm/types.hpp"

namespace scand3d::sfm {

struct BundleAdjustmentOptions {
    int max_iterations = 60;
    double initial_damping = 1e-4;
    double cost_tolerance = 1e-9;
    /// Huber threshold on the pixel residual; 0 disables the robust loss.
    double huber_px = 2.0;
    /// Camera 0 stays fixed, which removes the gauge freedom except for the global scale; the
    /// damping keeps that direction from drifting (see DECISIONS).
    bool fix_first_camera = true;
};

struct BundleAdjustmentResult {
    int iterations = 0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    int observations = 0;
    int points = 0;
    int cameras = 0;
};

/// Levenberg-Marquardt over camera poses and point positions, with the Schur complement on the
/// point blocks so only the 6N x 6N camera system is ever factorised (roadmap 4.4).
/// Poses are updated as R <- R * Exp(delta) and C <- C + delta.
BundleAdjustmentResult optimize(
    Reconstruction& reconstruction,
    const BundleAdjustmentOptions& options = {});

/// Robustified squared reprojection cost of the whole reconstruction, in px^2.
double reprojectionCost(const Reconstruction& reconstruction, double huber_px = 0.0);

/// Mean and median reprojection error over every observation of every valid point, in pixels.
double meanReprojectionPx(const Reconstruction& reconstruction);
double medianReprojectionPx(const Reconstruction& reconstruction);

}  // namespace scand3d::sfm
