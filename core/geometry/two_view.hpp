#pragma once

#include <cstdint>
#include <vector>

#include "geometry/linalg.hpp"

namespace scand3d::geometry {

struct PointCorrespondence {
    Vec2 first;   // pixels in the first image
    Vec2 second;  // pixels in the second image
};

struct CameraIntrinsics {
    double focal_px = 0.0;
    double cx_px = 0.0;
    double cy_px = 0.0;
};

struct RansacOptions {
    /// Inlier threshold on the Sampson distance, in pixels.
    double threshold_px = 1.5;
    int max_iterations = 2000;
    double confidence = 0.999;
    std::uint32_t seed = 7;
};

struct TwoViewResult {
    Mat3 matrix{};  // fundamental (pixels) or essential (calibrated) matrix, normalized
    std::vector<int> inlier_indices;
    double median_sampson_px = 0.0;
    bool valid = false;
};

/// Eight-point algorithm (Hartley-normalized) inside RANSAC, rank-2 enforced, refit on the
/// inliers (roadmap 3.4).
TwoViewResult estimateFundamentalRansac(
    const std::vector<PointCorrespondence>& correspondences,
    const RansacOptions& options = {});

/// The same estimator on calibrated coordinates, with the essential matrix constrained to two
/// equal singular values. Inlier indices refer to the input vector.
TwoViewResult estimateEssentialRansac(
    const std::vector<PointCorrespondence>& correspondences,
    const CameraIntrinsics& first,
    const CameraIntrinsics& second,
    const RansacOptions& options = {});

/// E = K2^T F K1, normalized. Useful when F was estimated in pixels.
Mat3 essentialFromFundamental(
    const Mat3& fundamental,
    const CameraIntrinsics& first,
    const CameraIntrinsics& second);

/// Sampson distance of one correspondence in pixels; the RANSAC inlier measure.
double sampsonDistancePx(const Mat3& matrix, const PointCorrespondence& correspondence);

}  // namespace scand3d::geometry
