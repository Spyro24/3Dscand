#include "sfm/pose.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "sfm/keyframe_selection.hpp"
#include "sfm/pnp.hpp"
#include "sfm/triangulation.hpp"

namespace scand3d::sfm {
namespace {

/// The four pose candidates an essential matrix allows: two rotations, then the translation sign.
struct PoseCandidate {
    Mat3 relative_rotation{};  // maps camera-1 coordinates to camera-2 coordinates
    Vec3 relative_translation{};  // in camera-1 coordinates
};

std::vector<PoseCandidate> decomposeEssential(const Mat3& essential) {
    Mat3 u{};
    Mat3 v{};
    std::array<double, 3> singular_values{};
    if (!geometry::svd3(essential, u, singular_values, v)) {
        return {};
    }

    // Guard against a U or V with determinant -1: W must stay a rotation. The third singular
    // direction has a zero singular value, so flipping those columns leaves E unchanged.
    auto fix = [](Mat3 matrix) {
        if (geometry::determinant3(matrix) < 0.0) {
            for (int row = 0; row < 3; ++row) {
                matrix[static_cast<std::size_t>(row * 3 + 2)] =
                    -matrix[static_cast<std::size_t>(row * 3 + 2)];
            }
        }
        return matrix;
    };
    u = fix(u);
    v = fix(v);

    const Mat3 w{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    const Mat3 wt = geometry::transpose3(w);
    const Vec3 tx{u[2], u[5], u[8]};

    std::vector<PoseCandidate> candidates;
    candidates.reserve(4);
    const Mat3 rotations[2] = {
        geometry::matmul3(geometry::matmul3(u, w), geometry::transpose3(v)),
        geometry::matmul3(geometry::matmul3(u, wt), geometry::transpose3(v)),
    };
    for (const Mat3& rotation : rotations) {
        candidates.push_back({rotation, tx});
        candidates.push_back({rotation, {-tx[0], -tx[1], -tx[2]}});
    }
    return candidates;
}

/// Converts `X2 = R X1 + t` into the second camera's world pose, with the first camera at the
/// world origin.
Pose worldPoseFromRelative(const Mat3& relative_rotation, const Vec3& relative_translation) {
    Pose pose;
    pose.rotation = geometry::transpose3(relative_rotation);
    pose.camera_centre_m = geometry::matvec3(pose.rotation,
                                             {-relative_translation[0],
                                              -relative_translation[1],
                                              -relative_translation[2]});
    return pose;
}

}  // namespace

TwoViewInitialization initializeTwoView(
    const std::vector<geometry::PointCorrespondence>& correspondences,
    const CameraModel& first_camera,
    const CameraModel& second_camera,
    const geometry::RansacOptions& options) {
    TwoViewInitialization initialization;
    if (correspondences.size() < 8) {
        return initialization;
    }

    const geometry::CameraIntrinsics first_intrinsics{
        first_camera.focal_px, first_camera.cx_px, first_camera.cy_px};
    const geometry::CameraIntrinsics second_intrinsics{
        second_camera.focal_px, second_camera.cx_px, second_camera.cy_px};
    const geometry::TwoViewResult essential = geometry::estimateEssentialRansac(
        correspondences, first_intrinsics, second_intrinsics, options);
    if (!essential.valid) {
        return initialization;
    }

    const std::vector<PoseCandidate> candidates = decomposeEssential(essential.matrix);
    if (candidates.empty()) {
        return initialization;
    }

    int best_in_front = -1;
    double best_median_px = 0.0;
    Pose best_second;
    std::vector<int> best_sources;
    std::vector<Vec3> best_points;

    for (const PoseCandidate& candidate : candidates) {
        const Pose second = worldPoseFromRelative(candidate.relative_rotation,
                                                 candidate.relative_translation);
        std::vector<int> sources;
        std::vector<Vec3> points;
        std::vector<double> errors;
        for (const int index : essential.inlier_indices) {
            const auto& correspondence = correspondences[static_cast<std::size_t>(index)];
            TriangulationResult triangulated;
            if (!triangulateTwoView(first_camera, initialization.first, correspondence.first,
                                    second_camera, second, correspondence.second, triangulated)) {
                continue;
            }
            sources.push_back(index);
            points.push_back(triangulated.position_m);
            errors.push_back(std::sqrt(squaredReprojectionPx(
                first_camera, initialization.first, triangulated.position_m,
                correspondence.first)));
            errors.push_back(std::sqrt(squaredReprojectionPx(
                second_camera, second, triangulated.position_m, correspondence.second)));
        }
        if (points.size() < 8) {
            continue;
        }
        std::sort(errors.begin(), errors.end());

        // Cheirality decides; the reprojection error only breaks ties. Selection between a mirror and
        // the true pair cannot be made here at all (both fit their own two views), so the pipeline
        // validates the seed against a third view instead of trusting this score.
        const double median_px = errors[errors.size() / 2];
        const bool better = best_in_front < 0 || points.size() > static_cast<std::size_t>(best_in_front) ||
                            (points.size() == static_cast<std::size_t>(best_in_front) &&
                             median_px < best_median_px);
        if (better) {
            best_in_front = static_cast<int>(points.size());
            best_median_px = median_px;
            best_second = second;
            best_sources = std::move(sources);
            best_points = std::move(points);
        }
    }

    if (best_in_front < 8) {
        return initialization;
    }

    // Polish the relative pose on its own inliers. The essential matrix fixes the translation
    // direction only to about a degree at these baselines, which is enough to mislead the seed.
    auto medianReprojectionPx = [&](const Pose& second, const std::vector<Vec3>& points,
                                    const std::vector<int>& sources) {
        std::vector<double> errors;
        errors.reserve(points.size() * 2);
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto& correspondence =
                correspondences[static_cast<std::size_t>(sources[i])];
            errors.push_back(std::sqrt(squaredReprojectionPx(
                first_camera, initialization.first, points[i], correspondence.first)));
            errors.push_back(std::sqrt(squaredReprojectionPx(second_camera, second, points[i],
                                                            correspondence.second)));
        }
        if (errors.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        std::sort(errors.begin(), errors.end());
        return errors[errors.size() / 2];
    };

    double best_error_px = medianReprojectionPx(best_second, best_points, best_sources);
    for (int round = 0; round < 3; ++round) {
        std::vector<Vec3> object_points;
        std::vector<Vec2> image_points;
        object_points.reserve(best_points.size());
        image_points.reserve(best_points.size());
        for (std::size_t i = 0; i < best_points.size(); ++i) {
            object_points.push_back(best_points[i]);
            image_points.push_back(
                correspondences[static_cast<std::size_t>(best_sources[i])].second);
        }

        PnpOptions pnp_options;
        pnp_options.threshold_px = 4.0;  // these points are already inliers; only precision matters
        pnp_options.max_iterations = 200;
        const PnpResult pnp =
            estimatePoseRansac(object_points, image_points, second_camera, pnp_options);
        if (!pnp.valid) {
            break;
        }

        std::vector<int> sources;
        std::vector<Vec3> points;
        for (const int index : essential.inlier_indices) {
            const auto& correspondence = correspondences[static_cast<std::size_t>(index)];
            TriangulationResult triangulated;
            if (triangulateTwoView(first_camera, initialization.first, correspondence.first,
                                   second_camera, pnp.pose, correspondence.second, triangulated)) {
                sources.push_back(index);
                points.push_back(triangulated.position_m);
            }
        }
        if (points.size() < 8) {
            break;
        }

        const double error_px = medianReprojectionPx(pnp.pose, points, sources);
        if (!(error_px < best_error_px)) {
            break;  // the refinement only ever accepts a strictly better fit
        }
        const double change = rotationAngleRad(best_second.rotation, pnp.pose.rotation);
        best_second = pnp.pose;
        best_points = std::move(points);
        best_sources = std::move(sources);
        best_error_px = error_px;
        if (change < 1e-6) {
            break;
        }
    }

    initialization.valid = true;
    initialization.second = best_second;
    initialization.points = std::move(best_points);
    initialization.source_index = std::move(best_sources);
    initialization.median_reprojection_px = best_error_px;
    initialization.inliers = static_cast<int>(initialization.points.size());
    return initialization;
}

}  // namespace scand3d::sfm
