#include "sfm/pnp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "sfm/linear.hpp"

namespace scand3d::sfm {
namespace {

/// Jacobian of the pixel projection with respect to a six-parameter pose update:
/// R <- R * Exp(delta_theta), C <- C + delta_translation.
void poseJacobian(
    const CameraModel& camera,
    const Pose& pose,
    const Vec3& point_world_m,
    double jacobian[2][6]) {
    const Vec3 point_camera = worldToCamera(pose, point_world_m);
    const double z = point_camera[2];
    const double inverse_z = 1.0 / z;
    const double du_dx = camera.focal_px * inverse_z;
    const double du_dz = -camera.focal_px * point_camera[0] * inverse_z * inverse_z;
    const double dv_dy = camera.focal_px * inverse_z;
    const double dv_dz = -camera.focal_px * point_camera[1] * inverse_z * inverse_z;

    // R <- R * Exp(delta) gives Xc <- Xc + Xc x delta, so dXc/dtheta = [Xc]x with [a]x b = a x b.
    const double dtheta[3][3] = {{0.0, -z, point_camera[1]},
                                 {z, 0.0, -point_camera[0]},
                                 {-point_camera[1], point_camera[0], 0.0}};
    const Mat3 rt = geometry::transpose3(pose.rotation);

    for (int column = 0; column < 3; ++column) {
        jacobian[0][column] =
            du_dx * dtheta[0][column] + du_dz * dtheta[2][column];
        jacobian[1][column] =
            dv_dy * dtheta[1][column] + dv_dz * dtheta[2][column];
        // dXc/dC = -R^T.
        jacobian[0][3 + column] =
            -du_dx * rt[static_cast<std::size_t>(column)] -
            du_dz * rt[static_cast<std::size_t>(2 * 3 + column)];
        jacobian[1][3 + column] =
            -dv_dy * rt[static_cast<std::size_t>(1 * 3 + column)] -
            dv_dz * rt[static_cast<std::size_t>(2 * 3 + column)];
    }
}

/// Five Gauss-Newton steps on the reprojection error.
void refinePose(
    const std::vector<Vec3>& object_points,
    const std::vector<Vec2>& image_points,
    const std::vector<int>& indices,
    const CameraModel& camera,
    Pose& pose) {
    for (int iteration = 0; iteration < 5; ++iteration) {
        std::vector<double> normal(36, 0.0);
        std::vector<double> gradient(6, 0.0);

        for (const int index : indices) {
            const auto position = static_cast<std::size_t>(index);
            Vec2 projected{};
            double depth = 0.0;
            if (!projectPoint(camera, pose, object_points[position], projected, depth)) {
                return;
            }
            const double residual[2] = {projected[0] - image_points[position][0],
                                        projected[1] - image_points[position][1]};
            double jacobian[2][6];
            poseJacobian(camera, pose, object_points[position], jacobian);

            for (int row = 0; row < 2; ++row) {
                for (int a = 0; a < 6; ++a) {
                    gradient[static_cast<std::size_t>(a)] +=
                        jacobian[row][a] * residual[row];
                    for (int b = 0; b < 6; ++b) {
                        normal[static_cast<std::size_t>(a * 6 + b)] +=
                            jacobian[row][a] * jacobian[row][b];
                    }
                }
            }
        }

        for (int i = 0; i < 6; ++i) {
            normal[static_cast<std::size_t>(i * 6 + i)] += 1e-9;
        }
        std::vector<double> step;
        if (!linear::solveDense(normal, 6, gradient, step)) {
            return;
        }

        // `step` solves H d = J^T r, so it is the ascent direction: subtract it.
        const Vec3 rotation_step{-step[0], -step[1], -step[2]};
        pose.rotation = geometry::matmul3(pose.rotation, rotationFromAxisAngle(rotation_step));
        pose.camera_centre_m = {pose.camera_centre_m[0] - step[3],
                                pose.camera_centre_m[1] - step[4],
                                pose.camera_centre_m[2] - step[5]};

        if (geometry::dot3(rotation_step, rotation_step) +
                step[3] * step[3] + step[4] * step[4] + step[5] * step[5] <
            1e-20) {
            return;
        }
    }
}

double medianOf(std::vector<double> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

}  // namespace

bool solvePoseDlt(
    const std::vector<Vec3>& object_points,
    const std::vector<Vec2>& image_points,
    const CameraModel& camera,
    Pose& pose) {
    if (object_points.size() < 6 || object_points.size() != image_points.size() ||
        !(camera.focal_px > 0.0)) {
        return false;
    }

    std::vector<double> normal(144, 0.0);  // 12x12
    for (std::size_t i = 0; i < object_points.size(); ++i) {
        const Vec3& point = object_points[i];
        const Vec2& pixel = image_points[i];
        const std::array<double, 12> row_u{point[0], point[1], point[2], 1.0, 0.0, 0.0, 0.0, 0.0,
                                           -pixel[0] * point[0], -pixel[0] * point[1],
                                           -pixel[0] * point[2], -pixel[0]};
        const std::array<double, 12> row_v{0.0, 0.0, 0.0, 0.0, point[0], point[1], point[2], 1.0,
                                           -pixel[1] * point[0], -pixel[1] * point[1],
                                           -pixel[1] * point[2], -pixel[1]};
        for (int a = 0; a < 12; ++a) {
            for (int b = 0; b < 12; ++b) {
                normal[static_cast<std::size_t>(a * 12 + b)] +=
                    row_u[static_cast<std::size_t>(a)] * row_u[static_cast<std::size_t>(b)] +
                    row_v[static_cast<std::size_t>(a)] * row_v[static_cast<std::size_t>(b)];
            }
        }
    }

    std::vector<double> vectors;
    std::vector<double> values;
    if (!geometry::symmetricEigen(normal, 12, vectors, values)) {
        return false;
    }
    int smallest = 0;
    for (int i = 1; i < 12; ++i) {
        if (values[static_cast<std::size_t>(i)] < values[static_cast<std::size_t>(smallest)]) {
            smallest = i;
        }
    }

    const auto value = [&vectors, smallest](int index) {
        return vectors[static_cast<std::size_t>(index * 12 + smallest)];
    };

    // P = K [R^T | -R^T C], so K^-1 P removes the intrinsics.
    const double inverse_focal = 1.0 / camera.focal_px;
    const double cx_over_f = camera.cx_px * inverse_focal;
    const double cy_over_f = camera.cy_px * inverse_focal;
    std::array<double, 12> normalized{};
    for (int column = 0; column < 4; ++column) {
        const double p1 = value(column);
        const double p2 = value(4 + column);
        const double p3 = value(8 + column);
        normalized[static_cast<std::size_t>(column)] = inverse_focal * p1 - cx_over_f * p3;
        normalized[static_cast<std::size_t>(4 + column)] = inverse_focal * p2 - cy_over_f * p3;
        normalized[static_cast<std::size_t>(8 + column)] = p3;
    }

    double scale = 0.0;
    for (int column = 0; column < 3; ++column) {
        scale += std::sqrt(normalized[static_cast<std::size_t>(column)] *
                               normalized[static_cast<std::size_t>(column)] +
                           normalized[static_cast<std::size_t>(4 + column)] *
                               normalized[static_cast<std::size_t>(4 + column)] +
                           normalized[static_cast<std::size_t>(8 + column)] *
                               normalized[static_cast<std::size_t>(8 + column)]);
    }
    scale /= 3.0;
    if (!(scale > 1e-12)) {
        return false;
    }
    for (double& entry : normalized) {
        entry /= scale;
    }

    const Mat3 rotation_world_to_camera{normalized[0], normalized[1], normalized[2],
                                        normalized[4], normalized[5], normalized[6],
                                        normalized[8], normalized[9], normalized[10]};
    Mat3 u{};
    Mat3 v{};
    std::array<double, 3> singular_values{};
    if (!geometry::svd3(rotation_world_to_camera, u, singular_values, v)) {
        return false;
    }
    Mat3 orthonormal = geometry::matmul3(u, geometry::transpose3(v));
    Vec3 translation{normalized[3], normalized[7], normalized[11]};
    if (geometry::determinant3(orthonormal) < 0.0) {
        // The homogeneous solution is defined up to sign and -P projects identically. Negating
        // all of it keeps the projection and makes the rotation block a proper rotation; flipping
        // a single column instead would turn the camera 180 degrees around that axis.
        orthonormal = {-orthonormal[0], -orthonormal[1], -orthonormal[2],
                       -orthonormal[3], -orthonormal[4], -orthonormal[5],
                       -orthonormal[6], -orthonormal[7], -orthonormal[8]};
        translation = {-translation[0], -translation[1], -translation[2]};
    }

    pose.rotation = geometry::transpose3(orthonormal);
    pose.camera_centre_m = geometry::matvec3(pose.rotation,
                                             {-translation[0], -translation[1], -translation[2]});
    return std::isfinite(pose.camera_centre_m[0]) && std::isfinite(pose.camera_centre_m[1]) &&
           std::isfinite(pose.camera_centre_m[2]);
}

PnpResult estimatePoseRansac(
    const std::vector<Vec3>& object_points,
    const std::vector<Vec2>& image_points,
    const CameraModel& camera,
    const PnpOptions& options) {
    PnpResult result;
    const std::size_t count = object_points.size();
    if (count < 6 || count != image_points.size() || !(camera.focal_px > 0.0)) {
        return result;
    }

    std::mt19937 random(options.seed);
    std::uniform_int_distribution<std::size_t> pick(0, count - 1);
    std::vector<int> best_inliers;
    Pose best_pose;
    int iteration_limit = std::max(1, options.max_iterations);
    const double threshold_squared = options.threshold_px * options.threshold_px;

    int iteration = 0;
    while (iteration < iteration_limit) {
        ++iteration;

        std::array<std::size_t, 6> sample{};
        bool unique = false;
        for (int attempt = 0; attempt < 100 && !unique; ++attempt) {
            for (std::size_t& index : sample) {
                index = pick(random);
            }
            std::array<std::size_t, 6> sorted = sample;
            std::sort(sorted.begin(), sorted.end());
            unique = std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end();
        }
        if (!unique) {
            continue;
        }

        std::vector<Vec3> sample_points;
        std::vector<Vec2> sample_pixels;
        sample_points.reserve(6);
        sample_pixels.reserve(6);
        for (const std::size_t index : sample) {
            sample_points.push_back(object_points[index]);
            sample_pixels.push_back(image_points[index]);
        }

        Pose candidate;
        if (!solvePoseDlt(sample_points, sample_pixels, camera, candidate)) {
            continue;
        }

        std::vector<int> inliers;
        std::vector<double> errors;
        for (std::size_t i = 0; i < count; ++i) {
            const double squared = squaredReprojectionPx(camera, candidate, object_points[i],
                                                         image_points[i]);
            if (squared < threshold_squared) {
                inliers.push_back(static_cast<int>(i));
                errors.push_back(squared);
            }
        }

        if (inliers.size() > best_inliers.size()) {
            best_inliers = std::move(inliers);
            best_pose = candidate;

            const double ratio = static_cast<double>(best_inliers.size()) /
                                 static_cast<double>(count);
            if (ratio > 0.0 && ratio < 1.0) {
                const double ratio_to_six = std::pow(ratio, 6.0);
                if (ratio_to_six > 0.0 && ratio_to_six < 1.0) {
                    const double required =
                        std::log(1.0 - options.confidence) / std::log(1.0 - ratio_to_six);
                    if (std::isfinite(required) && required > 0.0 &&
                        required < static_cast<double>(iteration_limit)) {
                        iteration_limit =
                            std::max(iteration, static_cast<int>(std::ceil(required)));
                    }
                }
            }
        }
    }

    if (best_inliers.size() < 6) {
        return result;
    }

    // Refit on all inliers, twice, so the reported pose is the one its inliers support.
    for (int round = 0; round < 2; ++round) {
        std::vector<Vec3> inlier_points;
        std::vector<Vec2> inlier_pixels;
        inlier_points.reserve(best_inliers.size());
        inlier_pixels.reserve(best_inliers.size());
        for (const int index : best_inliers) {
            inlier_points.push_back(object_points[static_cast<std::size_t>(index)]);
            inlier_pixels.push_back(image_points[static_cast<std::size_t>(index)]);
        }

        Pose refined;
        if (!solvePoseDlt(inlier_points, inlier_pixels, camera, refined)) {
            break;
        }
        std::vector<int> indices(inlier_points.size());
        for (std::size_t i = 0; i < indices.size(); ++i) {
            indices[i] = static_cast<int>(i);
        }
        refinePose(inlier_points, inlier_pixels, indices, camera, refined);
        best_pose = refined;

        std::vector<int> reclassified;
        for (std::size_t i = 0; i < count; ++i) {
            if (squaredReprojectionPx(camera, best_pose, object_points[i], image_points[i]) <
                threshold_squared) {
                reclassified.push_back(static_cast<int>(i));
            }
        }
        if (reclassified.size() < 6) {
            break;
        }
        const bool settled = reclassified == best_inliers;
        best_inliers = std::move(reclassified);
        if (settled) {
            break;
        }
    }

    std::vector<double> errors;
    errors.reserve(best_inliers.size());
    for (const int index : best_inliers) {
        errors.push_back(std::sqrt(squaredReprojectionPx(
            camera, best_pose, object_points[static_cast<std::size_t>(index)],
            image_points[static_cast<std::size_t>(index)])));
    }

    result.valid = true;
    result.pose = best_pose;
    result.inlier_indices = std::move(best_inliers);
    result.inliers = static_cast<int>(result.inlier_indices.size());
    result.median_reprojection_px = medianOf(std::move(errors));
    return result;
}

}  // namespace scand3d::sfm
