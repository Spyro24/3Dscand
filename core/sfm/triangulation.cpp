#include "sfm/triangulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace scand3d::sfm {
namespace {

/// Row-major 3x4 projection matrix P = [R^T | -R^T C] for a camera-to-world pose.
std::array<double, 12> projectionMatrix(const Pose& pose) {
    const Mat3 rt = geometry::transpose3(pose.rotation);
    const Vec3 shifted = geometry::matvec3(rt, pose.camera_centre_m);
    return {rt[0], rt[1], rt[2], -shifted[0],
            rt[3], rt[4], rt[5], -shifted[1],
            rt[6], rt[7], rt[8], -shifted[2]};
}

std::array<double, 2> normalizedPixel(const CameraModel& camera, const Vec2& pixel) {
    if (!(camera.focal_px > 0.0)) {
        return {pixel[0], pixel[1]};
    }
    return {(pixel[0] - camera.cx_px) / camera.focal_px,
            (pixel[1] - camera.cy_px) / camera.focal_px};
}

/// Smallest-eigenvector solve of the 2N x 4 homogeneous system, in calibrated coordinates.
bool linearTriangulate(const std::vector<ViewObservation>& observations, Vec3& point_m) {
    if (observations.size() < 2) {
        return false;
    }

    std::array<double, 16> normal_equations{};  // A^T A, 4x4
    for (const ViewObservation& observation : observations) {
        const std::array<double, 12> p = projectionMatrix(observation.pose);
        const std::array<double, 2> xy = normalizedPixel(observation.camera, observation.pixel);

        const std::array<double, 4> row_u{
            xy[0] * p[8] - p[0], xy[0] * p[9] - p[1], xy[0] * p[10] - p[2], xy[0] * p[11] - p[3]};
        const std::array<double, 4> row_v{
            xy[1] * p[8] - p[4], xy[1] * p[9] - p[5], xy[1] * p[10] - p[6], xy[1] * p[11] - p[7]};

        for (int a = 0; a < 4; ++a) {
            for (int b = 0; b < 4; ++b) {
                normal_equations[static_cast<std::size_t>(a * 4 + b)] +=
                    row_u[static_cast<std::size_t>(a)] * row_u[static_cast<std::size_t>(b)] +
                    row_v[static_cast<std::size_t>(a)] * row_v[static_cast<std::size_t>(b)];
            }
        }
    }

    std::vector<double> vectors;
    std::vector<double> values;
    if (!geometry::symmetricEigen(
            std::vector<double>(normal_equations.begin(), normal_equations.end()), 4, vectors,
            values)) {
        return false;
    }

    int smallest = 0;
    for (int i = 1; i < 4; ++i) {
        if (values[static_cast<std::size_t>(i)] < values[static_cast<std::size_t>(smallest)]) {
            smallest = i;
        }
    }

    const double w = vectors[static_cast<std::size_t>(3 * 4 + smallest)];
    if (!(std::fabs(w) > 1e-12)) {
        return false;  // a point at infinity: no usable depth
    }
    point_m = {vectors[static_cast<std::size_t>(0 * 4 + smallest)] / w,
               vectors[static_cast<std::size_t>(1 * 4 + smallest)] / w,
               vectors[static_cast<std::size_t>(2 * 4 + smallest)] / w};
    return std::isfinite(point_m[0]) && std::isfinite(point_m[1]) && std::isfinite(point_m[2]);
}

/// Three Gauss-Newton steps on the pixel reprojection error. Cheap, and it removes the DLT bias.
void refinePoint(const std::vector<ViewObservation>& observations, Vec3& point_m) {
    for (int iteration = 0; iteration < 3; ++iteration) {
        std::array<double, 9> normal{};  // J^T J
        std::array<double, 3> gradient{};  // J^T r
        for (const ViewObservation& observation : observations) {
            const Vec3 camera_point = worldToCamera(observation.pose, point_m);
            if (!(camera_point[2] > 1e-9)) {
                return;
            }
            const double z = camera_point[2];
            const double predicted_u =
                observation.camera.focal_px * camera_point[0] / z + observation.camera.cx_px;
            const double predicted_v =
                observation.camera.focal_px * camera_point[1] / z + observation.camera.cy_px;
            const double residual_u = predicted_u - observation.pixel[0];
            const double residual_v = predicted_v - observation.pixel[1];

            // d(u,v)/dXc for the pinhole model.
            const double inv_z = 1.0 / z;
            const double du_dx = observation.camera.focal_px * inv_z;
            const double du_dz = -observation.camera.focal_px * camera_point[0] * inv_z * inv_z;
            const double dv_dy = observation.camera.focal_px * inv_z;
            const double dv_dz = -observation.camera.focal_px * camera_point[1] * inv_z * inv_z;
            const double jacobian[2][3] = {{du_dx, 0.0, du_dz}, {0.0, dv_dy, dv_dz}};

            // dXc/dXw = R^T.
            const Mat3 rt = geometry::transpose3(observation.pose.rotation);
            const double residual[2] = {residual_u, residual_v};
            for (int row = 0; row < 2; ++row) {
                for (int col = 0; col < 3; ++col) {
                    double value = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        value += jacobian[row][k] * rt[static_cast<std::size_t>(k * 3 + col)];
                    }
                    gradient[static_cast<std::size_t>(col)] += value * residual[row];
                    for (int other = 0; other < 3; ++other) {
                        double other_value = 0.0;
                        for (int k = 0; k < 3; ++k) {
                            other_value += jacobian[row][k] *
                                           rt[static_cast<std::size_t>(k * 3 + other)];
                        }
                        normal[static_cast<std::size_t>(col * 3 + other)] += value * other_value;
                    }
                }
            }
        }

        // Levenberg-style damping keeps a nearly degenerate point from exploding.
        for (int i = 0; i < 3; ++i) {
            normal[static_cast<std::size_t>(i * 3 + i)] += 1e-9;
        }
        const Mat3 system{normal[0], normal[1], normal[2],
                          normal[3], normal[4], normal[5],
                          normal[6], normal[7], normal[8]};
        const Mat3 inverse = geometry::inverse3(system);
        if (!(geometry::frobeniusNorm3(inverse) > 0.0)) {
            return;
        }
        const Vec3 step = geometry::matvec3(inverse, gradient);
        point_m = {point_m[0] - step[0], point_m[1] - step[1], point_m[2] - step[2]};
        if (!std::isfinite(point_m[0]) || !std::isfinite(point_m[1]) ||
            !std::isfinite(point_m[2])) {
            return;
        }
        if (geometry::dot3(step, step) < 1e-18) {
            return;
        }
    }
}

}  // namespace

bool triangulatePoint(
    const std::vector<ViewObservation>& observations,
    TriangulationResult& result,
    double min_parallax_rad) {
    result = TriangulationResult{};
    if (observations.size() < 2) {
        return false;
    }

    Vec3 point_m{};
    if (!linearTriangulate(observations, point_m)) {
        return false;
    }
    refinePoint(observations, point_m);

    double max_error = 0.0;
    double min_depth = 0.0;
    for (const ViewObservation& observation : observations) {
        Vec2 projected{};
        double depth = 0.0;
        if (!projectPoint(observation.camera, observation.pose, point_m, projected, depth)) {
            return false;
        }
        const double error = std::hypot(projected[0] - observation.pixel[0],
                                        projected[1] - observation.pixel[1]);
        max_error = std::max(max_error, error);
        if (depth < min_depth || min_depth == 0.0) {
            min_depth = depth;
        }
    }

    // Parallax is the angle between the extreme viewing rays; a 1-pixel error at a 1-degree
    // angle moves the point by tens of depths, so such a point is rejected.
    double max_parallax = 0.0;
    std::vector<Vec3> rays;
    rays.reserve(observations.size());
    for (const ViewObservation& observation : observations) {
        // The rays must be compared in one common frame; comparing camera-space directions would
        // cancel out exactly the camera rotation that produces the parallax.
        const Vec3 centre = observation.pose.camera_centre_m;
        const Vec3 ray{point_m[0] - centre[0], point_m[1] - centre[1], point_m[2] - centre[2]};
        const double length = std::sqrt(geometry::dot3(ray, ray));
        if (length > 1e-12) {
            rays.push_back({ray[0] / length, ray[1] / length, ray[2] / length});
        }
    }
    for (std::size_t i = 0; i < rays.size(); ++i) {
        for (std::size_t j = i + 1; j < rays.size(); ++j) {
            const double cosine = std::clamp(geometry::dot3(rays[i], rays[j]), -1.0, 1.0);
            max_parallax = std::max(max_parallax, std::acos(cosine));
        }
    }

    if (!(min_depth > 0.0) || max_parallax < min_parallax_rad) {
        return false;
    }

    result.position_m = point_m;
    result.max_reprojection_px = max_error;
    result.min_depth_m = min_depth;
    result.max_parallax_rad = max_parallax;
    result.valid = true;
    return true;
}

bool triangulateTwoView(
    const CameraModel& first_camera,
    const Pose& first_pose,
    const Vec2& first_pixel,
    const CameraModel& second_camera,
    const Pose& second_pose,
    const Vec2& second_pixel,
    TriangulationResult& result,
    double min_parallax_rad) {
    const std::vector<ViewObservation> observations{
        {first_camera, first_pose, first_pixel},
        {second_camera, second_pose, second_pixel},
    };
    return triangulatePoint(observations, result, min_parallax_rad);
}

}  // namespace scand3d::sfm
