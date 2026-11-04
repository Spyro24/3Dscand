#include "sfm/bundle_adjustment.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "sfm/linear.hpp"

namespace scand3d::sfm {
namespace {

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

    // R <- R * Exp(delta) gives Xc <- Xc + Xc x delta, so dXc/dtheta = [Xc]x.
    const double dtheta[3][3] = {{0.0, -z, point_camera[1]},
                                 {z, 0.0, -point_camera[0]},
                                 {-point_camera[1], point_camera[0], 0.0}};
    const Mat3 rt = geometry::transpose3(pose.rotation);

    for (int column = 0; column < 3; ++column) {
        jacobian[0][column] = du_dx * dtheta[0][column] + du_dz * dtheta[2][column];
        jacobian[1][column] = dv_dy * dtheta[1][column] + dv_dz * dtheta[2][column];
        jacobian[0][3 + column] =
            -du_dx * rt[static_cast<std::size_t>(column)] -
            du_dz * rt[static_cast<std::size_t>(2 * 3 + column)];
        jacobian[1][3 + column] =
            -dv_dy * rt[static_cast<std::size_t>(1 * 3 + column)] -
            dv_dz * rt[static_cast<std::size_t>(2 * 3 + column)];
    }
}

double huberCost(double squared_error, double huber_px) {
    if (!(huber_px > 0.0)) {
        return squared_error;
    }
    const double threshold_squared = huber_px * huber_px;
    if (squared_error <= threshold_squared) {
        return squared_error;
    }
    return 2.0 * huber_px * std::sqrt(squared_error) - threshold_squared;
}

double huberWeight(double squared_error, double huber_px) {
    if (!(huber_px > 0.0)) {
        return 1.0;
    }
    const double threshold_squared = huber_px * huber_px;
    if (squared_error <= threshold_squared) {
        return 1.0;
    }
    return huber_px / std::sqrt(squared_error);
}

std::vector<double> collectErrors(const Reconstruction& reconstruction) {
    std::vector<double> errors;
    for (const MapPoint& point : reconstruction.points) {
        if (!point.valid) {
            continue;
        }
        for (const Observation& observation : point.observations) {
            const ViewState& view = reconstruction.views[static_cast<std::size_t>(observation.view)];
            if (!view.registered) {
                continue;
            }
            const double squared = squaredReprojectionPx(view.camera, view.pose,
                                                         point.position_m, observation.pixel);
            if (squared < 1e18) {
                errors.push_back(std::sqrt(squared));
            }
        }
    }
    return errors;
}

/// Per-point normal block plus the cross block to the camera parameters.
struct PointBlock {
    std::array<double, 9> h_pp{};
    std::array<double, 3> g_p{};
    std::vector<double> h_cp;  // 3 x camera_parameters
};

}  // namespace

double reprojectionCost(const Reconstruction& reconstruction, double huber_px) {
    double cost = 0.0;
    for (const MapPoint& point : reconstruction.points) {
        if (!point.valid) {
            continue;
        }
        for (const Observation& observation : point.observations) {
            const ViewState& view = reconstruction.views[static_cast<std::size_t>(observation.view)];
            if (!view.registered) {
                continue;
            }
            const double squared = squaredReprojectionPx(view.camera, view.pose,
                                                         point.position_m, observation.pixel);
            if (squared < 1e18) {
                cost += huberCost(squared, huber_px);
            }
        }
    }
    return cost;
}

double meanReprojectionPx(const Reconstruction& reconstruction) {
    const std::vector<double> errors = collectErrors(reconstruction);
    if (errors.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    for (const double error : errors) {
        sum += error;
    }
    return sum / static_cast<double>(errors.size());
}

double medianReprojectionPx(const Reconstruction& reconstruction) {
    std::vector<double> errors = collectErrors(reconstruction);
    if (errors.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    std::sort(errors.begin(), errors.end());
    return errors[errors.size() / 2];
}

BundleAdjustmentResult optimize(
    Reconstruction& reconstruction,
    const BundleAdjustmentOptions& options) {
    BundleAdjustmentResult result;

    // One fixed camera removes most of the gauge freedom; the global scale stays free by
    // construction and the damping keeps it from drifting.
    int fixed_view = -1;
    if (options.fix_first_camera) {
        for (std::size_t v = 0; v < reconstruction.views.size(); ++v) {
            if (reconstruction.views[v].registered) {
                fixed_view = static_cast<int>(v);
                break;
            }
        }
    }

    std::vector<int> camera_blocks(reconstruction.views.size(), -1);
    int camera_count = 0;
    for (std::size_t v = 0; v < reconstruction.views.size(); ++v) {
        if (!reconstruction.views[v].registered || static_cast<int>(v) == fixed_view) {
            continue;
        }
        camera_blocks[v] = camera_count++;
    }
    const int camera_parameters = 6 * camera_count;

    std::vector<int> point_slots(reconstruction.points.size(), -1);
    int point_count = 0;
    for (std::size_t p = 0; p < reconstruction.points.size(); ++p) {
        const MapPoint& point = reconstruction.points[p];
        if (point.valid && point.observations.size() >= 2) {
            point_slots[p] = point_count++;
        }
    }

    result.cameras = camera_count;
    result.points = point_count;
    result.initial_cost = reprojectionCost(reconstruction, options.huber_px);
    result.final_cost = result.initial_cost;
    if (point_count == 0) {
        return result;
    }

    std::vector<Pose> base_poses(reconstruction.views.size());
    for (std::size_t v = 0; v < reconstruction.views.size(); ++v) {
        base_poses[v] = reconstruction.views[v].pose;
    }
    std::vector<Vec3> base_positions(reconstruction.points.size());
    for (std::size_t p = 0; p < reconstruction.points.size(); ++p) {
        base_positions[p] = reconstruction.points[p].position_m;
    }

    for (std::size_t p = 0; p < reconstruction.points.size(); ++p) {
        if (point_slots[p] >= 0) {
            result.observations += static_cast<int>(reconstruction.points[p].observations.size());
        }
    }

    std::vector<Pose> working_poses = base_poses;
    std::vector<Vec3> working_positions = base_positions;
    double current_cost = result.initial_cost;
    double damping = options.initial_damping;

    std::vector<PointBlock> blocks(static_cast<std::size_t>(point_count));
    std::vector<double> h_cc(static_cast<std::size_t>(camera_parameters) *
                                 static_cast<std::size_t>(camera_parameters),
                             0.0);
    std::vector<double> g_c(static_cast<std::size_t>(camera_parameters), 0.0);

    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        result.iterations = iteration + 1;

        std::fill(h_cc.begin(), h_cc.end(), 0.0);
        std::fill(g_c.begin(), g_c.end(), 0.0);
        for (PointBlock& block : blocks) {
            block.h_pp = {};
            block.g_p = {};
            block.h_cp.assign(static_cast<std::size_t>(3 * camera_parameters), 0.0);
        }

        for (std::size_t p = 0; p < reconstruction.points.size(); ++p) {
            if (point_slots[p] < 0) {
                continue;
            }
            PointBlock& block = blocks[static_cast<std::size_t>(point_slots[p])];
            for (const Observation& observation : reconstruction.points[p].observations) {
                const auto view_index = static_cast<std::size_t>(observation.view);
                const ViewState& view = reconstruction.views[view_index];
                if (!view.registered) {
                    continue;
                }

                Vec2 projected{};
                double depth = 0.0;
                if (!projectPoint(view.camera, working_poses[view_index], working_positions[p],
                                  projected, depth)) {
                    continue;
                }
                const double residual[2] = {projected[0] - observation.pixel[0],
                                            projected[1] - observation.pixel[1]};
                const double squared = residual[0] * residual[0] + residual[1] * residual[1];
                const double weight = huberWeight(squared, options.huber_px);

                double jacobian[2][6];
                poseJacobian(view.camera, working_poses[view_index], working_positions[p],
                             jacobian);

                const Vec3 point_camera =
                    worldToCamera(working_poses[view_index], working_positions[p]);
                const double inverse_z = 1.0 / point_camera[2];
                const double du_dx = view.camera.focal_px * inverse_z;
                const double du_dz = -view.camera.focal_px * point_camera[0] * inverse_z * inverse_z;
                const double dv_dy = view.camera.focal_px * inverse_z;
                const double dv_dz = -view.camera.focal_px * point_camera[1] * inverse_z * inverse_z;
                const Mat3 rt = geometry::transpose3(working_poses[view_index].rotation);
                double jacobian_point[2][3];
                for (int column = 0; column < 3; ++column) {
                    jacobian_point[0][column] =
                        du_dx * rt[static_cast<std::size_t>(column)] +
                        du_dz * rt[static_cast<std::size_t>(2 * 3 + column)];
                    jacobian_point[1][column] =
                        dv_dy * rt[static_cast<std::size_t>(1 * 3 + column)] +
                        dv_dz * rt[static_cast<std::size_t>(2 * 3 + column)];
                }

                for (int row = 0; row < 2; ++row) {
                    for (int a = 0; a < 3; ++a) {
                        block.g_p[static_cast<std::size_t>(a)] +=
                            weight * jacobian_point[row][a] * residual[row];
                        for (int b = 0; b < 3; ++b) {
                            block.h_pp[static_cast<std::size_t>(a * 3 + b)] +=
                                weight * jacobian_point[row][a] * jacobian_point[row][b];
                        }
                        const int camera_block = camera_blocks[view_index];
                        if (camera_block >= 0) {
                            for (int b = 0; b < 6; ++b) {
                                block.h_cp[static_cast<std::size_t>(a * camera_parameters +
                                                                    camera_block * 6 + b)] +=
                                    weight * jacobian_point[row][a] * jacobian[row][b];
                            }
                        }
                    }

                    const int camera_block = camera_blocks[view_index];
                    if (camera_block >= 0) {
                        for (int a = 0; a < 6; ++a) {
                            g_c[static_cast<std::size_t>(camera_block * 6 + a)] +=
                                weight * jacobian[row][a] * residual[row];
                            for (int b = 0; b < 6; ++b) {
                                h_cc[static_cast<std::size_t>((camera_block * 6 + a) *
                                                                  camera_parameters +
                                                              camera_block * 6 + b)] +=
                                    weight * jacobian[row][a] * jacobian[row][b];
                            }
                        }
                    }
                }
            }
        }

        // Schur complement: eliminate the point blocks, solve only for the cameras.
        std::vector<double> schur = h_cc;
        std::vector<double> rhs = g_c;
        for (int i = 0; i < camera_parameters; ++i) {
            schur[static_cast<std::size_t>(i * camera_parameters + i)] *= 1.0 + damping;
            schur[static_cast<std::size_t>(i * camera_parameters + i)] += damping;
        }

        std::vector<std::array<double, 9>> inverses(static_cast<std::size_t>(point_count));
        std::vector<std::array<double, 3>> inverse_times_gradient(
            static_cast<std::size_t>(point_count));
        for (int p = 0; p < point_count; ++p) {
            const PointBlock& block = blocks[static_cast<std::size_t>(p)];
            Mat3 h_pp{block.h_pp[0], block.h_pp[1], block.h_pp[2],
                      block.h_pp[3], block.h_pp[4], block.h_pp[5],
                      block.h_pp[6], block.h_pp[7], block.h_pp[8]};
            for (int i = 0; i < 3; ++i) {
                h_pp[static_cast<std::size_t>(i * 3 + i)] *= 1.0 + damping;
                h_pp[static_cast<std::size_t>(i * 3 + i)] += damping;
            }
            const Mat3 inverse = geometry::inverse3(h_pp);
            inverses[static_cast<std::size_t>(p)] = inverse;
            const Vec3 solved = geometry::matvec3(
                inverse, {block.g_p[0], block.g_p[1], block.g_p[2]});
            inverse_times_gradient[static_cast<std::size_t>(p)] = {solved[0], solved[1], solved[2]};

            if (camera_parameters == 0) {
                continue;
            }

            // schur -= B^T Hpp^-1 B, rhs -= B^T Hpp^-1 g_p
            std::vector<double> solved_block(static_cast<std::size_t>(3 * camera_parameters), 0.0);
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < camera_parameters; ++b) {
                    double sum = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        sum += inverse[static_cast<std::size_t>(a * 3 + k)] *
                               block.h_cp[static_cast<std::size_t>(k * camera_parameters + b)];
                    }
                    solved_block[static_cast<std::size_t>(a * camera_parameters + b)] = sum;
                }
            }
            for (int a = 0; a < camera_parameters; ++a) {
                for (int b = 0; b < camera_parameters; ++b) {
                    double sum = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        sum += block.h_cp[static_cast<std::size_t>(k * camera_parameters + a)] *
                               solved_block[static_cast<std::size_t>(k * camera_parameters + b)];
                    }
                    schur[static_cast<std::size_t>(a * camera_parameters + b)] -= sum;
                }
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += block.h_cp[static_cast<std::size_t>(k * camera_parameters + a)] *
                           inverse_times_gradient[static_cast<std::size_t>(p)]
                                                 [static_cast<std::size_t>(k)];
                }
                rhs[static_cast<std::size_t>(a)] -= sum;
            }
        }

        std::vector<double> camera_step;
        if (camera_parameters > 0) {
            if (!linear::solveDense(schur, camera_parameters, rhs, camera_step)) {
                damping *= 10.0;
                if (damping > 1e12) {
                    break;
                }
                continue;
            }
        }

        std::vector<std::array<double, 3>> point_steps(static_cast<std::size_t>(point_count));
        for (int p = 0; p < point_count; ++p) {
            const PointBlock& block = blocks[static_cast<std::size_t>(p)];
            Vec3 step{inverse_times_gradient[static_cast<std::size_t>(p)][0],
                      inverse_times_gradient[static_cast<std::size_t>(p)][1],
                      inverse_times_gradient[static_cast<std::size_t>(p)][2]};
            if (camera_parameters > 0) {
                for (int a = 0; a < 3; ++a) {
                    double correction = 0.0;
                    for (int b = 0; b < camera_parameters; ++b) {
                        correction += block.h_cp[static_cast<std::size_t>(a * camera_parameters + b)] *
                                      camera_step[static_cast<std::size_t>(b)];
                    }
                    step[static_cast<std::size_t>(a)] -= correction;
                }
                const Mat3 inverse = inverses[static_cast<std::size_t>(p)];
                step = geometry::matvec3(inverse, step);
            }
            point_steps[static_cast<std::size_t>(p)] = {step[0], step[1], step[2]};
        }

        // Apply the step on a working copy and keep it only if the cost actually drops.
        std::vector<Pose> trial_poses = working_poses;
        std::vector<Vec3> trial_positions = working_positions;
        for (std::size_t v = 0; v < trial_poses.size(); ++v) {
            const int block = camera_blocks[v];
            if (block < 0) {
                continue;
            }
            const std::size_t offset = static_cast<std::size_t>(block * 6);
            // The system is H d = J^T r, so d is the ascent direction and the update subtracts it.
            const Vec3 rotation_step{-camera_step[offset], -camera_step[offset + 1],
                                     -camera_step[offset + 2]};
            trial_poses[v].rotation = geometry::matmul3(trial_poses[v].rotation,
                                                        rotationFromAxisAngle(rotation_step));
            trial_poses[v].camera_centre_m = {trial_poses[v].camera_centre_m[0] -
                                                  camera_step[offset + 3],
                                              trial_poses[v].camera_centre_m[1] -
                                                  camera_step[offset + 4],
                                              trial_poses[v].camera_centre_m[2] -
                                                  camera_step[offset + 5]};
        }
        for (std::size_t p = 0; p < trial_positions.size(); ++p) {
            const int slot = point_slots[p];
            if (slot < 0) {
                continue;
            }
            trial_positions[p] = {
                trial_positions[p][0] - point_steps[static_cast<std::size_t>(slot)][0],
                trial_positions[p][1] - point_steps[static_cast<std::size_t>(slot)][1],
                trial_positions[p][2] - point_steps[static_cast<std::size_t>(slot)][2]};
        }

        for (std::size_t v = 0; v < reconstruction.views.size(); ++v) {
            reconstruction.views[v].pose = trial_poses[v];
        }
        for (std::size_t p = 0; p < reconstruction.points.size(); ++p) {
            reconstruction.points[p].position_m = trial_positions[p];
        }
        const double trial_cost = reprojectionCost(reconstruction, options.huber_px);

        if (trial_cost <= current_cost) {
            const double improvement = current_cost - trial_cost;
            working_poses = std::move(trial_poses);
            working_positions = std::move(trial_positions);
            current_cost = trial_cost;
            damping = std::max(damping * 0.1, 1e-12);
            if (improvement < options.cost_tolerance * std::max(1.0, current_cost)) {
                break;
            }
        } else {
            for (std::size_t v = 0; v < reconstruction.views.size(); ++v) {
                reconstruction.views[v].pose = working_poses[v];
            }
            for (std::size_t p = 0; p < reconstruction.points.size(); ++p) {
                reconstruction.points[p].position_m = working_positions[p];
            }
            damping *= 10.0;
            if (damping > 1e12) {
                break;
            }
        }
    }

    result.final_cost = current_cost;
    return result;
}

}  // namespace scand3d::sfm
