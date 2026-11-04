#include "mvs/patchmatch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "geometry/linalg.hpp"
#include "sfm/types.hpp"

namespace scand3d::mvs {
namespace {

using geometry::Mat3;
using geometry::Vec3;

constexpr double kPi = 3.14159265358979323846;
/// Cost of a plane that cannot be tested at all: outside the images, or a patch with no texture.
constexpr double kNoMatch = 2.0;
/// Fixed seed: the same capture must produce the same depth map twice.
constexpr std::uint32_t kRandomSeed = 0x5CA4D3D5u;

struct Hypothesis {
    double depth_m = 0.0;
    Vec3 normal{0.0, 0.0, -1.0};
    double cost = kNoMatch;
};

Mat3 intrinsics(const CameraModel& camera) {
    return {camera.focal_px, 0.0, camera.cx_px,
            0.0, camera.focal_px, camera.cy_px,
            0.0, 0.0, 1.0};
}

Mat3 inverseIntrinsics(const CameraModel& camera) {
    const double focal = camera.focal_px;
    return {1.0 / focal, 0.0, -camera.cx_px / focal,
            0.0, 1.0 / focal, -camera.cy_px / focal,
            0.0, 0.0, 1.0};
}

Vec3 normalized(const Vec3& vector) {
    const double length = std::sqrt(geometry::dot3(vector, vector));
    if (!(length > 1e-12)) {
        return {0.0, 0.0, -1.0};
    }
    return {vector[0] / length, vector[1] / length, vector[2] / length};
}

bool sampleBilinear(const feature::GrayImage& image, double x, double y, double& out) {
    if (!(x >= 0.0) || !(y >= 0.0)) {
        return false;
    }
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= image.width_px || y0 + 1 >= image.height_px) {
        return false;
    }
    const double fx = x - x0;
    const double fy = y - y0;
    const double top = image.at(x0, y0) * (1.0 - fx) + image.at(x0 + 1, y0) * fx;
    const double bottom = image.at(x0, y0 + 1) * (1.0 - fx) + image.at(x0 + 1, y0 + 1) * fx;
    out = top * (1.0 - fy) + bottom * fy;
    return true;
}

Vec3 randomUnitVector(std::mt19937& rng) {
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Vec3 vector{unit(rng), unit(rng), unit(rng)};
    while (!(geometry::dot3(vector, vector) > 1e-6)) {
        vector = {unit(rng), unit(rng), unit(rng)};
    }
    return normalized(vector);
}

Vec3 rotatedNormal(std::mt19937& rng, const Vec3& normal, double angle_rad) {
    if (!(angle_rad > 1e-9)) {
        return normal;
    }
    Vec3 axis = normalized(geometry::cross3(normal, randomUnitVector(rng)));
    const Vec3 axis_angle{axis[0] * angle_rad, axis[1] * angle_rad, axis[2] * angle_rad};
    return normalized(geometry::matvec3(sfm::rotationFromAxisAngle(axis_angle), normal));
}

/// Zero-mean normalised cross correlation of a plane hypothesis' patch against the neighbour.
class CostEvaluator {
public:
    CostEvaluator(
        const feature::GrayImage& reference,
        const feature::GrayImage& neighbour,
        const CameraModel& reference_camera,
        const CameraModel& neighbour_camera,
        const RelativePose& relative,
        int radius_px)
        : reference_(reference),
          neighbour_(neighbour),
          inverse_reference_(inverseIntrinsics(reference_camera)),
          neighbour_intrinsics_(intrinsics(neighbour_camera)),
          rotation_(relative.rotation),
          translation_(relative.translation_m),
          radius_(radius_px) {}

    double cost(int x_px, int y_px, double depth_m, const Vec3& normal) const {
        if (!(depth_m > 0.0) || !std::isfinite(depth_m)) {
            return kNoMatch;
        }
        const Vec3 ray = geometry::matvec3(
            inverse_reference_, {static_cast<double>(x_px), static_cast<double>(y_px), 1.0});
        // Everything on the patch lies on the plane n . X = plane_offset (D045).
        const double plane_offset = geometry::dot3(normal, ray) * depth_m;
        if (!(std::abs(plane_offset) > 1e-9)) {
            return kNoMatch;
        }
        const double inverse_offset = 1.0 / plane_offset;

        Mat3 relative = rotation_;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                relative[row * 3 + column] +=
                    translation_[row] * normal[column] * inverse_offset;
            }
        }
        const Mat3 homography = geometry::matmul3(
            neighbour_intrinsics_, geometry::matmul3(relative, inverse_reference_));

        double sum_a = 0.0;
        double sum_b = 0.0;
        double sum_aa = 0.0;
        double sum_bb = 0.0;
        double sum_ab = 0.0;
        const int span = 2 * radius_ + 1;
        for (int dy = -radius_; dy <= radius_; ++dy) {
            const int y = y_px + dy;
            if (y < 0 || y >= reference_.height_px) {
                return kNoMatch;
            }
            for (int dx = -radius_; dx <= radius_; ++dx) {
                const int x = x_px + dx;
                if (x < 0 || x >= reference_.width_px) {
                    return kNoMatch;
                }
                const double reference_value = reference_.at(x, y);
                const Vec3 warped = geometry::matvec3(
                    homography, {static_cast<double>(x), static_cast<double>(y), 1.0});
                if (!(warped[2] > 1e-9)) {
                    return kNoMatch;
                }
                double neighbour_value = 0.0;
                if (!sampleBilinear(
                        neighbour_, warped[0] / warped[2], warped[1] / warped[2],
                        neighbour_value)) {
                    return kNoMatch;
                }
                sum_a += reference_value;
                sum_b += neighbour_value;
                sum_aa += reference_value * reference_value;
                sum_bb += neighbour_value * neighbour_value;
                sum_ab += reference_value * neighbour_value;
            }
        }

        const double count = static_cast<double>(span) * span;
        const double variance_a = sum_aa - sum_a * sum_a / count;
        const double variance_b = sum_bb - sum_b * sum_b / count;
        // A flat patch has no correlation to maximise, so it is not evidence of any depth.
        if (!(variance_a > 1e-6) || !(variance_b > 1e-6)) {
            return kNoMatch;
        }
        const double covariance = sum_ab - sum_a * sum_b / count;
        double zncc = covariance / std::sqrt(variance_a * variance_b);
        zncc = std::min(1.0, std::max(-1.0, zncc));
        return 1.0 - zncc;
    }

private:
    const feature::GrayImage& reference_;
    const feature::GrayImage& neighbour_;
    Mat3 inverse_reference_;
    Mat3 neighbour_intrinsics_;
    Mat3 rotation_;
    Vec3 translation_;
    int radius_ = 2;
};

}  // namespace

RelativePose relativePose(const Pose& reference, const Pose& neighbour) {
    const Mat3 neighbour_transpose = geometry::transpose3(neighbour.rotation);
    const Vec3 delta{
        reference.camera_centre_m[0] - neighbour.camera_centre_m[0],
        reference.camera_centre_m[1] - neighbour.camera_centre_m[1],
        reference.camera_centre_m[2] - neighbour.camera_centre_m[2]};
    return {geometry::matmul3(neighbour_transpose, reference.rotation),
            geometry::matvec3(neighbour_transpose, delta)};
}

DepthMap estimateDepth(
    const feature::GrayImage& reference,
    const feature::GrayImage& neighbour,
    const CameraModel& reference_camera,
    const CameraModel& neighbour_camera,
    const RelativePose& relative,
    const PatchMatchOptions& options,
    const std::atomic<bool>* cancel) {
    DepthMap map = DepthMap::sized(reference.width_px, reference.height_px);
    if (map.empty() || neighbour.empty()) {
        return map;
    }
    if (!(reference_camera.focal_px > 0.0) || !(neighbour_camera.focal_px > 0.0)) {
        return map;
    }
    const double minimum_depth = std::min(options.min_depth_m, options.max_depth_m);
    const double maximum_depth = std::max(options.min_depth_m, options.max_depth_m);
    if (!(minimum_depth > 0.0) || !(maximum_depth > minimum_depth)) {
        return map;
    }

    const int width = map.width_px;
    const int height = map.height_px;
    const CostEvaluator evaluator(
        reference, neighbour, reference_camera, neighbour_camera, relative,
        std::max(1, options.patch_radius_px));
    const Mat3 inverse_k = inverseIntrinsics(reference_camera);
    const double log_minimum = std::log(minimum_depth);
    const double log_maximum = std::log(maximum_depth);
    const double log_span = log_maximum - log_minimum;

    std::mt19937 rng(kRandomSeed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<Hypothesis> planes(static_cast<std::size_t>(width) * height);

    // Plane sweep: a log-spaced depth per pixel with the normal facing the camera. This gives
    // propagation a usable starting plane instead of a random one (D045).
    for (int y = 0; y < height; ++y) {
        if (cancel != nullptr && cancel->load()) {
            return map;
        }
        for (int x = 0; x < width; ++x) {
            const Vec3 ray = normalized(geometry::matvec3(
                inverse_k, {static_cast<double>(x), static_cast<double>(y), 1.0}));
            const Vec3 normal{-ray[0], -ray[1], -ray[2]};
            Hypothesis best;
            best.normal = normal;
            const int samples = options.depth_samples;
            if (samples > 1) {
                for (int sample = 0; sample < samples; ++sample) {
                    const double t = static_cast<double>(sample) / (samples - 1);
                    const double depth = std::exp(log_minimum + log_span * t);
                    const double cost = evaluator.cost(x, y, depth, normal);
                    if (cost < best.cost) {
                        best = {depth, normal, cost};
                    }
                }
            }
            if (!(best.depth_m > 0.0)) {
                const double depth = std::exp(log_minimum + log_span * unit(rng));
                best = {depth, normal, evaluator.cost(x, y, depth, normal)};
            }
            planes[static_cast<std::size_t>(y) * width + x] = best;
        }
    }

    // Propagation and random search. Depending on the sweep, each pass compares against the
    // left/up or the right/down neighbour, so information travels across the whole image.
    const int iterations = std::max(1, options.iterations);
    for (int iteration = 0; iteration < iterations; ++iteration) {
        const double depth_amplitude =
            0.5 * (maximum_depth - minimum_depth) * std::pow(0.5, iteration);
        const double angle_amplitude =
            options.initial_normal_deg * kPi / 180.0 * std::pow(0.5, iteration);
        const bool forward = (iteration % 2) == 0;
        for (int row = 0; row < height; ++row) {
            if (cancel != nullptr && cancel->load()) {
                return map;
            }
            const int y = forward ? row : height - 1 - row;
            for (int column = 0; column < width; ++column) {
                const int x = forward ? column : width - 1 - column;
                const std::size_t index = static_cast<std::size_t>(y) * width + x;
                Hypothesis best = planes[index];

                const int offsets_x[2] = {forward ? -1 : 1, 0};
                const int offsets_y[2] = {0, forward ? -1 : 1};
                for (int side = 0; side < 2; ++side) {
                    const int donor_x = x + offsets_x[side];
                    const int donor_y = y + offsets_y[side];
                    if (donor_x < 0 || donor_y < 0 || donor_x >= width || donor_y >= height) {
                        continue;
                    }
                    const Hypothesis& donor =
                        planes[static_cast<std::size_t>(donor_y) * width + donor_x];
                    if (!(donor.depth_m > 0.0) || !(donor.cost < kNoMatch)) {
                        continue;
                    }
                    const double cost = evaluator.cost(x, y, donor.depth_m, donor.normal);
                    if (cost < best.cost) {
                        best = {donor.depth_m, donor.normal, cost};
                    }
                }

                const Vec3 ray = normalized(geometry::matvec3(
                    inverse_k, {static_cast<double>(x), static_cast<double>(y), 1.0}));
                for (int trial = 0; trial < options.random_trials; ++trial) {
                    const double depth = std::min(
                        maximum_depth,
                        std::max(
                            minimum_depth,
                            best.depth_m + (unit(rng) * 2.0 - 1.0) * depth_amplitude));
                    Vec3 normal = rotatedNormal(rng, best.normal, angle_amplitude * unit(rng));
                    if (geometry::dot3(normal, ray) > 0.0) {
                        normal = {-normal[0], -normal[1], -normal[2]};
                    }
                    const double cost = evaluator.cost(x, y, depth, normal);
                    if (cost < best.cost) {
                        best = {depth, normal, cost};
                    }
                }
                planes[index] = best;
            }
        }
    }

    for (std::size_t index = 0; index < planes.size(); ++index) {
        const Hypothesis& plane = planes[index];
        if (!(plane.depth_m > 0.0) || !(plane.cost <= options.max_cost)) {
            continue;
        }
        map.depth_m[index] = static_cast<float>(plane.depth_m);
        map.normal_camera[index * 3] = static_cast<float>(plane.normal[0]);
        map.normal_camera[index * 3 + 1] = static_cast<float>(plane.normal[1]);
        map.normal_camera[index * 3 + 2] = static_cast<float>(plane.normal[2]);
        map.cost[index] = static_cast<float>(plane.cost);
    }
    return map;
}

}  // namespace scand3d::mvs
