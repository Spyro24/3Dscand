#include "synthetic/sfm_scan.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>

namespace scand3d::synthetic {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr double kFocalPx = 500.0;
constexpr double kRingRadiusM = 1.6;
constexpr double kRingHeightM = 0.5;
constexpr double kTargetHeightM = 0.6;
constexpr unsigned int kNoiseSeed = 20261104u;

}  // namespace

Camera sfmCameraAt(double angle_rad, double radius_m, double height_m) {
    Camera camera;
    camera.width_px = kImageWidth;
    camera.height_px = kImageHeight;
    camera.focal_px = kFocalPx;
    camera.cx_px = 0.5 * kImageWidth;
    camera.cy_px = 0.5 * kImageHeight;
    const Vec3 eye{radius_m * std::cos(angle_rad), radius_m * std::sin(angle_rad), height_m};
    camera.pose_camera_to_world =
        lookAtCameraToWorld(eye, {0.0, 0.0, kTargetHeightM}, {0.0, 0.0, 1.0});
    return camera;
}

sfm::CameraModel cameraModelOf(const Camera& camera) {
    return {camera.focal_px, camera.cx_px, camera.cy_px};
}

sfm::Pose poseOf(const Camera& camera) {
    const auto& m = camera.pose_camera_to_world;
    sfm::Pose pose;
    pose.rotation = {m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]};
    pose.camera_centre_m = {m[3], m[7], m[11]};
    return pose;
}

std::vector<sfm::Vec3> latticePoints() {
    std::vector<sfm::Vec3> points;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                points.push_back({-0.30 + 0.20 * i, -0.24 + 0.16 * j, 0.36 + 0.16 * k});
            }
        }
    }
    return points;
}

SfmScan makeLatticeScan(int view_count, double noise_px) {
    SfmScan scan;
    if (view_count <= 0) {
        return scan;
    }
    scan.points = latticePoints();
    scan.feature_of_point.assign(static_cast<std::size_t>(view_count),
                                 std::vector<int>(scan.points.size(), -1));

    std::mt19937 random(kNoiseSeed);
    std::normal_distribution<double> noise(0.0, noise_px);

    for (int v = 0; v < view_count; ++v) {
        const double angle = 2.0 * kPi * static_cast<double>(v) / static_cast<double>(view_count);
        const Camera camera = sfmCameraAt(angle, kRingRadiusM, kRingHeightM);
        sfm::FrameInput frame;
        frame.camera = cameraModelOf(camera);
        for (std::size_t p = 0; p < scan.points.size(); ++p) {
            const Projection projection = projectPoint(camera, scan.points[p]);
            if (!projection.in_front || projection.u_px < 0.0 || projection.v_px < 0.0 ||
                projection.u_px >= kImageWidth || projection.v_px >= kImageHeight) {
                continue;
            }
            scan.feature_of_point[static_cast<std::size_t>(v)][p] =
                static_cast<int>(frame.pixels.size());
            frame.pixels.push_back(
                {projection.u_px + noise(random), projection.v_px + noise(random)});
        }
        scan.frames.push_back(std::move(frame));
        scan.ground_truth.push_back(poseOf(camera));
    }

    for (int v = 0; v < view_count; ++v) {
        const int next = (v + 1) % view_count;
        sfm::PairMatches pair;
        pair.view_a = v;
        pair.view_b = next;
        for (std::size_t p = 0; p < scan.points.size(); ++p) {
            const int feature_a = scan.feature_of_point[static_cast<std::size_t>(v)][p];
            const int feature_b = scan.feature_of_point[static_cast<std::size_t>(next)][p];
            if (feature_a >= 0 && feature_b >= 0) {
                pair.matches.emplace_back(feature_a, feature_b);
            }
        }
        if (!pair.matches.empty()) {
            scan.pairs.push_back(std::move(pair));
        }
    }
    return scan;
}

double directionErrorDeg(const sfm::Vec3& first, const sfm::Vec3& second) {
    const double first_length = std::sqrt(geometry::dot3(first, first));
    const double second_length = std::sqrt(geometry::dot3(second, second));
    if (!(first_length > 0.0) || !(second_length > 0.0)) {
        return 180.0;
    }
    const double cosine =
        geometry::dot3(first, second) / (first_length * second_length);
    return std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / kPi;
}

double relativeRotationErrorDeg(
    const sfm::Reconstruction& reconstruction,
    const std::vector<sfm::Pose>& ground_truth,
    int seed,
    int index) {
    const auto& views = reconstruction.views;
    const geometry::Mat3 estimated = geometry::matmul3(
        geometry::transpose3(views[static_cast<std::size_t>(seed)].pose.rotation),
        views[static_cast<std::size_t>(index)].pose.rotation);
    const geometry::Mat3 expected = geometry::matmul3(
        geometry::transpose3(ground_truth[static_cast<std::size_t>(seed)].rotation),
        ground_truth[static_cast<std::size_t>(index)].rotation);
    return sfm::rotationAngleRad(estimated, expected) * 180.0 / kPi;
}

double relativeCentreDirectionErrorDeg(
    const sfm::Reconstruction& reconstruction,
    const std::vector<sfm::Pose>& ground_truth,
    int seed,
    int index) {
    const auto& views = reconstruction.views;
    const geometry::Mat3 seed_rotation = ground_truth[static_cast<std::size_t>(seed)].rotation;
    const sfm::Vec3 seed_centre = ground_truth[static_cast<std::size_t>(seed)].camera_centre_m;
    const sfm::Vec3 truth_delta{
        ground_truth[static_cast<std::size_t>(index)].camera_centre_m[0] - seed_centre[0],
        ground_truth[static_cast<std::size_t>(index)].camera_centre_m[1] - seed_centre[1],
        ground_truth[static_cast<std::size_t>(index)].camera_centre_m[2] - seed_centre[2]};
    const sfm::Vec3 expected =
        geometry::matvec3(geometry::transpose3(seed_rotation), truth_delta);

    const sfm::Vec3 estimate_delta{
        views[static_cast<std::size_t>(index)].pose.camera_centre_m[0] -
            views[static_cast<std::size_t>(seed)].pose.camera_centre_m[0],
        views[static_cast<std::size_t>(index)].pose.camera_centre_m[1] -
            views[static_cast<std::size_t>(seed)].pose.camera_centre_m[1],
        views[static_cast<std::size_t>(index)].pose.camera_centre_m[2] -
            views[static_cast<std::size_t>(seed)].pose.camera_centre_m[2]};
    const sfm::Vec3 actual = geometry::matvec3(
        geometry::transpose3(views[static_cast<std::size_t>(seed)].pose.rotation),
        estimate_delta);

    return directionErrorDeg(actual, expected);
}

}  // namespace scand3d::synthetic
