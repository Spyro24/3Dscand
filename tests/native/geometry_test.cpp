#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "geometry/linalg.hpp"
#include "geometry/two_view.hpp"
#include "synthetic/synthetic_scene.hpp"

namespace {

using scand3d::geometry::CameraIntrinsics;
using scand3d::geometry::Mat3;
using scand3d::geometry::PointCorrespondence;
using scand3d::geometry::RansacOptions;
using scand3d::geometry::Vec2;
using scand3d::geometry::Vec3;

scand3d::synthetic::Camera makeCamera(const Vec3& eye_m, const Vec3& target_m,
                                      double focal_px = 500.0) {
    scand3d::synthetic::Camera camera;
    camera.width_px = 640;
    camera.height_px = 480;
    camera.focal_px = focal_px;
    camera.cx_px = 320.0;
    camera.cy_px = 240.0;
    camera.pose_camera_to_world =
        scand3d::synthetic::lookAtCameraToWorld(eye_m, target_m, {0.0, 0.0, 1.0});
    return camera;
}

Mat3 rotationOf(const scand3d::synthetic::Mat4& pose) {
    // Row-major camera-to-world pose: the top-left 3x3 block is the camera-to-world rotation.
    return {pose[0], pose[1], pose[2],
            pose[4], pose[5], pose[6],
            pose[8], pose[9], pose[10]};
}

Vec3 translationOf(const scand3d::synthetic::Mat4& pose) {
    return {pose[3], pose[7], pose[11]};
}

Mat3 skew(const Vec3& t) {
    return {0.0, -t[2], t[1],
            t[2], 0.0, -t[0],
            -t[1], t[0], 0.0};
}

CameraIntrinsics intrinsicsOf(const scand3d::synthetic::Camera& camera) {
    return {camera.focal_px, camera.cx_px, camera.cy_px};
}

Mat3 calibrationMatrix(const scand3d::synthetic::Camera& camera) {
    return {camera.focal_px, 0.0, camera.cx_px,
            0.0, camera.focal_px, camera.cy_px,
            0.0, 0.0, 1.0};
}

/// Relative pose from camera 1 to camera 2, then E = [t]x R and F = K2^-T E K1^-1.
Mat3 groundTruthEssential(const scand3d::synthetic::Camera& first,
                          const scand3d::synthetic::Camera& second) {
    const Mat3 r1 = rotationOf(first.pose_camera_to_world);
    const Mat3 r2 = rotationOf(second.pose_camera_to_world);
    const Vec3 t1 = translationOf(first.pose_camera_to_world);
    const Vec3 t2 = translationOf(second.pose_camera_to_world);

    const Mat3 r2_transposed = scand3d::geometry::transpose3(r2);
    const Mat3 relative_rotation = scand3d::geometry::matmul3(r2_transposed, r1);
    const Vec3 relative_translation =
        scand3d::geometry::matvec3(r2_transposed, {t1[0] - t2[0], t1[1] - t2[1], t1[2] - t2[2]});
    return scand3d::geometry::matmul3(skew(relative_translation), relative_rotation);
}

Mat3 groundTruthFundamental(const scand3d::synthetic::Camera& first,
                            const scand3d::synthetic::Camera& second) {
    return scand3d::geometry::matmul3(
        scand3d::geometry::transpose3(scand3d::geometry::inverse3(calibrationMatrix(second))),
        scand3d::geometry::matmul3(groundTruthEssential(first, second),
                                   scand3d::geometry::inverse3(calibrationMatrix(first))));
}

struct Dataset {
    std::vector<PointCorrespondence> points;
    std::vector<bool> inlier;
    Mat3 fundamental{};
    Mat3 essential{};
    CameraIntrinsics first_intrinsics{};
    CameraIntrinsics second_intrinsics{};
};

/// A non-coplanar lattice projected through two cameras, with pixel noise and forced outliers.
/// Coplanar points would make the eight-point problem ambiguous, so the lattice is a volume.
Dataset makeDataset(double noise_px = 0.35, double outlier_ratio = 0.35) {
    const auto first = makeCamera({1.4, -0.9, 0.45}, {0.0, 0.0, 0.6});
    const auto second = makeCamera({1.5, 0.6, 0.55}, {0.0, 0.0, 0.6});

    std::vector<std::array<double, 3>> world_points;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                world_points.push_back({-0.30 + 0.20 * i, -0.24 + 0.16 * j, 0.36 + 0.16 * k});
            }
        }
    }

    std::vector<Vec2> first_pixels;
    std::vector<Vec2> second_pixels;
    for (const auto& point : world_points) {
        const auto a = scand3d::synthetic::projectPoint(first, point);
        const auto b = scand3d::synthetic::projectPoint(second, point);
        if (!a.in_front || !b.in_front) {
            continue;
        }
        if (a.u_px < 0.0 || a.v_px < 0.0 || a.u_px >= 640.0 || a.v_px >= 480.0) {
            continue;
        }
        if (b.u_px < 0.0 || b.v_px < 0.0 || b.u_px >= 640.0 || b.v_px >= 480.0) {
            continue;
        }
        first_pixels.push_back({a.u_px, a.v_px});
        second_pixels.push_back({b.u_px, b.v_px});
    }

    std::mt19937 random(20261104u);
    std::normal_distribution<double> noise(0.0, noise_px);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::uniform_int_distribution<std::size_t> pick(0, second_pixels.size() - 1);

    Dataset dataset;
    dataset.fundamental = groundTruthFundamental(first, second);
    dataset.essential = groundTruthEssential(first, second);
    dataset.first_intrinsics = intrinsicsOf(first);
    dataset.second_intrinsics = intrinsicsOf(second);

    for (std::size_t i = 0; i < first_pixels.size(); ++i) {
        PointCorrespondence correspondence;
        correspondence.first = {first_pixels[i][0] + noise(random), first_pixels[i][1] + noise(random)};
        correspondence.second = {
            second_pixels[i][0] + noise(random), second_pixels[i][1] + noise(random)};

        const bool is_outlier = uniform(random) < outlier_ratio;
        if (is_outlier) {
            // A different point's projection: geometrically wrong, but still a plausible pixel.
            std::size_t other = pick(random);
            if (other == i) {
                other = (other + 1) % second_pixels.size();
            }
            correspondence.second = second_pixels[other];
        }
        dataset.points.push_back(correspondence);
        dataset.inlier.push_back(!is_outlier);
    }
    return dataset;
}

double matrixDistanceUpToScale(const Mat3& a, const Mat3& b) {
    auto normalize = [](const Mat3& matrix) {
        Mat3 out = matrix;
        const double norm = scand3d::geometry::frobeniusNorm3(matrix);
        if (norm > 1e-15) {
            for (double& value : out) {
                value /= norm;
            }
        }
        return out;
    };
    auto distance = [](const Mat3& x, const Mat3& y) {
        double sum = 0.0;
        for (std::size_t i = 0; i < x.size(); ++i) {
            sum += (x[i] - y[i]) * (x[i] - y[i]);
        }
        return std::sqrt(sum);
    };
    const Mat3 first = normalize(a);
    const Mat3 second = normalize(b);
    Mat3 negated = second;
    for (double& value : negated) {
        value = -value;
    }
    return std::min(distance(first, second), distance(first, negated));
}

int countTrueInliers(const std::vector<bool>& inliers) {
    return static_cast<int>(std::count(inliers.begin(), inliers.end(), true));
}

}  // namespace

TEST(LinearAlgebra, SvdReconstructsTheInput) {
    const Mat3 matrix{0.3, -0.7, 0.2, 1.1, 0.4, -0.5, 0.9, 0.1, 0.6};
    Mat3 u{};
    Mat3 v{};
    std::array<double, 3> singular_values{};
    ASSERT_TRUE(scand3d::geometry::svd3(matrix, u, singular_values, v));

    const Mat3 reconstructed = scand3d::geometry::composeFromSvd(u, singular_values, v);
    double error = 0.0;
    for (std::size_t i = 0; i < matrix.size(); ++i) {
        error += std::fabs(matrix[i] - reconstructed[i]);
    }
    EXPECT_LT(error, 1e-9);
    EXPECT_GE(singular_values[0], singular_values[1]);
    EXPECT_GE(singular_values[1], singular_values[2]);
}

TEST(LinearAlgebra, InverseUndoesACalibrationMatrix) {
    const Mat3 k = calibrationMatrix(makeCamera({0.0, 0.0, 2.0}, {0.0, 0.0, 0.0}));
    const Mat3 product = scand3d::geometry::matmul3(k, scand3d::geometry::inverse3(k));
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            EXPECT_NEAR(row == col ? 1.0 : 0.0, product[row * 3 + col], 1e-12);
        }
    }
}

TEST(TwoViewGeometry, GroundTruthMatrixExplainsTheCorrespondences) {
    const Dataset clean = makeDataset(0.0, 0.0);
    for (const auto& point : clean.points) {
        EXPECT_LT(scand3d::geometry::sampsonDistancePx(clean.fundamental, point), 1e-6);
    }

    const Dataset noisy = makeDataset();
    std::vector<double> inlier_distances;
    for (std::size_t i = 0; i < noisy.points.size(); ++i) {
        if (noisy.inlier[i]) {
            inlier_distances.push_back(
                scand3d::geometry::sampsonDistancePx(noisy.fundamental, noisy.points[i]));
        }
    }
    std::sort(inlier_distances.begin(), inlier_distances.end());
    ASSERT_FALSE(inlier_distances.empty());
    const double median = inlier_distances[inlier_distances.size() / 2];
    EXPECT_LT(median, 0.5) << "the ground-truth matrix itself should fit the noisy inliers";

    int within_threshold = 0;
    for (const double distance : inlier_distances) {
        if (distance < 1.5) {
            ++within_threshold;
        }
    }
    EXPECT_GT(within_threshold, 25) << "dataset: " << noisy.points.size() << " points, "
                                    << inlier_distances.size() << " true inliers";
}

TEST(TwoViewGeometry, FundamentalMatrixSurvivesOutliers) {
    const Dataset data = makeDataset();
    ASSERT_GT(data.points.size(), 40u);
    ASSERT_GT(countTrueInliers(data.inlier), 25);

    RansacOptions options;
    options.threshold_px = 1.5;
    const auto result = scand3d::geometry::estimateFundamentalRansac(data.points, options);
    ASSERT_TRUE(result.valid);

    std::vector<bool> reported(data.points.size(), false);
    for (const int index : result.inlier_indices) {
        reported[static_cast<std::size_t>(index)] = true;
    }
    int found = 0;
    int false_positives = 0;
    for (std::size_t i = 0; i < data.points.size(); ++i) {
        if (data.inlier[i] && reported[i]) {
            ++found;
        }
        if (!data.inlier[i] && reported[i]) {
            ++false_positives;
        }
    }

    const int true_inliers = countTrueInliers(data.inlier);
    EXPECT_GE(static_cast<double>(found) / true_inliers, 0.95);
    EXPECT_LE(false_positives, 2);
    EXPECT_LT(result.median_sampson_px, 0.5);
    EXPECT_LT(matrixDistanceUpToScale(result.matrix, data.fundamental), 0.15);
}

TEST(TwoViewGeometry, EssentialMatrixRecoversTheCalibratedGeometry) {
    const Dataset data = makeDataset();

    RansacOptions options;
    options.threshold_px = 1.5;
    const auto result = scand3d::geometry::estimateEssentialRansac(
        data.points, data.first_intrinsics, data.second_intrinsics, options);
    ASSERT_TRUE(result.valid);

    EXPECT_LT(matrixDistanceUpToScale(result.matrix, data.essential), 0.15);
    EXPECT_LT(result.median_sampson_px, 0.5);

    Mat3 u{};
    Mat3 v{};
    std::array<double, 3> singular_values{};
    ASSERT_TRUE(scand3d::geometry::svd3(result.matrix, u, singular_values, v));
    EXPECT_NEAR(singular_values[0], singular_values[1], 1e-6);
    EXPECT_NEAR(0.0, singular_values[2], 1e-6);
}

TEST(TwoViewGeometry, ComputesTheSameEssentialFromThePixelMatrix) {
    const Dataset data = makeDataset(0.0, 0.0);

    RansacOptions options;
    options.threshold_px = 1.5;
    const auto fundamental =
        scand3d::geometry::estimateFundamentalRansac(data.points, options);
    ASSERT_TRUE(fundamental.valid);

    const Mat3 essential = scand3d::geometry::essentialFromFundamental(
        fundamental.matrix, data.first_intrinsics, data.second_intrinsics);
    EXPECT_LT(matrixDistanceUpToScale(essential, data.essential), 0.05);
}

TEST(TwoViewGeometry, RejectsTooFewCorrespondences) {
    std::vector<PointCorrespondence> points(7, {{0.0, 0.0}, {1.0, 1.0}});
    EXPECT_FALSE(scand3d::geometry::estimateFundamentalRansac(points).valid);
}
