#include "geometry/two_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace scand3d::geometry {
namespace {

Mat3 intrinsicMatrix(const CameraIntrinsics& intrinsics) {
    return {intrinsics.focal_px, 0.0, intrinsics.cx_px,
            0.0, intrinsics.focal_px, intrinsics.cy_px,
            0.0, 0.0, 1.0};
}

/// Hartley normalization: centroid at the origin, mean distance sqrt(2).
Mat3 normalizePoints(const std::vector<Vec2>& points, std::vector<Vec2>& normalized) {
    normalized.clear();
    if (points.empty()) {
        return identity3();
    }
    double centre_x = 0.0;
    double centre_y = 0.0;
    for (const Vec2& point : points) {
        centre_x += point[0];
        centre_y += point[1];
    }
    centre_x /= static_cast<double>(points.size());
    centre_y /= static_cast<double>(points.size());

    double mean_distance = 0.0;
    for (const Vec2& point : points) {
        mean_distance += std::hypot(point[0] - centre_x, point[1] - centre_y);
    }
    mean_distance /= static_cast<double>(points.size());

    const double scale = mean_distance > 1e-12 ? std::sqrt(2.0) / mean_distance : 1.0;
    normalized.reserve(points.size());
    for (const Vec2& point : points) {
        normalized.push_back({scale * (point[0] - centre_x), scale * (point[1] - centre_y)});
    }
    return {scale, 0.0, -scale * centre_x,
            0.0, scale, -scale * centre_y,
            0.0, 0.0, 1.0};
}

/// Rank-2 matrix from eight or more correspondences.
bool estimateRank2(
    const std::vector<PointCorrespondence>& correspondences,
    Mat3& matrix,
    bool equalize_singular_values) {
    if (correspondences.size() < 8) {
        return false;
    }

    std::vector<Vec2> first;
    std::vector<Vec2> second;
    first.reserve(correspondences.size());
    second.reserve(correspondences.size());
    for (const PointCorrespondence& correspondence : correspondences) {
        first.push_back(correspondence.first);
        second.push_back(correspondence.second);
    }

    std::vector<Vec2> normalized_first;
    std::vector<Vec2> normalized_second;
    const Mat3 first_transform = normalizePoints(first, normalized_first);
    const Mat3 second_transform = normalizePoints(second, normalized_second);

    // A^T A for the epipolar constraint [x2, y2, 1] F [x1, y1, 1]^T = 0.
    std::array<double, 81> normal_equations{};
    for (std::size_t i = 0; i < normalized_first.size(); ++i) {
        const double x1 = normalized_first[i][0];
        const double y1 = normalized_first[i][1];
        const double x2 = normalized_second[i][0];
        const double y2 = normalized_second[i][1];
        const std::array<double, 9> row{x2 * x1, x2 * y1, x2, y2 * x1, y2 * y1, y2, x1, y1, 1.0};
        for (int a = 0; a < 9; ++a) {
            for (int b = 0; b < 9; ++b) {
                normal_equations[static_cast<std::size_t>(a * 9 + b)] +=
                    row[static_cast<std::size_t>(a)] * row[static_cast<std::size_t>(b)];
            }
        }
    }

    std::array<double, 9> vector{};
    if (!smallestEigenvector9(normal_equations, vector)) {
        return false;
    }

    Mat3 fundamental{vector[0], vector[1], vector[2],
                     vector[3], vector[4], vector[5],
                     vector[6], vector[7], vector[8]};
    fundamental = matmul3(transpose3(second_transform), matmul3(fundamental, first_transform));

    Mat3 u{};
    Mat3 v{};
    std::array<double, 3> singular_values{};
    if (!svd3(fundamental, u, singular_values, v)) {
        return false;
    }
    if (equalize_singular_values) {
        const double average = 0.5 * (singular_values[0] + singular_values[1]);
        singular_values = {average, average, 0.0};
    } else {
        singular_values[2] = 0.0;
    }

    fundamental = composeFromSvd(u, singular_values, v);
    const double norm = frobeniusNorm3(fundamental);
    if (!(norm > 1e-12) || !std::isfinite(norm)) {
        return false;
    }
    for (double& value : fundamental) {
        value /= norm;
    }
    matrix = fundamental;
    return true;
}

double median(std::vector<double> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle),
                     values.end());
    double upper = values[middle];
    if (values.size() % 2 == 1) {
        return upper;
    }
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle - 1),
                     values.end());
    const double lower = values[middle - 1];
    return 0.5 * (lower + upper);
}

TwoViewResult ransac(
    const std::vector<PointCorrespondence>& correspondences,
    double threshold,
    const RansacOptions& options,
    bool equalize_singular_values) {
    TwoViewResult result;
    if (correspondences.size() < 8 || !(threshold > 0.0)) {
        return result;
    }

    std::mt19937 random(options.seed);
    std::uniform_int_distribution<int> pick(
        0, static_cast<int>(correspondences.size()) - 1);

    // Stratified sampling. Eight points drawn from one face of an object are a planar sample, and a
    // planar sample produces a model that explains that face while rejecting the rest of the image;
    // on a box that is most of the image, so the inlier count collapses. Requiring the sample to
    // span the image keeps the minimal set non-degenerate (D038).
    double min_x = correspondences[0].first[0];
    double max_x = min_x;
    double min_y = correspondences[0].first[1];
    double max_y = min_y;
    for (const PointCorrespondence& correspondence : correspondences) {
        min_x = std::min(min_x, correspondence.first[0]);
        max_x = std::max(max_x, correspondence.first[0]);
        min_y = std::min(min_y, correspondence.first[1]);
        max_y = std::max(max_y, correspondence.first[1]);
    }
    const double span_x = std::max(max_x - min_x, 1e-9);
    const double span_y = std::max(max_y - min_y, 1e-9);
    const auto cellOf = [min_x, min_y, span_x, span_y, &correspondences](int index) {
        const Vec2& point = correspondences[static_cast<std::size_t>(index)].first;
        const int cell_x = std::clamp(static_cast<int>(3.0 * (point[0] - min_x) / span_x), 0, 2);
        const int cell_y = std::clamp(static_cast<int>(3.0 * (point[1] - min_y) / span_y), 0, 2);
        return cell_x * 3 + cell_y;
    };

    Mat3 best{};
    std::vector<int> best_inliers;
    int iteration_limit = std::max(1, options.max_iterations);
    int iteration = 0;

    while (iteration < iteration_limit) {
        ++iteration;

        std::array<int, 8> sample{};
        bool unique = false;
        int distinct_cells = 0;
        for (int attempt = 0; attempt < 100 && (!unique || distinct_cells < 3); ++attempt) {
            for (int& index : sample) {
                index = pick(random);
            }
            std::array<int, 8> sorted = sample;
            std::sort(sorted.begin(), sorted.end());
            unique = std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end();
            if (!unique) {
                continue;
            }
            std::array<bool, 9> cells{};
            for (const int index : sample) {
                cells[static_cast<std::size_t>(cellOf(index))] = true;
            }
            distinct_cells = static_cast<int>(std::count(cells.begin(), cells.end(), true));
        }
        if (!unique || distinct_cells < 3) {
            continue;
        }

        std::vector<PointCorrespondence> subset;
        subset.reserve(8);
        for (const int index : sample) {
            subset.push_back(correspondences[static_cast<std::size_t>(index)]);
        }

        Mat3 candidate{};
        if (!estimateRank2(subset, candidate, equalize_singular_values)) {
            continue;
        }

        std::vector<int> inliers;
        inliers.reserve(correspondences.size());
        for (std::size_t i = 0; i < correspondences.size(); ++i) {
            if (sampsonDistancePx(candidate, correspondences[i]) < threshold) {
                inliers.push_back(static_cast<int>(i));
            }
        }

        if (inliers.size() > best_inliers.size()) {
            best = candidate;
            best_inliers = std::move(inliers);

            // Adaptive stopping: how many samples a set this clean needs at the given confidence.
            const double inlier_ratio =
                static_cast<double>(best_inliers.size()) / static_cast<double>(correspondences.size());
            if (inlier_ratio > 0.0 && inlier_ratio < 1.0) {
                const double ratio_to_eight = std::pow(inlier_ratio, 8.0);
                if (ratio_to_eight > 0.0 && ratio_to_eight < 1.0) {
                    const double required =
                        std::log(1.0 - options.confidence) / std::log(1.0 - ratio_to_eight);
                    // Only shorten the budget, and compare as double: with a weak inlier ratio the
                    // estimate is astronomically large and would overflow an int cast.
                    if (std::isfinite(required) && required > 0.0 &&
                        required < static_cast<double>(iteration_limit)) {
                        iteration_limit =
                            std::max(iteration, static_cast<int>(std::ceil(required)));
                    }
                }
            }
        }
    }

    if (best_inliers.size() < 8) {
        return result;
    }

    // The 8-point estimate from a minimal sample is noisy. Alternate refitting on all inliers and
    // reclassifying until the inlier set settles: one refit leaves a visible bias, and the matrix
    // that is returned must be the one its reported inliers actually support.
    std::vector<int> current_inliers = best_inliers;
    Mat3 current_matrix = best;
    for (int round = 0; round < 4; ++round) {
        std::vector<PointCorrespondence> inlier_points;
        inlier_points.reserve(current_inliers.size());
        for (const int index : current_inliers) {
            inlier_points.push_back(correspondences[static_cast<std::size_t>(index)]);
        }

        Mat3 refined{};
        if (!estimateRank2(inlier_points, refined, equalize_singular_values)) {
            break;
        }

        std::vector<int> next_inliers;
        next_inliers.reserve(correspondences.size());
        for (std::size_t i = 0; i < correspondences.size(); ++i) {
            if (sampsonDistancePx(refined, correspondences[i]) < threshold) {
                next_inliers.push_back(static_cast<int>(i));
            }
        }
        if (next_inliers.size() < 8) {
            break;
        }

        const bool settled = next_inliers == current_inliers;
        current_matrix = refined;
        current_inliers = std::move(next_inliers);
        if (settled) {
            break;
        }
    }

    if (current_inliers.size() < 8) {
        return result;
    }

    std::vector<double> distances;
    distances.reserve(current_inliers.size());
    for (const int index : current_inliers) {
        distances.push_back(
            sampsonDistancePx(current_matrix, correspondences[static_cast<std::size_t>(index)]));
    }

    result.matrix = current_matrix;
    result.inlier_indices = std::move(current_inliers);
    result.median_sampson_px = median(std::move(distances));
    result.valid = true;
    return result;
}

}  // namespace

double sampsonDistancePx(const Mat3& matrix, const PointCorrespondence& correspondence) {
    const Vec3 first{correspondence.first[0], correspondence.first[1], 1.0};
    const Vec3 second{correspondence.second[0], correspondence.second[1], 1.0};
    const Vec3 first_epipolar = matvec3(matrix, first);
    const Vec3 second_epipolar = matvec3(transpose3(matrix), second);

    const double numerator = dot3(second, first_epipolar);
    const double denominator = first_epipolar[0] * first_epipolar[0] +
                               first_epipolar[1] * first_epipolar[1] +
                               second_epipolar[0] * second_epipolar[0] +
                               second_epipolar[1] * second_epipolar[1];
    if (!(denominator > 1e-18)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::fabs(numerator) / std::sqrt(denominator);
}

TwoViewResult estimateFundamentalRansac(
    const std::vector<PointCorrespondence>& correspondences,
    const RansacOptions& options) {
    return ransac(correspondences, options.threshold_px, options, false);
}

TwoViewResult estimateEssentialRansac(
    const std::vector<PointCorrespondence>& correspondences,
    const CameraIntrinsics& first,
    const CameraIntrinsics& second,
    const RansacOptions& options) {
    if (!(first.focal_px > 0.0) || !(second.focal_px > 0.0)) {
        return {};
    }

    std::vector<PointCorrespondence> calibrated;
    calibrated.reserve(correspondences.size());
    for (const PointCorrespondence& correspondence : correspondences) {
        calibrated.push_back({
            {(correspondence.first[0] - first.cx_px) / first.focal_px,
             (correspondence.first[1] - first.cy_px) / first.focal_px},
            {(correspondence.second[0] - second.cx_px) / second.focal_px,
             (correspondence.second[1] - second.cy_px) / second.focal_px},
        });
    }

    // The threshold is in pixels, so convert it into the calibrated image plane.
    const double mean_focal = std::sqrt(first.focal_px * second.focal_px);
    const double calibrated_threshold = options.threshold_px / mean_focal;
    TwoViewResult result = ransac(calibrated, calibrated_threshold, options, true);

    // On a mostly planar surface the equal-singular-value fit is poorly conditioned and can keep
    // only a small fraction of the correct matches, while the unconstrained fundamental fit keeps
    // nearly all of them. When that happens, estimate F in pixels and calibrate it: the essential
    // matrix is E = K2^T F K1 by definition, so nothing is approximated (D038).
    if (!result.valid || result.inlier_indices.size() * 2 < correspondences.size()) {
        const TwoViewResult pixel_result =
            ransac(correspondences, options.threshold_px, options, false);
        if (pixel_result.valid) {
            const Mat3 essential = essentialFromFundamental(pixel_result.matrix, first, second);
            std::vector<int> inliers;
            std::vector<double> distances;
            for (std::size_t i = 0; i < calibrated.size(); ++i) {
                const double distance = sampsonDistancePx(essential, calibrated[i]);
                if (distance < calibrated_threshold) {
                    inliers.push_back(static_cast<int>(i));
                    distances.push_back(distance);
                }
            }
            if (inliers.size() >= 8 && inliers.size() > result.inlier_indices.size()) {
                // Scaled once below, with the other path's median.
                result.median_sampson_px = median(std::move(distances));
                result.inlier_indices = std::move(inliers);
                result.matrix = essential;
                result.valid = true;
            }
        }
    }

    if (result.valid) {
        result.median_sampson_px *= mean_focal;
    }
    return result;
}

Mat3 essentialFromFundamental(
    const Mat3& fundamental,
    const CameraIntrinsics& first,
    const CameraIntrinsics& second) {
    const Mat3 essential =
        matmul3(transpose3(intrinsicMatrix(second)), matmul3(fundamental, intrinsicMatrix(first)));
    const double norm = frobeniusNorm3(essential);
    if (!(norm > 1e-15)) {
        return essential;
    }
    Mat3 scaled = essential;
    for (double& value : scaled) {
        value /= norm;
    }
    return scaled;
}

}  // namespace scand3d::geometry
