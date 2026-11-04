#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>
#include <utility>
#include <vector>

#include "feature/feature_extractor.hpp"
#include "feature/feature_store.hpp"
#include "feature/gray_image.hpp"
#include "feature/matcher.hpp"
#include "geometry/two_view.hpp"
#include "synthetic/synthetic_scene.hpp"

namespace {

using scand3d::feature::Descriptor;
using scand3d::feature::FeatureSet;
using scand3d::feature::GrayImage;
using scand3d::feature::Keypoint;
using scand3d::feature::MatchOptions;

GrayImage toGray(const scand3d::synthetic::Image& image) {
    return {image.width_px, image.height_px, image.pixels};
}

/// Exact 2x2 area downscale: same content at half resolution, so a correct match has to survive
/// the pyramid rather than any change in how the scene was rendered.
GrayImage downscaleHalf(const GrayImage& image) {
    GrayImage out;
    out.width_px = image.width_px / 2;
    out.height_px = image.height_px / 2;
    out.pixels.assign(static_cast<std::size_t>(out.width_px) * static_cast<std::size_t>(out.height_px),
                      0);
    for (int y = 0; y < out.height_px; ++y) {
        for (int x = 0; x < out.width_px; ++x) {
            const std::size_t i0 = static_cast<std::size_t>(2 * y) *
                                       static_cast<std::size_t>(image.width_px) +
                                   static_cast<std::size_t>(2 * x);
            const int sum = image.pixels[i0] + image.pixels[i0 + 1] +
                            image.pixels[i0 + static_cast<std::size_t>(image.width_px)] +
                            image.pixels[i0 + static_cast<std::size_t>(image.width_px) + 1];
            out.pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width_px) +
                       static_cast<std::size_t>(x)] = static_cast<std::uint8_t>(sum / 4);
        }
    }
    return out;
}

scand3d::synthetic::Scene texturedScene(int width_px = 640, int height_px = 480,
                                        double focal_px = 500.0) {
    return scand3d::synthetic::makeTexturedBoxScene(12, 1.5, 0.45, width_px, height_px, focal_px,
                                                    12, 20261104u);
}

/// Id of the projected surface vertex closest to a keypoint, or -1 when nothing is close enough.
int nearestVertexId(const std::vector<scand3d::synthetic::SurfaceVertex>& vertices, double x_px,
                    double y_px, double max_distance_px) {
    int best_id = -1;
    double best_distance = max_distance_px;
    for (const auto& vertex : vertices) {
        const double distance =
            std::hypot(vertex.projection.u_px - x_px, vertex.projection.v_px - y_px);
        if (distance < best_distance) {
            best_distance = distance;
            best_id = vertex.id;
        }
    }
    return best_id;
}

}  // namespace

TEST(FeatureDetector, FindsManyCornersInsideTheImage) {
    const auto scene = texturedScene();
    const GrayImage image = toGray(scand3d::synthetic::render(scene, 0));
    const FeatureSet features = scand3d::feature::extractFeatures(image);

    ASSERT_GT(features.size(), 100u);
    EXPECT_EQ(features.keypoints.size(), features.descriptors.size());

    for (const Keypoint& keypoint : features.keypoints) {
        EXPECT_GE(keypoint.x_px, 0.0);
        EXPECT_GE(keypoint.y_px, 0.0);
        EXPECT_LT(keypoint.x_px, static_cast<double>(image.width_px));
        EXPECT_LT(keypoint.y_px, static_cast<double>(image.height_px));
        EXPECT_GE(keypoint.scale, 1.0);
        EXPECT_TRUE(std::isfinite(keypoint.angle_rad));
    }
}

TEST(FeaturePyramid, BuildsLevelsThatHalveTheResolution) {
    const auto scene = texturedScene();
    const GrayImage image = toGray(scand3d::synthetic::render(scene, 0));

    const auto pyramid = scand3d::feature::buildPyramid(image, 3);
    ASSERT_EQ(3u, pyramid.size());
    EXPECT_EQ(image.width_px, pyramid[0].image.width_px);
    EXPECT_NEAR(1.0, pyramid[0].scale, 1e-12);
    EXPECT_EQ(image.width_px / 2, pyramid[1].image.width_px);
    EXPECT_NEAR(0.5, pyramid[1].scale, 1e-12);
}

TEST(FeatureMatching, RatioTestMatchesAgreeWithKnown3DPoints) {
    const auto scene = texturedScene();
    const FeatureSet first =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 0)));
    const FeatureSet second =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 1)));

    const auto matches = scand3d::feature::matchBruteForce(first, second, MatchOptions{80, false});
    ASSERT_GT(matches.size(), 40u);

    const auto ground_truth_first = scand3d::synthetic::visibleSurfaceVertices(scene, 0);
    const auto ground_truth_second = scand3d::synthetic::visibleSurfaceVertices(scene, 1);

    int mapped = 0;
    int correct = 0;
    for (const auto& match : matches) {
        const auto& a = first.keypoints[static_cast<std::size_t>(match.query_index)];
        const auto& b = second.keypoints[static_cast<std::size_t>(match.train_index)];
        const int id_a = nearestVertexId(ground_truth_first, a.x_px, a.y_px, 3.0);
        const int id_b = nearestVertexId(ground_truth_second, b.x_px, b.y_px, 3.0);
        if (id_a < 0 || id_b < 0) {
            continue;  // the keypoint is not a grid corner, so it has no ground-truth id
        }
        ++mapped;
        if (id_a == id_b) {
            ++correct;
        }
    }

    ASSERT_GT(mapped, 25);
    const double correctness = static_cast<double>(correct) / static_cast<double>(mapped);
    // Raw descriptor matching is not expected to be outlier free: on a flat-shaded object many
    // corner patches look alike. RANSAC is the precision filter (roadmap 3.4), and
    // FeaturePipeline.MatchedInliersAgreeWithKnown3DPoints checks the filtered set.
    EXPECT_GE(correctness, 0.5) << "correct " << correct << " of " << mapped << " mapped matches";
}

TEST(FeatureMatching, ApproximateMatchesAgreeWithExhaustive) {
    const auto scene = texturedScene();
    const FeatureSet first =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 0)));
    const FeatureSet second =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 1)));

    const auto exhaustive =
        scand3d::feature::matchBruteForce(first, second, MatchOptions{80, false});
    const auto approximate =
        scand3d::feature::matchApproximate(first, second, MatchOptions{80, false});
    ASSERT_FALSE(exhaustive.empty());
    ASSERT_FALSE(approximate.empty());

    std::set<std::pair<int, int>> exhaustive_pairs;
    std::set<std::pair<int, int>> approximate_pairs;
    for (const auto& match : exhaustive) {
        exhaustive_pairs.emplace(match.query_index, match.train_index);
    }
    for (const auto& match : approximate) {
        approximate_pairs.emplace(match.query_index, match.train_index);
    }

    int shared = 0;
    for (const auto& pair : exhaustive_pairs) {
        if (approximate_pairs.count(pair) != 0) {
            ++shared;
        }
    }
    const double recall = static_cast<double>(shared) / static_cast<double>(exhaustive_pairs.size());
    EXPECT_GE(recall, 0.85);
}

TEST(FeatureDetector, MatchingSurvivesAChangeOfResolution) {
    const auto large_scene = texturedScene(640, 480, 500.0);
    const GrayImage large_image = toGray(scand3d::synthetic::render(large_scene, 0));
    const GrayImage small_image = downscaleHalf(large_image);

    const FeatureSet large = scand3d::feature::extractFeatures(large_image);
    const FeatureSet small = scand3d::feature::extractFeatures(small_image);

    const auto matches = scand3d::feature::matchBruteForce(small, large, MatchOptions{80, false});
    ASSERT_GT(matches.size(), 15u);

    // The descriptor patch is a fixed pixel footprint, so a half-resolution view matches the
    // pyramid level of the full-resolution one; only those pairs are expected to line up.
    int evaluated = 0;
    int consistent = 0;
    for (const auto& match : matches) {
        const auto& a = small.keypoints[static_cast<std::size_t>(match.query_index)];
        const auto& b = large.keypoints[static_cast<std::size_t>(match.train_index)];
        if (b.scale < 1.9) {
            continue;
        }
        ++evaluated;
        if (std::hypot(a.x_px * 2.0 - b.x_px, a.y_px * 2.0 - b.y_px) < 3.0) {
            ++consistent;
        }
    }
    ASSERT_GT(evaluated, 10);
    // Measured ~79%: the descriptor patch is a fixed pixel footprint, so only the pyramid level
    // that matches the other view's scale lines up. This is tracked as a limitation, not a claim
    // of scale invariance.
    EXPECT_GE(static_cast<double>(consistent) / static_cast<double>(evaluated), 0.75);
}

TEST(FeaturePipeline, MatchedInliersAgreeWithKnown3DPoints) {
    const auto scene = texturedScene();
    const FeatureSet first =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 0)));
    const FeatureSet second =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 1)));

    const auto matches = scand3d::feature::matchBruteForce(first, second, MatchOptions{80, false});
    ASSERT_GT(matches.size(), 20u);

    std::vector<scand3d::geometry::PointCorrespondence> points;
    points.reserve(matches.size());
    for (const auto& match : matches) {
        const auto& a = first.keypoints[static_cast<std::size_t>(match.query_index)];
        const auto& b = second.keypoints[static_cast<std::size_t>(match.train_index)];
        points.push_back({{a.x_px, a.y_px}, {b.x_px, b.y_px}});
    }

    scand3d::geometry::RansacOptions options;
    options.threshold_px = 3.0;  // descriptor corners are integer pixel positions
    const auto result = scand3d::geometry::estimateFundamentalRansac(points, options);
    ASSERT_TRUE(result.valid);
    ASSERT_GE(result.inlier_indices.size(), 15u);

    const auto ground_truth_first = scand3d::synthetic::visibleSurfaceVertices(scene, 0);
    const auto ground_truth_second = scand3d::synthetic::visibleSurfaceVertices(scene, 1);

    int mapped = 0;
    int correct = 0;
    for (const int index : result.inlier_indices) {
        const auto& point = points[static_cast<std::size_t>(index)];
        const int id_a = nearestVertexId(ground_truth_first, point.first[0], point.first[1], 4.0);
        const int id_b = nearestVertexId(ground_truth_second, point.second[0], point.second[1], 4.0);
        if (id_a < 0 || id_b < 0) {
            continue;
        }
        ++mapped;
        if (id_a == id_b) {
            ++correct;
        }
    }

    ASSERT_GT(mapped, 10);
    EXPECT_GE(static_cast<double>(correct) / static_cast<double>(mapped), 0.95)
        << "correct " << correct << " of " << mapped << " verified inliers";
}

TEST(FeatureMatching, EdgeDirectionBreaksADescriptorTie) {
    // Two identical descriptors on the same contour: only the gradient direction says which one a
    // query belongs to. This is the ambiguity that made contour points cost precision before the
    // matcher used the direction (D054).
    FeatureSet query;
    query.keypoints = {Keypoint{0.0, 0.0, 1.0, 0.0, 1.0, 0.0, true}};
    query.descriptors = {Descriptor{}};

    FeatureSet train;
    train.keypoints = {Keypoint{10.0, 10.0, 1.0, 0.0, 1.0, 1.48, true},
                       Keypoint{20.0, 20.0, 1.0, 0.0, 1.0, 0.09, true}};
    train.descriptors = {Descriptor{}, Descriptor{}};

    // Without the direction the two candidates are indistinguishable, so the ratio test rejects.
    EXPECT_TRUE(scand3d::feature::matchBruteForce(query, train, MatchOptions{80, false, 0}).empty());

    // With it the agreeing direction wins the tie and the match survives.
    const auto matches = scand3d::feature::matchBruteForce(query, train, MatchOptions{80, false, 8});
    ASSERT_EQ(1u, matches.size());
    EXPECT_EQ(1, matches[0].train_index);

    const auto approximate =
        scand3d::feature::matchApproximate(query, train, MatchOptions{80, false, 8});
    ASSERT_EQ(1u, approximate.size());
    EXPECT_EQ(1, approximate[0].train_index);
}

TEST(FeatureMatching, RatioTestRejectsAmbiguousDescriptors) {
    FeatureSet query;
    query.keypoints = {{0.0, 0.0, 1.0, 0.0, 1.0}};
    query.descriptors = {Descriptor{}};

    FeatureSet train;
    train.keypoints = {{0.0, 0.0, 1.0, 0.0, 1.0}, {1.0, 1.0, 1.0, 0.0, 1.0}};
    // Two identical candidates: nothing distinguishes them, so the ratio test must reject.
    train.descriptors = {Descriptor{}, Descriptor{}};

    EXPECT_TRUE(scand3d::feature::matchBruteForce(query, train, MatchOptions{80, false}).empty());
    EXPECT_TRUE(scand3d::feature::matchApproximate(query, train, MatchOptions{80, false}).empty());

    Descriptor far{};
    far.fill(0xFF);
    train.descriptors[1] = far;
    EXPECT_EQ(1u, scand3d::feature::matchBruteForce(query, train, MatchOptions{80, false}).size());
}

TEST(FeatureStore, RoundTripsKeypointsAndDescriptors) {
    const auto scene = texturedScene();
    const FeatureSet features =
        scand3d::feature::extractFeatures(toGray(scand3d::synthetic::render(scene, 0)));
    ASSERT_FALSE(features.empty());

    const auto path =
        (std::filesystem::temp_directory_path() / "scand3d-feature-roundtrip.bin").string();
    ASSERT_TRUE(scand3d::feature::saveFeatures(path, features));

    FeatureSet loaded;
    ASSERT_TRUE(scand3d::feature::loadFeatures(path, loaded));
    ASSERT_EQ(features.keypoints.size(), loaded.keypoints.size());
    for (std::size_t i = 0; i < features.keypoints.size(); ++i) {
        EXPECT_FLOAT_EQ(static_cast<float>(features.keypoints[i].x_px),
                        static_cast<float>(loaded.keypoints[i].x_px));
        EXPECT_FLOAT_EQ(static_cast<float>(features.keypoints[i].response),
                        static_cast<float>(loaded.keypoints[i].response));
        EXPECT_EQ(features.descriptors[i], loaded.descriptors[i]);
    }

    std::filesystem::remove(path);
    EXPECT_FALSE(scand3d::feature::loadFeatures(path, loaded));
}

TEST(FeatureStore, RejectsATruncatedFile) {
    const auto path =
        (std::filesystem::temp_directory_path() / "scand3d-feature-truncated.bin").string();
    {
        std::FILE* file = std::fopen(path.c_str(), "wb");
        ASSERT_NE(nullptr, file);
        const char junk[] = "S3DFEAT1";
        std::fwrite(junk, 1, sizeof(junk) - 1, file);
        std::fclose(file);
    }

    FeatureSet loaded;
    EXPECT_FALSE(scand3d::feature::loadFeatures(path, loaded));
    std::filesystem::remove(path);
}
