#include "sfm/session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "feature/matcher.hpp"
#include "geometry/two_view.hpp"
#include "sfm/bundle_adjustment.hpp"
#include "sfm/keyframe_selection.hpp"
#include "sfm/pnp.hpp"
#include "sfm/pose.hpp"
#include "sfm/report.hpp"
#include "sfm/tracker.hpp"
#include "sfm/triangulation.hpp"

namespace scand3d::sfm {
namespace {

// Pairs below this many matches carry no useful baseline and only cost a RANSAC round.
constexpr int kMinPairMatches = 12;

// Capture is sequential, so only neighbouring views are worth matching.
constexpr int kMaxViewGap = 2;

// Seed pairs evaluated before the best-scoring ones are grown.
constexpr int kMaxSeedAttempts = 16;

// Growing costs a PnP pass per candidate view, so only the best few seeds are grown.
constexpr int kMaxGrowAttempts = 4;

// A neighbour needs at least this many correspondences before PnP can place it.
constexpr int kMinPnpCorrespondences = 6;

constexpr double kMinParallaxRad = 0.017453292519943295;  // 1 degree
constexpr double kOutlierReprojectionPx = 4.0;

std::uint64_t featureKey(int view, int feature) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(view)) << 32) |
           static_cast<std::uint32_t>(feature);
}

feature::MatchOptions neighbourMatchOptions() {
    feature::MatchOptions options;
    options.ratio_percent = 80;
    options.cross_check = true;
    return options;
}

geometry::CameraIntrinsics intrinsicsOf(const CameraModel& camera) {
    return {camera.focal_px, camera.cx_px, camera.cy_px};
}

/// Drops observations that reproject far off their point and retires points left with one view.
void prune(Reconstruction& reconstruction) {
    const double limit = kOutlierReprojectionPx * kOutlierReprojectionPx;
    for (MapPoint& point : reconstruction.points) {
        if (!point.valid) {
            continue;
        }
        std::vector<Observation>& observations = point.observations;
        observations.erase(
            std::remove_if(observations.begin(), observations.end(),
                           [&reconstruction, &point, limit](const Observation& observation) {
                               const ViewState& view =
                                   reconstruction.views[static_cast<std::size_t>(observation.view)];
                               return squaredReprojectionPx(view.camera, view.pose, point.position_m,
                                                            observation.pixel) > limit;
                           }),
            observations.end());
        if (observations.size() < 2) {
            point.valid = false;
        }
    }
}

/// Feature -> point lookup over the valid points, rebuilt after every map change.
std::unordered_map<std::uint64_t, int> featureIndex(const Reconstruction& reconstruction) {
    std::unordered_map<std::uint64_t, int> index;
    for (std::size_t point_index = 0; point_index < reconstruction.points.size(); ++point_index) {
        const MapPoint& point = reconstruction.points[point_index];
        if (!point.valid) {
            continue;
        }
        for (const Observation& observation : point.observations) {
            index[featureKey(observation.view, observation.feature)] =
                static_cast<int>(point_index);
        }
    }
    return index;
}

int validPointCount(const Reconstruction& reconstruction) {
    int count = 0;
    for (const MapPoint& point : reconstruction.points) {
        if (point.valid) {
            ++count;
        }
    }
    return count;
}

int registeredCount(const Reconstruction& reconstruction) {
    int count = 0;
    for (const ViewState& view : reconstruction.views) {
        if (view.registered) {
            ++count;
        }
    }
    return count;
}

/// Chains a pose expressed in the first camera's frame onto a world pose of that camera.
Pose composePose(const Pose& world_from_first, const Pose& first_from_second) {
    Pose world;
    world.rotation = geometry::matmul3(world_from_first.rotation, first_from_second.rotation);
    const Vec3 offset =
        geometry::matvec3(world_from_first.rotation, first_from_second.camera_centre_m);
    world.camera_centre_m = {world_from_first.camera_centre_m[0] + offset[0],
                             world_from_first.camera_centre_m[1] + offset[1],
                             world_from_first.camera_centre_m[2] + offset[2]};
    return world;
}

/// The feature of `view` in a match stored with the pair's own orientation.
int featureOf(const PairMatches& pair, int view, const std::pair<int, int>& match) {
    return pair.view_a == view ? match.first : match.second;
}

/// The feature of the *other* view in the same match.
int otherFeatureOf(const PairMatches& pair, int view, const std::pair<int, int>& match) {
    return pair.view_a == view ? match.second : match.first;
}

/// How many of a seed's points a neighbouring view could actually use for PnP. This decides which
/// seed is worth growing: a seed whose points no neighbour can match will never extend (D037).
int thirdViewSupport(
    const Reconstruction& reconstruction,
    const std::vector<PairMatches>& pairs,
    const PairMatches& seed) {
    const std::unordered_map<std::uint64_t, int> index = featureIndex(reconstruction);
    int best = 0;
    for (const int seed_view : {seed.view_a, seed.view_b}) {
        for (const PairMatches& pair : pairs) {
            int other = -1;
            if (pair.view_a == seed_view) {
                other = pair.view_b;
            } else if (pair.view_b == seed_view) {
                other = pair.view_a;
            }
            if (other < 0) {
                continue;
            }
            int matched = 0;
            for (const std::pair<int, int>& match : pair.matches) {
                const int theirs = featureOf(pair, seed_view, match);
                if (index.find(featureKey(seed_view, theirs)) != index.end()) {
                    ++matched;
                }
            }
            best = std::max(best, matched);
        }
    }
    return best;
}

}  // namespace

Session::Session(SfmOptions options, feature::FeatureParams feature_params)
    : options_(options), feature_params_(feature_params) {}

int Session::addView(
    int width_px,
    int height_px,
    const unsigned char* gray,
    double focal_px,
    double cx_px,
    double cy_px) {
    if (gray == nullptr || width_px <= 0 || height_px <= 0 || !(focal_px > 0.0)) {
        return -1;
    }

    feature::GrayImage image;
    image.width_px = width_px;
    image.height_px = height_px;
    image.pixels.assign(gray,
                        gray + static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px));

    feature::FeatureSet set = feature::extractFeatures(image, feature_params_);
    if (set.empty()) {
        return -1;
    }

    std::vector<Vec2> pixels;
    pixels.reserve(set.size());
    for (const feature::Keypoint& keypoint : set.keypoints) {
        pixels.push_back({keypoint.x_px, keypoint.y_px});
    }

    cameras_.push_back(CameraModel{focal_px, cx_px, cy_px});
    features_.push_back(std::move(set));
    pixels_.push_back(std::move(pixels));

    // A new view invalidates whatever was reconstructed from the previous set.
    result_ = SfmResult{};
    ran_ = false;
    return static_cast<int>(features_.size()) - 1;
}

int Session::viewCount() const { return static_cast<int>(features_.size()); }

ReconstructionReport Session::run() {
    result_ = SfmResult{};
    ran_ = false;
    if (features_.size() < 2) {
        return result_.report;
    }

    // Matching is pairwise and geometric: a pair is validated by an essential matrix before any of
    // its matches can create a 3D point, so a false match cannot drag a point off its epipolar line.
    const feature::MatchOptions match_options = neighbourMatchOptions();
    std::vector<PairMatches> pairs;
    for (std::size_t first = 0; first < features_.size(); ++first) {
        const std::size_t limit =
            std::min(features_.size(), first + static_cast<std::size_t>(kMaxViewGap) + 1);
        for (std::size_t second = first + 1; second < limit; ++second) {
            const std::vector<feature::FeatureMatch> matches =
                feature::matchCornersThenContours(
                    features_[first], features_[second], match_options,
                    static_cast<std::size_t>(kMinPairMatches));
            if (matches.size() < static_cast<std::size_t>(kMinPairMatches)) {
                continue;
            }

            std::vector<geometry::PointCorrespondence> correspondences;
            correspondences.reserve(matches.size());
            for (const feature::FeatureMatch& match : matches) {
                correspondences.push_back(
                    {pixels_[first][static_cast<std::size_t>(match.query_index)],
                     pixels_[second][static_cast<std::size_t>(match.train_index)]});
            }
            const geometry::TwoViewResult ransac = geometry::estimateEssentialRansac(
                correspondences, intrinsicsOf(cameras_[first]), intrinsicsOf(cameras_[second]),
                options_.two_view);
            if (!ransac.valid ||
                ransac.inlier_indices.size() < static_cast<std::size_t>(kMinPairMatches)) {
                continue;
            }

            PairMatches pair;
            pair.view_a = static_cast<int>(first);
            pair.view_b = static_cast<int>(second);
            pair.matches.reserve(ransac.inlier_indices.size());
            for (const int inlier : ransac.inlier_indices) {
                const feature::FeatureMatch& match = matches[static_cast<std::size_t>(inlier)];
                pair.matches.emplace_back(match.query_index, match.train_index);
            }
            pairs.push_back(std::move(pair));
        }
    }
    if (pairs.empty()) {
        return result_.report;
    }

    std::vector<ViewState> base_views(features_.size());
    for (std::size_t index = 0; index < features_.size(); ++index) {
        base_views[index].camera = cameras_[index];
        base_views[index].feature_count = pixels_[index].size();
    }

    std::vector<const PairMatches*> ordered;
    ordered.reserve(pairs.size());
    for (const PairMatches& pair : pairs) {
        ordered.push_back(&pair);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const PairMatches* a, const PairMatches* b) {
                         return a->matches.size() > b->matches.size();
                     });

    auto seedInto = [this](Reconstruction& reconstruction, const PairMatches& pair) {
        std::vector<geometry::PointCorrespondence> correspondences;
        correspondences.reserve(pair.matches.size());
        for (const std::pair<int, int>& match : pair.matches) {
            correspondences.push_back(
                {pixels_[static_cast<std::size_t>(pair.view_a)]
                          [static_cast<std::size_t>(match.first)],
                 pixels_[static_cast<std::size_t>(pair.view_b)]
                          [static_cast<std::size_t>(match.second)]});
        }
        const TwoViewInitialization initialization = initializeTwoView(
            correspondences, cameras_[static_cast<std::size_t>(pair.view_a)],
            cameras_[static_cast<std::size_t>(pair.view_b)], options_.two_view);
        if (!initialization.valid) {
            return false;
        }

        ViewState& first = reconstruction.views[static_cast<std::size_t>(pair.view_a)];
        first.pose = initialization.first;
        first.registered = true;
        ViewState& second = reconstruction.views[static_cast<std::size_t>(pair.view_b)];
        second.pose = initialization.second;
        second.registered = true;

        for (std::size_t index = 0; index < initialization.points.size(); ++index) {
            const std::pair<int, int>& match =
                pair.matches[static_cast<std::size_t>(initialization.source_index[index])];
            MapPoint point;
            point.position_m = initialization.points[index];
            point.observations = {
                {pair.view_a, match.first,
                 pixels_[static_cast<std::size_t>(pair.view_a)]
                        [static_cast<std::size_t>(match.first)]},
                {pair.view_b, match.second,
                 pixels_[static_cast<std::size_t>(pair.view_b)]
                        [static_cast<std::size_t>(match.second)]},
            };
            reconstruction.points.push_back(std::move(point));
        }
        return true;
    };

    // Registers views one at a time against the nearest registered neighbour: PnP when the candidate
    // sees enough of the map, then the matches that pose actually explains become points.
    auto grow = [this, &pairs](Reconstruction& reconstruction) {
        std::unordered_map<std::uint64_t, int> point_of_feature = featureIndex(reconstruction);
        bool progress = true;
        while (progress) {
            progress = false;
            for (int view = 0; view < static_cast<int>(features_.size()); ++view) {
                if (reconstruction.views[static_cast<std::size_t>(view)].registered) {
                    continue;
                }

                const PairMatches* pair = nullptr;
                int reference = -1;
                std::size_t best_matches = 0;
                for (int offset = -kMaxViewGap; offset <= kMaxViewGap; ++offset) {
                    if (offset == 0) {
                        continue;
                    }
                    const int other = view + offset;
                    if (other < 0 || other >= static_cast<int>(features_.size()) ||
                        !reconstruction.views[static_cast<std::size_t>(other)].registered) {
                        continue;
                    }
                    for (const PairMatches& candidate : pairs) {
                        const bool links_views =
                            (candidate.view_a == view && candidate.view_b == other) ||
                            (candidate.view_a == other && candidate.view_b == view);
                        if (links_views && candidate.matches.size() > best_matches) {
                            best_matches = candidate.matches.size();
                            pair = &candidate;
                            reference = other;
                        }
                    }
                }
                if (pair == nullptr) {
                    continue;
                }

                const ViewState& reference_view =
                    reconstruction.views[static_cast<std::size_t>(reference)];

                std::vector<Vec3> object_points;
                std::vector<Vec2> image_points;
                for (const std::pair<int, int>& match : pair->matches) {
                    const int mine = featureOf(*pair, view, match);
                    const int theirs = otherFeatureOf(*pair, view, match);
                    const auto found = point_of_feature.find(featureKey(reference, theirs));
                    if (found == point_of_feature.end()) {
                        continue;
                    }
                    object_points.push_back(
                        reconstruction.points[static_cast<std::size_t>(found->second)].position_m);
                    image_points.push_back(
                        pixels_[static_cast<std::size_t>(view)][static_cast<std::size_t>(mine)]);
                }

                Pose candidate_pose;
                bool have_pose = false;
                if (object_points.size() >= static_cast<std::size_t>(kMinPnpCorrespondences)) {
                    const PnpResult pnp = estimatePoseRansac(
                        object_points, image_points, cameras_[static_cast<std::size_t>(view)],
                        options_.pnp);
                    const double required =
                        std::max(6.0, options_.min_pnp_inlier_ratio *
                                          static_cast<double>(object_points.size()));
                    if (pnp.valid && static_cast<double>(pnp.inliers) >= required) {
                        candidate_pose = pnp.pose;
                        have_pose = true;
                    }
                }

                // A two-view map is far too thin for PnP. Capture is sequential, so the pair's own
                // relative pose, scaled by the depths of the map points the pair also sees, places
                // the view well enough to triangulate against; later views then see a real map
                // (D037).
                if (!have_pose) {
                    std::vector<geometry::PointCorrespondence> correspondences;
                    correspondences.reserve(pair->matches.size());
                    for (const std::pair<int, int>& match : pair->matches) {
                        correspondences.push_back(
                            {pixels_[static_cast<std::size_t>(reference)]
                                    [static_cast<std::size_t>(otherFeatureOf(*pair, view, match))],
                             pixels_[static_cast<std::size_t>(view)]
                                    [static_cast<std::size_t>(featureOf(*pair, view, match))]});
                    }
                    const TwoViewInitialization relative = initializeTwoView(
                        correspondences, cameras_[static_cast<std::size_t>(reference)],
                        cameras_[static_cast<std::size_t>(view)], options_.two_view);
                    if (!relative.valid || relative.inliers < 8) {
                        continue;
                    }

                    std::vector<double> scales;
                    for (std::size_t index = 0; index < relative.points.size(); ++index) {
                        const std::pair<int, int>& match =
                            pair->matches[static_cast<std::size_t>(relative.source_index[index])];
                        const auto found = point_of_feature.find(
                            featureKey(reference, otherFeatureOf(*pair, view, match)));
                        if (found == point_of_feature.end()) {
                            continue;
                        }
                        const Vec3 camera_point = worldToCamera(
                            reference_view.pose,
                            reconstruction.points[static_cast<std::size_t>(found->second)]
                                .position_m);
                        const double relative_depth = relative.points[index][2];
                        if (camera_point[2] > 1e-9 && relative_depth > 1e-9) {
                            scales.push_back(camera_point[2] / relative_depth);
                        }
                    }
                    if (scales.empty()) {
                        continue;
                    }
                    std::sort(scales.begin(), scales.end());
                    const double scale = scales[scales.size() / 2];
                    if (!(scale > 0.0) || !std::isfinite(scale)) {
                        continue;
                    }

                    Pose relative_pose = relative.second;
                    relative_pose.camera_centre_m = {relative_pose.camera_centre_m[0] * scale,
                                                     relative_pose.camera_centre_m[1] * scale,
                                                     relative_pose.camera_centre_m[2] * scale};
                    candidate_pose = composePose(reference_view.pose, relative_pose);
                    have_pose = true;
                }

                // Rotation and baseline windows keep near-duplicate frames out of the map.
                const KeyframeDecision decision = evaluateKeyframe(
                    rotationAngleRad(reference_view.pose.rotation, candidate_pose.rotation),
                    baselineM(reference_view.pose, candidate_pose), 1.0, options_.keyframe);
                if (!decision.accept) {
                    continue;
                }

                ViewState& state = reconstruction.views[static_cast<std::size_t>(view)];
                state.pose = candidate_pose;
                state.registered = true;
                progress = true;

                // Attach this view to points it already sees, then create the points only it and the
                // reference can triangulate.
                const double limit = kOutlierReprojectionPx * kOutlierReprojectionPx;
                for (const std::pair<int, int>& match : pair->matches) {
                    const int mine = featureOf(*pair, view, match);
                    const int theirs = otherFeatureOf(*pair, view, match);
                    const auto found = point_of_feature.find(featureKey(reference, theirs));
                    if (found == point_of_feature.end()) {
                        continue;
                    }
                    MapPoint& point =
                        reconstruction.points[static_cast<std::size_t>(found->second)];
                    const Vec2 pixel =
                        pixels_[static_cast<std::size_t>(view)][static_cast<std::size_t>(mine)];
                    if (squaredReprojectionPx(state.camera, state.pose, point.position_m, pixel) <=
                        limit) {
                        point.observations.push_back({view, mine, pixel});
                    }
                }

                for (const std::pair<int, int>& match : pair->matches) {
                    const int mine = featureOf(*pair, view, match);
                    const int theirs = otherFeatureOf(*pair, view, match);
                    if (point_of_feature.count(featureKey(view, mine)) != 0 ||
                        point_of_feature.count(featureKey(reference, theirs)) != 0) {
                        continue;
                    }
                    TriangulationResult triangulated;
                    if (!triangulateTwoView(
                            reference_view.camera, reference_view.pose,
                            pixels_[static_cast<std::size_t>(reference)]
                                  [static_cast<std::size_t>(theirs)],
                            state.camera, state.pose,
                            pixels_[static_cast<std::size_t>(view)]
                                  [static_cast<std::size_t>(mine)],
                            triangulated, kMinParallaxRad)) {
                        continue;
                    }
                    const int index = static_cast<int>(reconstruction.points.size());
                    MapPoint point;
                    point.position_m = triangulated.position_m;
                    point.observations = {
                        {reference, theirs,
                         pixels_[static_cast<std::size_t>(reference)]
                               [static_cast<std::size_t>(theirs)]},
                        {view, mine,
                         pixels_[static_cast<std::size_t>(view)][static_cast<std::size_t>(mine)]},
                    };
                    point_of_feature[featureKey(reference, theirs)] = index;
                    point_of_feature[featureKey(view, mine)] = index;
                    reconstruction.points.push_back(std::move(point));
                }

                prune(reconstruction);
                point_of_feature = featureIndex(reconstruction);
            }
        }
    };

    // Every run re-seeds from whichever pair scores best, and the seed decides the frame: its first
    // camera is the origin. A live session runs this repeatedly on a growing view set, so the frame
    // used to move between updates and the overlay cloud jumped away from the features it belongs to
    // (D043). Re-expressing the result in view 0's camera frame keeps every run in one gauge.
    auto reanchorToView = [](Reconstruction& reconstruction, int view) {
        if (view < 0 || static_cast<std::size_t>(view) >= reconstruction.views.size()) {
            return;
        }
        const ViewState& anchor = reconstruction.views[static_cast<std::size_t>(view)];
        if (!anchor.registered) {
            return;
        }
        const Mat3 rotation = geometry::transpose3(anchor.pose.rotation);
        const Vec3 centre = anchor.pose.camera_centre_m;
        auto transformPoint = [&](Vec3& point) {
            const Vec3 delta{point[0] - centre[0], point[1] - centre[1], point[2] - centre[2]};
            point = geometry::matvec3(rotation, delta);
        };
        auto transformRotation = [&](Mat3& pose_rotation) {
            Mat3 out{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    double sum = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        sum += rotation[row * 3 + k] * pose_rotation[k * 3 + column];
                    }
                    out[row * 3 + column] = sum;
                }
            }
            pose_rotation = out;
        };

        for (MapPoint& point : reconstruction.points) {
            transformPoint(point.position_m);
        }
        for (ViewState& state : reconstruction.views) {
            if (!state.registered) {
                continue;
            }
            transformPoint(state.pose.camera_centre_m);
            transformRotation(state.pose.rotation);
        }
    };

    struct SeedAttempt {
        const PairMatches* pair = nullptr;
        Reconstruction reconstruction;
        int support = 0;
        int points = 0;
    };

    std::vector<SeedAttempt> seeds;
    for (const PairMatches* pair : ordered) {
        if (static_cast<int>(seeds.size()) >= kMaxSeedAttempts) {
            break;
        }
        SeedAttempt attempt;
        attempt.pair = pair;
        attempt.reconstruction.views = base_views;
        if (!seedInto(attempt.reconstruction, *pair)) {
            continue;
        }
        prune(attempt.reconstruction);
        attempt.points = validPointCount(attempt.reconstruction);
        if (attempt.points == 0) {
            continue;
        }
        attempt.support = thirdViewSupport(attempt.reconstruction, pairs, *pair);
        seeds.push_back(std::move(attempt));
    }
    if (seeds.empty()) {
        return result_.report;
    }

    // A pair alone cannot tell a mirror from the truth, and the mirrored seed keeps only a sliver of
    // its inliers, which no neighbour can match. Scoring by third-view support picks the branches
    // that can actually grow; only the best few are grown because that is the expensive step
    // (D029, D037).
    std::stable_sort(seeds.begin(), seeds.end(), [](const SeedAttempt& a, const SeedAttempt& b) {
        if (a.support != b.support) {
            return a.support > b.support;
        }
        return a.points > b.points;
    });

    SfmResult best;
    int best_registered = -1;
    int best_points = -1;
    int grown = 0;
    for (SeedAttempt& seed : seeds) {
        if (grown > 0 && seed.support < kMinPnpCorrespondences) {
            continue;
        }
        ++grown;

        SfmResult attempt;
        attempt.reconstruction = std::move(seed.reconstruction);
        attempt.registration_order = {seed.pair->view_a, seed.pair->view_b};

        grow(attempt.reconstruction);

        const int registered_views = registeredCount(attempt.reconstruction);
        // Bundle adjustment only runs once a third view agrees: with two views it can trade the
        // reconstruction for a mirrored one that fits the same pixels (D029).
        if (registered_views >= 3 && validPointCount(attempt.reconstruction) > 0) {
            optimize(attempt.reconstruction, options_.bundle_adjustment);
            prune(attempt.reconstruction);
        }
        attempt.report = summarize(attempt.reconstruction);

        if (attempt.report.views_registered > best_registered ||
            (attempt.report.views_registered == best_registered &&
             attempt.report.points > best_points)) {
            best_registered = attempt.report.views_registered;
            best_points = attempt.report.points;
            best = attempt;
        }
        if (registered_views >= 3 || grown >= kMaxGrowAttempts) {
            break;
        }
    }

    reanchorToView(best.reconstruction, 0);
    result_ = std::move(best);
    ran_ = result_.report.views_registered >= 2;
    return result_.report;
}

int Session::pointCount() const {
    int count = 0;
    for (const MapPoint& point : result_.reconstruction.points) {
        if (point.valid) {
            ++count;
        }
    }
    return count;
}

int Session::copyPoints(double* positions_m, int max_points) const {
    if (positions_m == nullptr || max_points <= 0) {
        return 0;
    }
    int written = 0;
    for (const MapPoint& point : result_.reconstruction.points) {
        if (!point.valid || written >= max_points) {
            continue;
        }
        positions_m[written * 3] = point.position_m[0];
        positions_m[written * 3 + 1] = point.position_m[1];
        positions_m[written * 3 + 2] = point.position_m[2];
        ++written;
    }
    return written;
}

int Session::cameraCount() const {
    return static_cast<int>(result_.reconstruction.views.size());
}

bool Session::copyCamera(int index, double out_rotation[9], double out_centre_m[3]) const {
    if (out_rotation == nullptr || out_centre_m == nullptr || index < 0 ||
        static_cast<std::size_t>(index) >= result_.reconstruction.views.size()) {
        return false;
    }
    const ViewState& view = result_.reconstruction.views[static_cast<std::size_t>(index)];
    for (int i = 0; i < 9; ++i) {
        out_rotation[i] = view.pose.rotation[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 3; ++i) {
        out_centre_m[i] = view.pose.camera_centre_m[static_cast<std::size_t>(i)];
    }
    return true;
}

std::shared_ptr<const Tracker> Session::snapshot() const {
    // Copy the map rather than sharing it: the session rewrites its reconstruction on every run,
    // and a tracker that saw a half-updated map would place the camera in a frame that no longer
    // matches the cloud on screen.
    auto tracker = std::shared_ptr<Tracker>(new Tracker());
    tracker->features_ = features_;
    // Parallel to features_, not to the reconstruction: a session that has frames but no map yet
    // (the state the app snapshots when a scan starts) holds more feature sets than the
    // reconstruction has views, and a shorter list made the tracker read past its end.
    tracker->registered_.assign(features_.size(), false);
    const std::vector<ViewState>& views = result_.reconstruction.views;
    for (std::size_t index = 0; index < views.size() && index < tracker->registered_.size();
         ++index) {
        tracker->registered_[index] = views[index].registered;
    }
    tracker->point_of_feature_ = featureIndex(result_.reconstruction);
    tracker->point_positions_.reserve(result_.reconstruction.points.size());
    for (const MapPoint& point : result_.reconstruction.points) {
        tracker->point_positions_.push_back(point.position_m);
    }
    tracker->feature_params_ = feature_params_;
    tracker->pnp_ = options_.pnp;
    return tracker;
}

bool Session::trackPose(
    int width_px,
    int height_px,
    const unsigned char* gray,
    double focal_px,
    double cx_px,
    double cy_px,
    Pose& out_pose,
    int& out_inliers,
    int view_hint) const {
    // One implementation, in the snapshot: the app tracks off the session lock, and the two paths
    // must not drift apart.
    return snapshot()->track(width_px, height_px, gray, focal_px, cx_px, cy_px, out_pose,
                             out_inliers, view_hint);
}

}  // namespace scand3d::sfm
