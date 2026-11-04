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
#include "sfm/triangulation.hpp"

namespace scand3d::sfm {
namespace {

// Pairs below this many matches carry no useful baseline and only cost a RANSAC round.
constexpr int kMinPairMatches = 12;

// Capture is sequential, so only neighbouring views are worth matching.
constexpr int kMaxViewGap = 2;

// Seed pairs tried before the reconstruction is abandoned.
constexpr int kMaxSeedAttempts = 8;

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

/// The feature of `view` in a match stored with the pair's own orientation.
int featureOf(const PairMatches& pair, int view, const std::pair<int, int>& match) {
    return pair.view_a == view ? match.first : match.second;
}

/// The feature of the *other* view in the same match.
int otherFeatureOf(const PairMatches& pair, int view, const std::pair<int, int>& match) {
    return pair.view_a == view ? match.second : match.first;
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

    // Matching is pairwise and geometric: every pair is validated by an essential matrix before
    // any of its matches can create a 3D point, so a false match cannot drag a track off geometry.
    const feature::MatchOptions match_options = neighbourMatchOptions();
    std::vector<PairMatches> pairs;
    for (std::size_t first = 0; first < features_.size(); ++first) {
        const std::size_t limit =
            std::min(features_.size(), first + static_cast<std::size_t>(kMaxViewGap) + 1);
        for (std::size_t second = first + 1; second < limit; ++second) {
            const std::vector<feature::FeatureMatch> matches =
                feature::matchApproximate(features_[first], features_[second], match_options);
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

    Reconstruction& reconstruction = result_.reconstruction;
    reconstruction.views.resize(features_.size());
    for (std::size_t index = 0; index < features_.size(); ++index) {
        reconstruction.views[index].camera = cameras_[index];
        reconstruction.views[index].feature_count = pixels_[index].size();
    }

    // The seed is the richest pair that initialises; both decompositions fit their own pair, so a
    // mirrored seed only shows up once a third view fails to register against it.
    std::vector<const PairMatches*> ordered;
    ordered.reserve(pairs.size());
    for (const PairMatches& pair : pairs) {
        ordered.push_back(&pair);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const PairMatches* a, const PairMatches* b) {
                         return a->matches.size() > b->matches.size();
                     });

    bool seeded = false;
    int attempts = 0;
    for (const PairMatches* pair : ordered) {
        if (attempts >= kMaxSeedAttempts) {
            break;
        }
        ++attempts;

        std::vector<geometry::PointCorrespondence> correspondences;
        correspondences.reserve(pair->matches.size());
        for (const std::pair<int, int>& match : pair->matches) {
            correspondences.push_back(
                {pixels_[static_cast<std::size_t>(pair->view_a)]
                          [static_cast<std::size_t>(match.first)],
                 pixels_[static_cast<std::size_t>(pair->view_b)]
                          [static_cast<std::size_t>(match.second)]});
        }
        const TwoViewInitialization initialization =
            initializeTwoView(correspondences,
                              cameras_[static_cast<std::size_t>(pair->view_a)],
                              cameras_[static_cast<std::size_t>(pair->view_b)], options_.two_view);
        if (!initialization.valid) {
            continue;
        }

        const int view_a = pair->view_a;
        const int view_b = pair->view_b;
        reconstruction.views[static_cast<std::size_t>(view_a)].pose = initialization.first;
        reconstruction.views[static_cast<std::size_t>(view_a)].registered = true;
        reconstruction.views[static_cast<std::size_t>(view_b)].pose = initialization.second;
        reconstruction.views[static_cast<std::size_t>(view_b)].registered = true;
        result_.registration_order = {view_a, view_b};

        // The seed already triangulated its own points; keeping them avoids a second solve.
        for (std::size_t index = 0; index < initialization.points.size(); ++index) {
            const std::pair<int, int>& match =
                pair->matches[static_cast<std::size_t>(initialization.source_index[index])];
            MapPoint point;
            point.position_m = initialization.points[index];
            point.observations = {
                {view_a, match.first, pixels_[static_cast<std::size_t>(view_a)]
                                                 [static_cast<std::size_t>(match.first)]},
                {view_b, match.second, pixels_[static_cast<std::size_t>(view_b)]
                                                  [static_cast<std::size_t>(match.second)]},
            };
            reconstruction.points.push_back(std::move(point));
        }
        seeded = true;
        break;
    }
    if (!seeded) {
        return result_.report;
    }

    prune(reconstruction);
    std::unordered_map<std::uint64_t, int> point_of_feature = featureIndex(reconstruction);

    // Incremental growth: register one view at a time against the nearest registered neighbour and
    // extend the map only from matches the new pose actually explains. Rebuilding the whole map from
    // global tracks welds inconsistent matches together, which is why this walks the capture order
    // instead (D035).
    int registered = static_cast<int>(result_.registration_order.size());
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
                    const bool matches_views =
                        (candidate.view_a == view && candidate.view_b == other) ||
                        (candidate.view_a == other && candidate.view_b == view);
                    if (matches_views && candidate.matches.size() > best_matches) {
                        best_matches = candidate.matches.size();
                        pair = &candidate;
                        reference = other;
                    }
                }
            }
            if (pair == nullptr) {
                continue;
            }

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
            if (object_points.size() < 6) {
                continue;
            }

            const PnpResult pnp =
                estimatePoseRansac(object_points, image_points,
                                   cameras_[static_cast<std::size_t>(view)], options_.pnp);
            const double required =
                std::max(6.0, options_.min_pnp_inlier_ratio *
                                  static_cast<double>(object_points.size()));
            if (!pnp.valid || static_cast<double>(pnp.inliers) < required) {
                continue;
            }

            // Rotation and baseline windows keep near-duplicate frames out of the map.
            const ViewState& reference_view = reconstruction.views[static_cast<std::size_t>(reference)];
            const KeyframeDecision decision = evaluateKeyframe(
                rotationAngleRad(reference_view.pose.rotation, pnp.pose.rotation),
                baselineM(reference_view.pose, pnp.pose), 1.0, options_.keyframe);
            if (!decision.accept) {
                continue;
            }

            ViewState& state = reconstruction.views[static_cast<std::size_t>(view)];
            state.pose = pnp.pose;
            state.registered = true;
            result_.registration_order.push_back(view);
            ++registered;
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
                        pixels_[static_cast<std::size_t>(view)][static_cast<std::size_t>(mine)],
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

            if (options_.bundle_adjust_every > 0 &&
                registered % options_.bundle_adjust_every == 0) {
                optimize(reconstruction, options_.bundle_adjustment);
                prune(reconstruction);
                point_of_feature = featureIndex(reconstruction);
            }
        }
    }

    if (registered >= 3) {
        optimize(reconstruction, options_.bundle_adjustment);
        prune(reconstruction);
    }
    result_.report = summarize(reconstruction);
    ran_ = registered >= 2;
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
    out_inliers = 0;
    if (!ran_ || gray == nullptr || width_px <= 0 || height_px <= 0 || !(focal_px > 0.0)) {
        return false;
    }

    feature::GrayImage image;
    image.width_px = width_px;
    image.height_px = height_px;
    image.pixels.assign(gray,
                        gray + static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px));
    const feature::FeatureSet live = feature::extractFeatures(image, feature_params_);
    if (live.empty()) {
        return false;
    }

    int reference = view_hint;
    if (reference < 0 ||
        static_cast<std::size_t>(reference) >= result_.reconstruction.views.size()) {
        reference = -1;
        for (std::size_t index = result_.reconstruction.views.size(); index > 0; --index) {
            if (result_.reconstruction.views[index - 1].registered) {
                reference = static_cast<int>(index - 1);
                break;
            }
        }
    }
    if (reference < 0 || static_cast<std::size_t>(reference) >= features_.size()) {
        return false;
    }

    std::unordered_map<std::uint64_t, int> point_of_feature;
    for (std::size_t index = 0; index < result_.reconstruction.points.size(); ++index) {
        const MapPoint& point = result_.reconstruction.points[index];
        if (!point.valid) {
            continue;
        }
        for (const Observation& observation : point.observations) {
            point_of_feature[featureKey(observation.view, observation.feature)] =
                static_cast<int>(index);
        }
    }

    feature::MatchOptions match_options;
    match_options.ratio_percent = 85;
    match_options.cross_check = true;
    const std::vector<feature::FeatureMatch> matches = feature::matchApproximate(
        live, features_[static_cast<std::size_t>(reference)], match_options);

    std::vector<Vec3> object_points;
    std::vector<Vec2> image_points;
    object_points.reserve(matches.size());
    image_points.reserve(matches.size());
    for (const feature::FeatureMatch& match : matches) {
        const auto found = point_of_feature.find(featureKey(reference, match.train_index));
        if (found == point_of_feature.end()) {
            continue;
        }
        const MapPoint& point =
            result_.reconstruction.points[static_cast<std::size_t>(found->second)];
        object_points.push_back(point.position_m);
        image_points.push_back(
            {live.keypoints[static_cast<std::size_t>(match.query_index)].x_px,
             live.keypoints[static_cast<std::size_t>(match.query_index)].y_px});
    }

    if (object_points.size() < 6) {
        return false;
    }

    const PnpResult pnp = estimatePoseRansac(
        object_points, image_points, CameraModel{focal_px, cx_px, cy_px}, options_.pnp);
    if (!pnp.valid) {
        return false;
    }
    out_pose = pnp.pose;
    out_inliers = pnp.inliers;
    return true;
}

}  // namespace scand3d::sfm
