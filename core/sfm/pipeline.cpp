#include "sfm/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "sfm/map_store.hpp"
#include "sfm/pose.hpp"
#include "sfm/triangulation.hpp"

namespace scand3d::sfm {
namespace {

/// How many seed pairs are tried before giving up on growing a reconstruction.
constexpr int kMaxSeedAttempts = 8;

/// The features of a view that already have a map point, in the view's own feature order.
void mapCorrespondences(
    const FrameInput& frame,
    int view,
    const MapStore& store,
    std::vector<Vec3>& points,
    std::vector<Vec2>& pixels) {
    for (std::size_t feature = 0; feature < frame.pixels.size(); ++feature) {
        const int point_index = store.pointForFeature(view, static_cast<int>(feature));
        if (point_index < 0) {
            continue;
        }
        const MapPoint& point = store.data().points[static_cast<std::size_t>(point_index)];
        if (!point.valid) {
            continue;
        }
        points.push_back(point.position_m);
        pixels.push_back(frame.pixels[feature]);
    }
}

/// Seeds the map from one matched pair: essential matrix, relative pose, both views registry.
bool seedFromPair(
    const PairMatches& pair,
    const std::vector<FrameInput>& frames,
    const SfmOptions& options,
    MapStore& store,
    SfmResult& result) {
    const FrameInput& first_frame = frames[static_cast<std::size_t>(pair.view_a)];
    const FrameInput& second_frame = frames[static_cast<std::size_t>(pair.view_b)];

    std::vector<geometry::PointCorrespondence> correspondences;
    correspondences.reserve(pair.matches.size());
    for (const auto& match : pair.matches) {
        const auto feature_a = static_cast<std::size_t>(match.first);
        const auto feature_b = static_cast<std::size_t>(match.second);
        if (feature_a >= first_frame.pixels.size() || feature_b >= second_frame.pixels.size()) {
            continue;
        }
        correspondences.push_back(
            {first_frame.pixels[feature_a], second_frame.pixels[feature_b]});
    }
    if (correspondences.size() < 8) {
        return false;
    }

    const TwoViewInitialization initialization =
        initializeTwoView(correspondences, first_frame.camera, second_frame.camera, options.two_view);
    if (!initialization.valid) {
        return false;
    }

    ViewState& first = store.data().views[static_cast<std::size_t>(pair.view_a)];
    ViewState& second = store.data().views[static_cast<std::size_t>(pair.view_b)];
    first.pose = initialization.first;
    first.registered = true;
    second.pose = initialization.second;
    second.registered = true;
    result.registration_order = {pair.view_a, pair.view_b};
    return true;
}

/// Registers views by PnP until a full pass adds nothing. A view that failed while the map was
/// still thin gets another chance in the next pass; sweeps replace extending from one end, so a
/// scan seeded in its middle grows in both directions.
void growReconstruction(
    const std::vector<FrameInput>& frames,
    const SfmOptions& options,
    MapStore& store,
    SfmResult& result) {
    std::vector<KeyframeDecision> view_decisions(frames.size());
    std::vector<bool> has_decision(frames.size(), false);
    int registered_count = static_cast<int>(result.registration_order.size());

    bool progress = true;
    while (progress) {
        progress = false;
        for (std::size_t v = 0; v < frames.size(); ++v) {
            if (store.data().views[v].registered) {
                continue;
            }

            std::vector<Vec3> points;
            std::vector<Vec2> pixels;
            mapCorrespondences(frames[v], static_cast<int>(v), store, points, pixels);
            if (points.size() < 6) {
                continue;
            }

            const PnpResult pnp =
                estimatePoseRansac(points, pixels, frames[v].camera, options.pnp);
            const double required = std::max(
                6.0, options.min_pnp_inlier_ratio * static_cast<double>(points.size()));
            if (!pnp.valid || static_cast<double>(pnp.inliers) < required) {
                view_decisions[v].view = static_cast<int>(v);
                view_decisions[v].reason = "pnp failed";
                has_decision[v] = true;
                continue;
            }

            // Compare against the nearest registered view by rotation: with a turntable that is the
            // neighbouring keyframe, whose baseline and overlap actually constrain this candidate.
            int reference = -1;
            double rotation = 0.0;
            for (std::size_t u = 0; u < store.data().views.size(); ++u) {
                const ViewState& other = store.data().views[u];
                if (!other.registered) {
                    continue;
                }
                const double candidate_rotation =
                    rotationAngleRad(other.pose.rotation, pnp.pose.rotation);
                if (reference < 0 || candidate_rotation < rotation) {
                    reference = static_cast<int>(u);
                    rotation = candidate_rotation;
                }
            }
            if (reference < 0) {
                break;
            }

            const double baseline =
                baselineM(store.data().views[static_cast<std::size_t>(reference)].pose, pnp.pose);
            const double overlap = static_cast<double>(points.size()) /
                                   static_cast<double>(std::max<std::size_t>(1, frames[v].pixels.size()));
            KeyframeDecision decision =
                evaluateKeyframe(rotation, baseline, overlap, options.keyframe);
            decision.view = static_cast<int>(v);
            view_decisions[v] = decision;
            has_decision[v] = true;
            if (!decision.accept) {
                continue;
            }

            ViewState& view = store.data().views[v];
            view.pose = pnp.pose;
            view.registered = true;
            result.registration_order.push_back(static_cast<int>(v));
            ++registered_count;
            progress = true;

            store.triangulateTracks(options.min_parallax_rad);

            if (options.bundle_adjust_every > 0 &&
                registered_count % options.bundle_adjust_every == 0) {
                optimize(store.data(), options.bundle_adjustment);
                store.removeOutlierObservations(options.outlier_reprojection_px);
                store.triangulateTracks(options.min_parallax_rad);
            }
        }
    }

    for (std::size_t v = 0; v < frames.size(); ++v) {
        if (has_decision[v]) {
            result.decisions.push_back(view_decisions[v]);
        }
    }
}

/// Bundle adjustment, outlier removal and duplicate merging, then the report. Two-view
/// reconstructions skip BA: with two cameras it can trade the whole reconstruction for a mirrored
/// one that fits the same pixels.
void finalize(const SfmOptions& options, MapStore& store, SfmResult& result) {
    if (result.registration_order.size() >= 3) {
        optimize(store.data(), options.bundle_adjustment);
    }
    store.removeOutlierObservations(options.outlier_reprojection_px);
    store.mergeDuplicates(0.01);
    result.reconstruction = store.data();
    result.report = summarize(result.reconstruction);
}

}  // namespace

SfmResult reconstruct(
    const std::vector<FrameInput>& frames,
    const std::vector<PairMatches>& pairs,
    const SfmOptions& options) {
    SfmResult result;
    if (frames.size() < 2 || pairs.empty()) {
        result.report = summarize(result.reconstruction);
        return result;
    }

    std::vector<ViewState> base_views(frames.size());
    std::vector<std::vector<Vec2>> pixels(frames.size());
    for (std::size_t i = 0; i < frames.size(); ++i) {
        base_views[i].camera = frames[i].camera;
        base_views[i].feature_count = frames[i].pixels.size();
        pixels[i] = frames[i].pixels;
    }

    TrackBuilder builder(frames.size());
    for (const PairMatches& pair : pairs) {
        for (const auto& match : pair.matches) {
            builder.addMatch(pair.view_a, match.first, pair.view_b, match.second);
        }
    }
    const std::vector<std::vector<Observation>> tracks = builder.build();

    std::vector<const PairMatches*> ordered;
    ordered.reserve(pairs.size());
    for (const PairMatches& pair : pairs) {
        if (pair.view_a >= 0 && pair.view_b >= 0 &&
            static_cast<std::size_t>(pair.view_a) < frames.size() &&
            static_cast<std::size_t>(pair.view_b) < frames.size() && pair.matches.size() >= 8) {
            ordered.push_back(&pair);
        }
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const PairMatches* a, const PairMatches* b) {
                         return a->matches.size() > b->matches.size();
                     });

    // A seed is only trusted once another view registers against it. Both decompositions of the
    // essential matrix fit their own pair, so a mirrored seed is indistinguishable from the right
    // one until a third view contradicts it. Pairs are tried in match-count order until one grows.
    SfmResult partial;
    int attempts = 0;
    for (const PairMatches* pair : ordered) {
        if (attempts >= kMaxSeedAttempts) {
            break;
        }
        ++attempts;

        MapStore store(base_views);
        store.setFeaturePixels(pixels);
        store.setTracks(tracks);

        SfmResult attempt;
        if (!seedFromPair(*pair, frames, options, store, attempt)) {
            continue;
        }

        store.triangulateTracks(options.min_parallax_rad);
        growReconstruction(frames, options, store, attempt);
        finalize(options, store, attempt);

        if (attempt.registration_order.size() >= 3) {
            return attempt;
        }
        if (partial.registration_order.empty()) {
            partial = std::move(attempt);
        }
    }

    if (!partial.registration_order.empty()) {
        return partial;
    }
    result.report = summarize(result.reconstruction);
    return result;
}

}  // namespace scand3d::sfm
