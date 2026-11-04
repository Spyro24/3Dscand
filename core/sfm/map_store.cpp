#include "sfm/map_store.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <unordered_map>
#include <utility>

#include "sfm/triangulation.hpp"

namespace scand3d::sfm {
namespace {

std::uint64_t featureKey(int view, int feature) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(view)) << 32) |
           static_cast<std::uint32_t>(feature);
}

}  // namespace

TrackBuilder::TrackBuilder(std::size_t view_count) : view_count_(view_count) {}

int TrackBuilder::nodeFor(int view, int feature) const {
    const std::uint64_t key = featureKey(view, feature);
    const auto found = lookup_.find(key);
    if (found != lookup_.end()) {
        return found->second;
    }
    const int id = static_cast<int>(nodes_.size());
    nodes_.push_back(Observation{view, feature, {}});
    parent_.push_back(id);
    lookup_.emplace(key, id);
    return id;
}

void TrackBuilder::addMatch(int view_a, int feature_a, int view_b, int feature_b) {
    if (view_a < 0 || view_b < 0 || static_cast<std::size_t>(view_a) >= view_count_ ||
        static_cast<std::size_t>(view_b) >= view_count_) {
        return;
    }
    const int first = nodeFor(view_a, feature_a);
    const int second = nodeFor(view_b, feature_b);

    auto root = [this](int node) {
        while (parent_[static_cast<std::size_t>(node)] != node) {
            parent_[static_cast<std::size_t>(node)] =
                parent_[static_cast<std::size_t>(parent_[static_cast<std::size_t>(node)])];
            node = parent_[static_cast<std::size_t>(node)];
        }
        return node;
    };
    const int first_root = root(first);
    const int second_root = root(second);
    if (first_root != second_root) {
        parent_[static_cast<std::size_t>(second_root)] = first_root;
    }
}

std::vector<std::vector<Observation>> TrackBuilder::build() const {
    std::map<int, std::vector<Observation>> grouped;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        int root = static_cast<int>(i);
        while (parent_[static_cast<std::size_t>(root)] != root) {
            root = parent_[static_cast<std::size_t>(root)];
        }
        grouped[root].push_back(nodes_[i]);
    }

    std::vector<std::vector<Observation>> tracks;
    tracks.reserve(grouped.size());
    for (auto& entry : grouped) {
        std::vector<Observation>& observations = entry.second;
        std::sort(observations.begin(), observations.end(),
                  [](const Observation& a, const Observation& b) { return a.view < b.view; });
        observations.erase(std::unique(observations.begin(), observations.end(),
                                       [](const Observation& a, const Observation& b) {
                                           return a.view == b.view;
                                       }),
                           observations.end());
        if (observations.size() >= 2) {
            tracks.push_back(std::move(observations));
        }
    }
    return tracks;
}

MapStore::MapStore(std::vector<ViewState> views) {
    data_.views = std::move(views);
}

Reconstruction& MapStore::data() { return data_; }
const Reconstruction& MapStore::data() const { return data_; }

void MapStore::setFeaturePixels(std::vector<std::vector<Vec2>> pixels) {
    feature_pixels_ = std::move(pixels);
}

void MapStore::setTracks(const std::vector<std::vector<Observation>>& tracks) {
    data_.points.clear();
    data_.points.reserve(tracks.size());
    for (const std::vector<Observation>& observations : tracks) {
        MapPoint point;
        point.observations = observations;
        for (Observation& observation : point.observations) {
            const auto view = static_cast<std::size_t>(observation.view);
            const auto feature = static_cast<std::size_t>(observation.feature);
            if (view < feature_pixels_.size() && feature < feature_pixels_[view].size()) {
                observation.pixel = feature_pixels_[view][feature];
            }
        }
        point.valid = false;  // triangulation decides
        data_.points.push_back(std::move(point));
    }
    rebuildFeatureIndex();
}

int MapStore::triangulateTracks(double min_parallax_rad) {
    int triangulated = 0;
    for (MapPoint& point : data_.points) {
        if (point.observations.size() < 2) {
            point.valid = false;
            continue;
        }
        std::vector<ViewObservation> views;
        views.reserve(point.observations.size());
        for (const Observation& observation : point.observations) {
            const ViewState& view = data_.views[static_cast<std::size_t>(observation.view)];
            if (!view.registered) {
                continue;
            }
            views.push_back({view.camera, view.pose, observation.pixel});
        }
        TriangulationResult result;
        if (views.size() >= 2 && triangulatePoint(views, result, min_parallax_rad)) {
            point.position_m = result.position_m;
            point.valid = true;
            ++triangulated;
        } else {
            point.valid = false;
        }
    }
    rebuildFeatureIndex();
    return triangulated;
}

int MapStore::removeOutlierObservations(double max_reprojection_px) {
    const double threshold_squared = max_reprojection_px * max_reprojection_px;
    int dropped = 0;
    for (MapPoint& point : data_.points) {
        if (!point.valid) {
            continue;
        }
        std::vector<Observation> kept;
        kept.reserve(point.observations.size());
        for (const Observation& observation : point.observations) {
            const ViewState& view = data_.views[static_cast<std::size_t>(observation.view)];
            if (!view.registered) {
                kept.push_back(observation);
                continue;
            }
            if (squaredReprojectionPx(view.camera, view.pose, point.position_m, observation.pixel) >
                threshold_squared) {
                ++dropped;
                continue;
            }
            kept.push_back(observation);
        }
        if (kept.size() < 2) {
            point.valid = false;
        }
        point.observations = std::move(kept);
    }
    rebuildFeatureIndex();
    return dropped;
}

int MapStore::mergeDuplicates(double min_distance_m) {
    if (!(min_distance_m > 0.0)) {
        return 0;
    }
    const double min_squared = min_distance_m * min_distance_m;
    std::unordered_map<std::uint64_t, std::vector<int>> grid;
    const double cell = std::max(min_distance_m, 1e-6);

    auto cellKey = [cell](const Vec3& position) {
        const auto x = static_cast<std::int64_t>(std::floor(position[0] / cell));
        const auto y = static_cast<std::int64_t>(std::floor(position[1] / cell));
        const auto z = static_cast<std::int64_t>(std::floor(position[2] / cell));
        return (static_cast<std::uint64_t>(x & 0x1FFFFF) << 42) |
               (static_cast<std::uint64_t>(y & 0x1FFFFF) << 21) |
               static_cast<std::uint64_t>(z & 0x1FFFFF);
    };

    int merged = 0;
    for (std::size_t i = 0; i < data_.points.size(); ++i) {
        MapPoint& point = data_.points[i];
        if (!point.valid) {
            continue;
        }
        std::vector<int>& occupants = grid[cellKey(point.position_m)];
        bool duplicate = false;
        for (const int other : occupants) {
            const MapPoint& candidate = data_.points[static_cast<std::size_t>(other)];
            const Vec3 delta{point.position_m[0] - candidate.position_m[0],
                             point.position_m[1] - candidate.position_m[1],
                             point.position_m[2] - candidate.position_m[2]};
            if (geometry::dot3(delta, delta) > min_squared) {
                continue;
            }
            // Only merge when they also share an observation: points at the same place from
            // disjoint views can be genuinely different parts of the object.
            const bool shares_view = std::any_of(
                point.observations.begin(), point.observations.end(),
                [&candidate](const Observation& observation) {
                    return std::any_of(candidate.observations.begin(),
                                       candidate.observations.end(),
                                       [&observation](const Observation& other) {
                                           return other.view == observation.view;
                                       });
                });
            if (shares_view) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            point.valid = false;
            ++merged;
            continue;
        }
        occupants.push_back(static_cast<int>(i));
    }
    rebuildFeatureIndex();
    return merged;
}

int MapStore::pointForFeature(int view, int feature) const {
    const auto found = feature_to_point_.find(featureKey(view, feature));
    return found == feature_to_point_.end() ? -1 : found->second;
}

int MapStore::validPoints() const {
    return static_cast<int>(
        std::count_if(data_.points.begin(), data_.points.end(),
                      [](const MapPoint& point) { return point.valid; }));
}

int MapStore::observationCount() const {
    int count = 0;
    for (const MapPoint& point : data_.points) {
        if (point.valid) {
            count += static_cast<int>(point.observations.size());
        }
    }
    return count;
}

void MapStore::rebuildFeatureIndex() {
    feature_to_point_.clear();
    for (std::size_t i = 0; i < data_.points.size(); ++i) {
        const MapPoint& point = data_.points[i];
        if (!point.valid) {
            continue;
        }
        for (const Observation& observation : point.observations) {
            feature_to_point_[featureKey(observation.view, observation.feature)] =
                static_cast<int>(i);
        }
    }
}

}  // namespace scand3d::sfm
