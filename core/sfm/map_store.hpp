#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "sfm/types.hpp"

namespace scand3d::sfm {

/// Groups per-view feature indices into tracks with union-find, so a point matched through
/// several views becomes one track instead of several pairs (roadmap 4.5).
class TrackBuilder {
public:
    explicit TrackBuilder(std::size_t view_count);

    void addMatch(int view_a, int feature_a, int view_b, int feature_b);

    /// One sorted, de-duplicated observation list per track. Tracks with a single observation are
    /// dropped: they cannot be triangulated.
    std::vector<std::vector<Observation>> build() const;

private:
    int nodeFor(int view, int feature) const;

    mutable std::vector<int> parent_;
    mutable std::vector<Observation> nodes_;
    mutable std::unordered_map<std::uint64_t, int> lookup_;
    std::size_t view_count_ = 0;
};

/// Owns the reconstruction while it is being extended: track observations, triangulation, outlier
/// removal and the feature-to-point lookup PnP needs.
class MapStore {
public:
    explicit MapStore(std::vector<ViewState> views);

    Reconstruction& data();
    const Reconstruction& data() const;

    /// Pixels of every feature of every view, indexed like the caller's feature arrays. Tracks
    /// carry only (view, feature); this is where their pixel comes from.
    void setFeaturePixels(std::vector<std::vector<Vec2>> pixels);

    /// Replaces the map with these tracks; positions are cleared until triangulation runs.
    void setTracks(const std::vector<std::vector<Observation>>& tracks);

    /// Triangulates every track seen by at least two registered views. Returns the new point count.
    int triangulateTracks(double min_parallax_rad);

    /// Drops observations whose reprojection error exceeds the threshold, and invalidates points
    /// left with fewer than two observations. Returns the number of dropped observations.
    int removeOutlierObservations(double max_reprojection_px);

    /// Invalidates points closer than `min_distance_m` to another point that shares a view.
    int mergeDuplicates(double min_distance_m);

    /// Point index for a feature of a view, or -1. Uses the index rebuilt after every map change.
    int pointForFeature(int view, int feature) const;

    int validPoints() const;
    int observationCount() const;

private:
    void rebuildFeatureIndex();

    Reconstruction data_;
    std::unordered_map<std::uint64_t, int> feature_to_point_;
    std::vector<std::vector<Vec2>> feature_pixels_;
};

}  // namespace scand3d::sfm
