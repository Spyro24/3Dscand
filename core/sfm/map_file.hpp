#pragma once

#include <string>
#include <vector>

#include "feature/features.hpp"
#include "sfm/types.hpp"

namespace scand3d::sfm {

/// One view as it is stored on disk: intrinsics, pose and the features a later process needs to
/// match a live frame against it.
struct StoredMapView {
    CameraModel camera;
    Pose pose;
    bool registered = false;
    feature::FeatureSet features;
};

/// Everything one scan contributes to relocalisation: the views with their poses and the sparse
/// points with their observations.
struct StoredMap {
    std::vector<StoredMapView> views;
    std::vector<MapPoint> points;
};

/// Writes `map` into `directory` as `map.bin` plus one `view_<i>.feat` per view. The features use
/// the same versioned format as the feature store, so a map can be inspected with its loader.
bool saveMapToDirectory(const std::string& directory, const StoredMap& map);

/// Reads a directory written by [saveMapToDirectory]. Leaves `map` untouched when the header,
/// the version or a feature file does not match.
bool loadMapFromDirectory(const std::string& directory, StoredMap& map);

}  // namespace scand3d::sfm
