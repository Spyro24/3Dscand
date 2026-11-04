#pragma once

#include <string>

#include "feature/features.hpp"

namespace scand3d::feature {

/// Per-frame feature storage for Phase 4. The format is a fixed little-endian record (magic,
/// version, count, then keypoints and descriptors), because both ends are the same build and a
/// versioned binary blob is smaller and less error-prone than a text format.
bool saveFeatures(const std::string& path, const FeatureSet& features);

/// Loads a file written by [saveFeatures]. Returns false and leaves `features` untouched when the
/// magic, version, count or file size does not match.
bool loadFeatures(const std::string& path, FeatureSet& features);

}  // namespace scand3d::feature
