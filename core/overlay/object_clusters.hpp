#pragma once

#include <cstddef>
#include <vector>

#include "overlay/overlay_scene.hpp"

namespace scand3d::overlay {

/// Groups the cloud into objects and gives each object its own colour (D055), so the point cloud
/// view shows which points sit on which surface instead of a field of look-alike dots. Two points
/// are linked when they are within `link_distance_m`; a group needs `min_points` members before it
/// counts as an object, which keeps stray outliers from claiming a colour of their own.
struct ObjectClusterOptions {
    double link_distance_m = 0.02;
    int min_points = 12;
    float saturation = 0.85f;
    float value = 0.95f;
};

struct ObjectClusterResult {
    int objects = 0;
    int largest_object = 0;
    std::size_t coloured_points = 0;
};

/// Recolours the points of every object-sized cluster. Unclustered points keep the colour they
/// came with, so the photo texturing still shows through on isolated detail.
ObjectClusterResult colourByObject(
    std::vector<OverlayPoint>& points,
    const ObjectClusterOptions& options = {});

}  // namespace scand3d::overlay
