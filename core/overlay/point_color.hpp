#pragma once

#include <cstddef>
#include <vector>

#include "overlay/overlay_scene.hpp"

namespace scand3d::overlay {

/// Packed colour image, three bytes per pixel, row major, top-left origin, in the same pixel
/// space as the overlay camera. The reconstruction core itself only ever sees gray images, so the
/// colour comes from the app's keyframe JPEGs through this struct.
struct RgbImage {
    int width_px = 0;
    int height_px = 0;
    std::vector<unsigned char> rgb;

    bool valid() const {
        return width_px > 0 && height_px > 0 &&
               rgb.size() == static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px) *
                                 3u;
    }
};

/// Replaces each point's colour with the per-channel median of a 3x3 keyframe patch around where it
/// projects (roadmap 5.3). The median keeps a single specular or anti-aliased pixel from tinting
/// the point. Points behind the camera, outside the image, or on an unusable keyframe keep the
/// colour they came with. Returns how many points received a sample.
int samplePointColors(
    const RgbImage& keyframe,
    const OverlayCamera& camera,
    std::vector<OverlayPoint>& points);

}  // namespace scand3d::overlay
