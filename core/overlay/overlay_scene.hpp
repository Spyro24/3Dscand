#pragma once

#include <cstddef>
#include <vector>

#include "geometry/linalg.hpp"

// Sparse colour overlay (roadmap Phase 5). The scene is a set of world-space points with sampled
// colours; project() turns them into screen vertices for the GL pass that draws over the
// monochrome preview. Nothing here touches Android, and the module depends only on
// core/geometry, so the projection contract is testable on the host (D031).
namespace scand3d::overlay {

using geometry::Mat3;
using geometry::Vec3;

/// One reconstructed point, in the reconstruction's own world frame (meters) and its sampled
/// colour in linear 0..1 per channel (roadmap 5.3 supplies the colour).
struct OverlayPoint {
    Vec3 position_m{};
    float rgb[3] = {1.0f, 1.0f, 1.0f};
};

/// Screen-space vertex: normalised device coordinates plus colour. Depth is only used to sort.
struct ColoredVertex {
    float ndc[2] = {0.0f, 0.0f};
    float rgb[3] = {1.0f, 1.0f, 1.0f};
    float depth_m = 0.0f;
};

/// Pinhole camera in *displayed preview image pixels* (origin top-left, +x right, +y down) with a
/// camera-to-world pose on the reconstruction's frame. The image size is the size of the image the
/// intrinsics describe; the renderer draws the overlay into the same letterboxed rectangle as the
/// camera image, so NDC maps onto that rectangle one to one (D031).
struct OverlayCamera {
    Mat3 rotation = geometry::identity3();
    Vec3 centre_m{};
    double focal_px = 0.0;
    double cx_px = 0.0;
    double cy_px = 0.0;
    int image_width_px = 0;
    int image_height_px = 0;
};

/// Projects one world point into displayed-image pixels. False when it is behind the camera or
/// the camera cannot project; `depth_m` is the distance along the optical axis.
bool projectToPixel(
    const OverlayCamera& camera,
    const Vec3& point_world_m,
    double& pixel_x,
    double& pixel_y,
    double& depth_m);

class OverlayScene {
public:
    /// Replaces every point. Anything the previous scene held is gone.
    void setPoints(std::vector<OverlayPoint> points);

    std::size_t pointCount() const { return points_.size(); }
    const std::vector<OverlayPoint>& points() const { return points_; }

    /// Points in front of the camera, projected to NDC and sorted far to near so blending needs no
    /// depth buffer. Points behind the camera or a degenerate camera yield no vertices.
    std::vector<ColoredVertex> project(const OverlayCamera& camera) const;

private:
    std::vector<OverlayPoint> points_;
};

}  // namespace scand3d::overlay
