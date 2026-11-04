#include "overlay/overlay_scene.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace scand3d::overlay {

void OverlayScene::setPoints(std::vector<OverlayPoint> points) {
    points_ = std::move(points);
}

std::vector<ColoredVertex> OverlayScene::project(const OverlayCamera& camera) const {
    std::vector<ColoredVertex> vertices;
    if (points_.empty() || camera.image_width_px <= 0 || camera.image_height_px <= 0 ||
        !(camera.focal_px > 0.0)) {
        return vertices;
    }

    const Mat3 rotation_transpose = geometry::transpose3(camera.rotation);
    const double half_width = 0.5 * static_cast<double>(camera.image_width_px);
    const double half_height = 0.5 * static_cast<double>(camera.image_height_px);

    vertices.reserve(points_.size());
    for (const OverlayPoint& point : points_) {
        const Vec3 delta{point.position_m[0] - camera.centre_m[0],
                         point.position_m[1] - camera.centre_m[1],
                         point.position_m[2] - camera.centre_m[2]};
        const Vec3 point_camera = geometry::matvec3(rotation_transpose, delta);
        const double depth_m = point_camera[2];
        if (!(depth_m > 1e-9) || !std::isfinite(depth_m)) {
            continue;
        }

        const double pixel_x = camera.focal_px * point_camera[0] / depth_m + camera.cx_px;
        const double pixel_y = camera.focal_px * point_camera[1] / depth_m + camera.cy_px;
        if (!std::isfinite(pixel_x) || !std::isfinite(pixel_y)) {
            continue;
        }

        ColoredVertex vertex;
        vertex.ndc[0] = static_cast<float>(2.0 * pixel_x / camera.image_width_px - 1.0);
        // Image y grows downward and NDC y grows upward, so the sign flips here rather than in
        // every shader that ever draws the overlay.
        vertex.ndc[1] = static_cast<float>(1.0 - 2.0 * pixel_y / camera.image_height_px);
        vertex.rgb[0] = point.rgb[0];
        vertex.rgb[1] = point.rgb[1];
        vertex.rgb[2] = point.rgb[2];
        vertex.depth_m = static_cast<float>(depth_m);
        vertices.push_back(vertex);
    }

    std::stable_sort(
        vertices.begin(), vertices.end(),
        [](const ColoredVertex& a, const ColoredVertex& b) { return a.depth_m > b.depth_m; });
    return vertices;
}

}  // namespace scand3d::overlay
