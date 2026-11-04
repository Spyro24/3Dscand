#include "mvs/types.hpp"

#include <algorithm>

namespace scand3d::mvs {

DepthMap DepthMap::sized(int width_px, int height_px) {
    DepthMap map;
    if (width_px <= 0 || height_px <= 0) {
        return map;
    }
    map.width_px = width_px;
    map.height_px = height_px;
    const std::size_t count = static_cast<std::size_t>(width_px) * height_px;
    map.depth_m.assign(count, 0.0f);
    map.normal_camera.assign(count * 3u, 0.0f);
    map.cost.assign(count, 2.0f);
    return map;
}

CameraModel scaledCamera(const CameraModel& camera, double factor) {
    return {camera.focal_px * factor, camera.cx_px * factor, camera.cy_px * factor};
}

}  // namespace scand3d::mvs
