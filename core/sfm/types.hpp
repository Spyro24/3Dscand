#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry/linalg.hpp"

// Sparse reconstruction core (roadmap Phase 4). Conventions follow agents.md section 3:
// right-handed, meters, radians, camera-to-world poses stored as float64, camera axes +X right,
// +Y down, +Z forward. Nothing here touches Android or depends on the feature pipeline; callers
// hand in pixels and camera models.
namespace scand3d::sfm {

using geometry::Mat3;
using geometry::Vec2;
using geometry::Vec3;

/// Pinhole intrinsics with square pixels, the only camera model Phase 4 needs.
struct CameraModel {
    double focal_px = 0.0;
    double cx_px = 0.0;
    double cy_px = 0.0;
};

/// Camera-to-world pose: `rotation` maps camera axes to world, `camera_centre_m` is the centre.
struct Pose {
    Mat3 rotation = geometry::identity3();
    Vec3 camera_centre_m{};
};

/// One feature seen in one view. `feature` indexes the caller's per-view feature array.
struct Observation {
    int view = -1;
    int feature = -1;
    Vec2 pixel{};
};

/// A 3D point with every observation that supports it.
struct MapPoint {
    Vec3 position_m{};
    std::vector<Observation> observations;
    bool valid = true;
};

struct ViewState {
    CameraModel camera;
    Pose pose;
    bool registered = false;
    std::size_t feature_count = 0;
};

struct Reconstruction {
    std::vector<ViewState> views;
    std::vector<MapPoint> points;
};

/// Welds a camera model and pose into the world-to-camera transform used everywhere below.
inline Vec3 worldToCamera(const Pose& pose, const Vec3& point_world_m) {
    const Vec3 delta{point_world_m[0] - pose.camera_centre_m[0],
                     point_world_m[1] - pose.camera_centre_m[1],
                     point_world_m[2] - pose.camera_centre_m[2]};
    return geometry::matvec3(geometry::transpose3(pose.rotation), delta);
}

inline Vec3 cameraToWorld(const Pose& pose, const Vec3& point_camera_m) {
    const Vec3 rotated = geometry::matvec3(pose.rotation, point_camera_m);
    return {rotated[0] + pose.camera_centre_m[0], rotated[1] + pose.camera_centre_m[1],
            rotated[2] + pose.camera_centre_m[2]};
}

/// Projects a world point. False when the point is behind the camera or not finite.
inline bool projectPoint(
    const CameraModel& camera,
    const Pose& pose,
    const Vec3& point_world_m,
    Vec2& out_px,
    double& depth_m) {
    const Vec3 point_camera = worldToCamera(pose, point_world_m);
    depth_m = point_camera[2];
    if (!(depth_m > 1e-9) || !std::isfinite(depth_m)) {
        return false;
    }
    out_px = {camera.focal_px * point_camera[0] / depth_m + camera.cx_px,
              camera.focal_px * point_camera[1] / depth_m + camera.cy_px};
    return std::isfinite(out_px[0]) && std::isfinite(out_px[1]);
}

/// Rodrigues rotation from an axis-angle vector (direction is the axis, length the angle).
inline Mat3 rotationFromAxisAngle(const Vec3& axis_angle_rad) {
    const double angle = std::sqrt(geometry::dot3(axis_angle_rad, axis_angle_rad));
    if (!(angle > 1e-12)) {
        return geometry::identity3();
    }
    const Vec3 axis{axis_angle_rad[0] / angle, axis_angle_rad[1] / angle,
                    axis_angle_rad[2] / angle};
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;
    return {c + axis[0] * axis[0] * t, axis[0] * axis[1] * t - axis[2] * s,
            axis[0] * axis[2] * t + axis[1] * s,
            axis[1] * axis[0] * t + axis[2] * s, c + axis[1] * axis[1] * t,
            axis[1] * axis[2] * t - axis[0] * s,
            axis[2] * axis[0] * t - axis[1] * s, axis[2] * axis[1] * t + axis[0] * s,
            c + axis[2] * axis[2] * t};
}

/// Squared pixel error of one observation, or a large value when it cannot be projected.
inline double squaredReprojectionPx(
    const CameraModel& camera,
    const Pose& pose,
    const Vec3& point_world_m,
    const Vec2& observed_px) {
    Vec2 projected{};
    double depth = 0.0;
    if (!projectPoint(camera, pose, point_world_m, projected, depth)) {
        return 1e18;
    }
    const double dx = projected[0] - observed_px[0];
    const double dy = projected[1] - observed_px[1];
    return dx * dx + dy * dy;
}

}  // namespace scand3d::sfm
