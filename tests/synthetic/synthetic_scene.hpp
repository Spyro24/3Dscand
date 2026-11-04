#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

// Synthetic scenes for tests: a known mesh observed by simulated cameras on a turntable ring.
// Everything is deterministic, so tests assert against exact ground truth instead of tolerances.
// Conventions follow agents.md section 3: right-handed, meters, radians, camera-to-world poses
// stored as float64. Camera axes follow OpenCV: +X right, +Y down, +Z forward.
namespace scand3d::synthetic {

using Vec3 = std::array<double, 3>;   // meters
using Mat4 = std::array<double, 16>;  // row-major, camera-to-world

struct Vertex {
    Vec3 position_m{};
    Vec3 normal{};
    std::uint8_t albedo = 255;  // grayscale 0..255
};

struct Triangle {
    std::array<int, 3> vertex_index{};
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
};

struct Camera {
    int width_px = 0;
    int height_px = 0;
    double focal_px = 0.0;  // square pixels
    double cx_px = 0.0;
    double cy_px = 0.0;
    Mat4 pose_camera_to_world{};
};

struct Image {
    int width_px = 0;
    int height_px = 0;
    std::vector<std::uint8_t> pixels;  // row-major grayscale, 0 is background

    /// Bounds-checked read; anything outside the image is background.
    std::uint8_t at(int x_px, int y_px) const;
};

struct Projection {
    double u_px = 0.0;
    double v_px = 0.0;
    double depth_m = 0.0;
    bool in_front = false;
};

struct Scene {
    Mesh mesh;
    std::vector<Camera> cameras;
};

/// Asymmetric box-on-box object: enough shape and shading variation to recover pose later.
Mesh makeTurntableObject();

/// Camera-to-world pose looking from `eye_m` at `target_m`, with `up_m` as the world up vector.
Mat4 lookAtCameraToWorld(const Vec3& eye_m, const Vec3& target_m, const Vec3& up_m);

/// Camera on a ring around the object, looking at its centre.
Scene makeTurntableScene(
    int ring_count,
    double radius_m,
    double height_m,
    int width_px,
    int height_px,
    double focal_px);

Projection projectPoint(const Camera& camera, const Vec3& point_m);

/// Grayscale view of the mesh: z-buffer, back-face rejection and Lambert shading, background 0.
Image render(const Scene& scene, int camera_index);

/// Vertex index and pixel position of every vertex in front of the camera and inside the image,
/// used as matching ground truth.
std::vector<std::pair<int, Projection>> visibleVertices(const Scene& scene, int camera_index);

}  // namespace scand3d::synthetic
