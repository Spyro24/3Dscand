#include "synthetic/synthetic_scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scand3d::synthetic {
namespace {

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 normalize(const Vec3& a) {
    const double length = std::sqrt(dot(a, a));
    return {a[0] / length, a[1] / length, a[2] / length};
}

Vec3 scaled(const Vec3& a, double factor) {
    return {a[0] * factor, a[1] * factor, a[2] * factor};
}

// Object centre in meters; the turntable cameras aim here.
constexpr double kObjectCentreZ_m = 0.5;

// Adds one quad as two triangles. Each face gets its own vertices so it can carry a flat normal.
void addQuad(Mesh& mesh, const std::array<Vec3, 4>& corners, const Vec3& normal, std::uint8_t albedo) {
    const auto base = static_cast<int>(mesh.vertices.size());
    for (const Vec3& corner : corners) {
        Vertex vertex;
        vertex.position_m = corner;
        vertex.normal = normal;
        vertex.albedo = albedo;
        mesh.vertices.push_back(vertex);
    }
    mesh.triangles.push_back({base + 0, base + 1, base + 2});
    mesh.triangles.push_back({base + 0, base + 2, base + 3});
}

void addBox(Mesh& mesh, const Vec3& min_m, const Vec3& max_m, std::uint8_t albedo) {
    const Vec3 c000{min_m[0], min_m[1], min_m[2]};
    const Vec3 c100{max_m[0], min_m[1], min_m[2]};
    const Vec3 c110{max_m[0], max_m[1], min_m[2]};
    const Vec3 c010{min_m[0], max_m[1], min_m[2]};
    const Vec3 c001{min_m[0], min_m[1], max_m[2]};
    const Vec3 c101{max_m[0], min_m[1], max_m[2]};
    const Vec3 c111{max_m[0], max_m[1], max_m[2]};
    const Vec3 c011{min_m[0], max_m[1], max_m[2]};

    addQuad(mesh, {c001, c101, c111, c011}, {0.0, 0.0, 1.0}, albedo);
    addQuad(mesh, {c000, c010, c110, c100}, {0.0, 0.0, -1.0}, albedo);
    addQuad(mesh, {c100, c110, c111, c101}, {1.0, 0.0, 0.0}, albedo);
    addQuad(mesh, {c000, c001, c011, c010}, {-1.0, 0.0, 0.0}, albedo);
    addQuad(mesh, {c010, c011, c111, c110}, {0.0, 1.0, 0.0}, albedo);
    addQuad(mesh, {c000, c100, c101, c001}, {0.0, -1.0, 0.0}, albedo);
}

}  // namespace

std::uint8_t Image::at(int x_px, int y_px) const {
    if (x_px < 0 || y_px < 0 || x_px >= width_px || y_px >= height_px) {
        return 0;
    }
    return pixels[static_cast<std::size_t>(y_px) * static_cast<std::size_t>(width_px) +
                  static_cast<std::size_t>(x_px)];
}

Mesh makeTurntableObject() {
    Mesh mesh;
    // Two stacked, offset boxes: no axis of symmetry, so a pose estimate is unique.
    addBox(mesh, {-0.5, -0.3, 0.0}, {0.5, 0.3, 0.6}, 210);
    addBox(mesh, {-0.25, -0.15, 0.6}, {0.25, 0.15, 1.0}, 120);
    return mesh;
}

Mat4 lookAtCameraToWorld(const Vec3& eye_m, const Vec3& target_m, const Vec3& up_m) {
    const Vec3 forward = normalize(sub(target_m, eye_m));
    const Vec3 down_world = scaled(up_m, -1.0);
    // Orthogonalise against the viewing direction so the basis stays orthonormal.
    const Vec3 down = normalize(sub(down_world, scaled(forward, dot(down_world, forward))));
    const Vec3 right = normalize(cross(down, forward));

    Mat4 pose{};
    // Column c of the rotation is the camera axis c expressed in world coordinates.
    pose[0] = right[0];
    pose[1] = down[0];
    pose[2] = forward[0];
    pose[3] = eye_m[0];
    pose[4] = right[1];
    pose[5] = down[1];
    pose[6] = forward[1];
    pose[7] = eye_m[1];
    pose[8] = right[2];
    pose[9] = down[2];
    pose[10] = forward[2];
    pose[11] = eye_m[2];
    pose[12] = 0.0;
    pose[13] = 0.0;
    pose[14] = 0.0;
    pose[15] = 1.0;
    return pose;
}

Projection projectPoint(const Camera& camera, const Vec3& point_m) {
    const Mat4& pose = camera.pose_camera_to_world;
    const Vec3 right{pose[0], pose[4], pose[8]};
    const Vec3 down{pose[1], pose[5], pose[9]};
    const Vec3 forward{pose[2], pose[6], pose[10]};
    const Vec3 offset = sub(point_m, {pose[3], pose[7], pose[11]});

    const double x_m = dot(offset, right);
    const double y_m = dot(offset, down);
    const double z_m = dot(offset, forward);

    Projection projection;
    projection.depth_m = z_m;
    projection.in_front = z_m > 0.0;
    if (!projection.in_front) {
        return projection;
    }
    projection.u_px = camera.focal_px * x_m / z_m + camera.cx_px;
    projection.v_px = camera.focal_px * y_m / z_m + camera.cy_px;
    return projection;
}

Scene makeTurntableScene(
    int ring_count,
    double radius_m,
    double height_m,
    int width_px,
    int height_px,
    double focal_px) {
    Scene scene;
    scene.mesh = makeTurntableObject();

    const Vec3 target{0.0, 0.0, kObjectCentreZ_m};
    const double two_pi = 2.0 * std::acos(-1.0);
    scene.cameras.reserve(static_cast<std::size_t>(std::max(ring_count, 0)));
    for (int i = 0; i < ring_count; ++i) {
        const double angle_rad = two_pi * static_cast<double>(i) / static_cast<double>(ring_count);
        const Vec3 eye_m{radius_m * std::cos(angle_rad), radius_m * std::sin(angle_rad), height_m};

        Camera camera;
        camera.width_px = width_px;
        camera.height_px = height_px;
        camera.focal_px = focal_px;
        camera.cx_px = 0.5 * static_cast<double>(width_px);
        camera.cy_px = 0.5 * static_cast<double>(height_px);
        camera.pose_camera_to_world = lookAtCameraToWorld(eye_m, target, {0.0, 0.0, 1.0});
        scene.cameras.push_back(camera);
    }
    return scene;
}

Image render(const Scene& scene, int camera_index) {
    const Camera& camera = scene.cameras.at(static_cast<std::size_t>(camera_index));
    Image image;
    image.width_px = camera.width_px;
    image.height_px = camera.height_px;

    const auto pixel_count = static_cast<std::size_t>(camera.width_px) *
                             static_cast<std::size_t>(camera.height_px);
    image.pixels.assign(pixel_count, 0);
    std::vector<double> depth_buffer(pixel_count, std::numeric_limits<double>::infinity());

    const Vec3 light_direction = normalize({0.3, 0.4, 0.85});
    const Vec3 eye{camera.pose_camera_to_world[3],
                   camera.pose_camera_to_world[7],
                   camera.pose_camera_to_world[11]};
    constexpr double kAmbient = 0.25;

    for (const Triangle& triangle : scene.mesh.triangles) {
        const Vertex& v0 = scene.mesh.vertices[static_cast<std::size_t>(triangle.vertex_index[0])];
        const Vertex& v1 = scene.mesh.vertices[static_cast<std::size_t>(triangle.vertex_index[1])];
        const Vertex& v2 = scene.mesh.vertices[static_cast<std::size_t>(triangle.vertex_index[2])];

        const Projection p0 = projectPoint(camera, v0.position_m);
        const Projection p1 = projectPoint(camera, v1.position_m);
        const Projection p2 = projectPoint(camera, v2.position_m);
        // Triangles crossing the image plane are dropped; the turntable scenes never produce them.
        if (!p0.in_front || !p1.in_front || !p2.in_front) {
            continue;
        }

        // Back faces are rejected so a face seen from behind cannot win the depth test.
        const Vec3 centroid{(v0.position_m[0] + v1.position_m[0] + v2.position_m[0]) / 3.0,
                            (v0.position_m[1] + v1.position_m[1] + v2.position_m[1]) / 3.0,
                            (v0.position_m[2] + v1.position_m[2] + v2.position_m[2]) / 3.0};
        const Vec3 to_eye = sub(eye, centroid);
        if (dot(normalize(v0.normal), to_eye) <= 0.0) {
            continue;
        }

        const double area = (p1.u_px - p0.u_px) * (p2.v_px - p0.v_px) -
                            (p2.u_px - p0.u_px) * (p1.v_px - p0.v_px);
        if (std::abs(area) < 1e-12) {
            continue;
        }

        const double lambert = std::max(0.0, dot(normalize(v0.normal), light_direction));
        const double shade = kAmbient + (1.0 - kAmbient) * lambert;
        const auto intensity = static_cast<std::uint8_t>(
            std::clamp(v0.albedo * shade, 0.0, 255.0));

        const int x_min = std::max(0, static_cast<int>(std::floor(std::min({p0.u_px, p1.u_px, p2.u_px}))));
        const int x_max = std::min(camera.width_px - 1,
                                   static_cast<int>(std::ceil(std::max({p0.u_px, p1.u_px, p2.u_px}))));
        const int y_min = std::max(0, static_cast<int>(std::floor(std::min({p0.v_px, p1.v_px, p2.v_px}))));
        const int y_max = std::min(camera.height_px - 1,
                                   static_cast<int>(std::ceil(std::max({p0.v_px, p1.v_px, p2.v_px}))));

        for (int y_px = y_min; y_px <= y_max; ++y_px) {
            for (int x_px = x_min; x_px <= x_max; ++x_px) {
                const double sample_u = static_cast<double>(x_px) + 0.5;
                const double sample_v = static_cast<double>(y_px) + 0.5;

                const double w0 = ((p1.u_px - sample_u) * (p2.v_px - sample_v) -
                                   (p2.u_px - sample_u) * (p1.v_px - sample_v)) /
                                  area;
                const double w1 = ((p2.u_px - sample_u) * (p0.v_px - sample_v) -
                                   (p0.u_px - sample_u) * (p2.v_px - sample_v)) /
                                  area;
                const double w2 = 1.0 - w0 - w1;
                if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0) {
                    continue;
                }

                const double depth_m =
                    w0 * p0.depth_m + w1 * p1.depth_m + w2 * p2.depth_m;
                const auto index = static_cast<std::size_t>(y_px) *
                                       static_cast<std::size_t>(camera.width_px) +
                                   static_cast<std::size_t>(x_px);
                if (depth_m < depth_buffer[index]) {
                    depth_buffer[index] = depth_m;
                    image.pixels[index] = intensity;
                }
            }
        }
    }
    return image;
}

std::vector<std::pair<int, Projection>> visibleVertices(const Scene& scene, int camera_index) {
    const Camera& camera = scene.cameras.at(static_cast<std::size_t>(camera_index));
    std::vector<std::pair<int, Projection>> visible;
    for (std::size_t i = 0; i < scene.mesh.vertices.size(); ++i) {
        const Projection projection = projectPoint(camera, scene.mesh.vertices[i].position_m);
        if (!projection.in_front) {
            continue;
        }
        if (projection.u_px < 0.0 || projection.v_px < 0.0 ||
            projection.u_px >= static_cast<double>(camera.width_px) ||
            projection.v_px >= static_cast<double>(camera.height_px)) {
            continue;
        }
        visible.emplace_back(static_cast<int>(i), projection);
    }
    return visible;
}

}  // namespace scand3d::synthetic
