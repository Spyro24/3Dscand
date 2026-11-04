#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "geometry/linalg.hpp"
#include "sfm/types.hpp"
#include "synthetic/synthetic_scene.hpp"

// Test-only geometry metrics for the dense stage (roadmap Phase 6). A fused cloud is compared with
// the synthetic ground-truth mesh in both directions, so a cloud that covers one face well and
// nothing else cannot look accurate.
namespace scand3d::testutil {

using geometry::Vec3;

inline Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline double squaredLength(const Vec3& vector) {
    return geometry::dot3(vector, vector);
}

/// Closest-point distance from a point to a triangle (Ericson, Real-Time Collision Detection).
inline double pointTriangleDistanceSquared(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = subtract(b, a);
    const Vec3 ac = subtract(c, a);
    const Vec3 ap = subtract(p, a);
    const double d1 = geometry::dot3(ab, ap);
    const double d2 = geometry::dot3(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return squaredLength(ap);
    }

    const Vec3 bp = subtract(p, b);
    const double d3 = geometry::dot3(ab, bp);
    const double d4 = geometry::dot3(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return squaredLength(bp);
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        const Vec3 closest{a[0] + ab[0] * v, a[1] + ab[1] * v, a[2] + ab[2] * v};
        return squaredLength(subtract(p, closest));
    }

    const Vec3 cp = subtract(p, c);
    const double d5 = geometry::dot3(ab, cp);
    const double d6 = geometry::dot3(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return squaredLength(cp);
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        const Vec3 closest{a[0] + ac[0] * w, a[1] + ac[1] * w, a[2] + ac[2] * w};
        return squaredLength(subtract(p, closest));
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        const Vec3 closest{b[0] + (c[0] - b[0]) * w, b[1] + (c[1] - b[1]) * w,
                           b[2] + (c[2] - b[2]) * w};
        return squaredLength(subtract(p, closest));
    }

    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    const Vec3 closest{a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w,
                       a[2] + ab[2] * v + ac[2] * w};
    return squaredLength(subtract(p, closest));
}

namespace detail {

inline std::int64_t cellKey(int x, int y, int z) {
    return (static_cast<std::int64_t>(x) & 0x1FFFFF) << 42 |
           (static_cast<std::int64_t>(y) & 0x1FFFFF) << 21 |
           (static_cast<std::int64_t>(z) & 0x1FFFFF);
}

/// Uniform grid over the cloud, used for the completeness direction (nearest cloud point).
class CloudIndex {
public:
    CloudIndex(const std::vector<Vec3>& points, double cell_m)
        : points_(points), cell_m_(cell_m > 1e-9 ? cell_m : 1e-3) {
        for (std::size_t index = 0; index < points.size(); ++index) {
            cells_[cellKey(
                static_cast<int>(std::floor(points[index][0] / cell_m_)),
                static_cast<int>(std::floor(points[index][1] / cell_m_)),
                static_cast<int>(std::floor(points[index][2] / cell_m_)))]
                .push_back(static_cast<int>(index));
        }
    }

    double nearestDistance(const Vec3& point) const {
        if (points_.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        const int base_x = static_cast<int>(std::floor(point[0] / cell_m_));
        const int base_y = static_cast<int>(std::floor(point[1] / cell_m_));
        const int base_z = static_cast<int>(std::floor(point[2] / cell_m_));
        double best = std::numeric_limits<double>::infinity();
        for (int radius = 0; radius <= 128; ++radius) {
            for (int dz = -radius; dz <= radius; ++dz) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        // Only the cells on this ring are new.
                        const int chebyshev =
                            std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz)));
                        if (chebyshev != radius) {
                            continue;
                        }
                        const auto found =
                            cells_.find(cellKey(base_x + dx, base_y + dy, base_z + dz));
                        if (found == cells_.end()) {
                            continue;
                        }
                        for (const int index : found->second) {
                            const Vec3 delta = subtract(points_[index], point);
                            best = std::min(best, squaredLength(delta));
                        }
                    }
                }
            }
            if (std::isfinite(best) && best <= (radius * cell_m_) * (radius * cell_m_)) {
                break;
            }
        }
        return std::sqrt(best);
    }

private:
    const std::vector<Vec3>& points_;
    double cell_m_ = 0.005;
    std::unordered_map<std::int64_t, std::vector<int>> cells_;
};

}  // namespace detail

struct ChamferResult {
    /// Mean distance from a fused point to the ground-truth surface.
    double accuracy_m = 0.0;
    /// Mean distance from a sampled surface point to the nearest fused point.
    double completeness_m = 0.0;
    double symmetric_m = 0.0;
    /// Surface area the completeness number was measured over, in square meters.
    double measured_area_m2 = 0.0;
};

/// Compares a cloud with a mesh in both directions. `surface_samples` mesh points are drawn with a
/// fixed pseudorandom sequence, so repeated runs report the same number.
inline ChamferResult chamferToMesh(
    const std::vector<Vec3>& points,
    const synthetic::Mesh& mesh,
    int surface_samples = 12000);

/// Completeness only over triangles at least one camera can see: a turntable capture never observes
/// the underside of an object, and counting that surface would measure the capture geometry, not
/// the reconstruction. A triangle counts as observable when some camera stands on its front side.
inline ChamferResult chamferToMesh(
    const std::vector<Vec3>& points,
    const synthetic::Mesh& mesh,
    const std::vector<sfm::Pose>& visible_from,
    int surface_samples = 12000) {
    ChamferResult result;
    if (points.empty() || mesh.triangles.empty()) {
        result.accuracy_m = std::numeric_limits<double>::infinity();
        result.completeness_m = std::numeric_limits<double>::infinity();
        result.symmetric_m = std::numeric_limits<double>::infinity();
        return result;
    }

    double accuracy_sum = 0.0;
    for (const Vec3& point : points) {
        double best = std::numeric_limits<double>::infinity();
        for (const synthetic::Triangle& triangle : mesh.triangles) {
            const synthetic::Vertex& a = mesh.vertices[triangle.vertex_index[0]];
            const synthetic::Vertex& b = mesh.vertices[triangle.vertex_index[1]];
            const synthetic::Vertex& c = mesh.vertices[triangle.vertex_index[2]];
            best = std::min(
                best, pointTriangleDistanceSquared(point, a.position_m, b.position_m, c.position_m));
        }
        accuracy_sum += std::sqrt(best);
    }
    result.accuracy_m = accuracy_sum / points.size();

    std::vector<std::uint8_t> observable(mesh.triangles.size(), 1);
    if (!visible_from.empty()) {
        for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
            const synthetic::Triangle& triangle = mesh.triangles[index];
            const Vec3& a = mesh.vertices[triangle.vertex_index[0]].position_m;
            const Vec3& b = mesh.vertices[triangle.vertex_index[1]].position_m;
            const Vec3& c = mesh.vertices[triangle.vertex_index[2]].position_m;
            const Vec3 normal = geometry::cross3(subtract(b, a), subtract(c, a));
            const Vec3 centre{
                (a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0,
                (a[2] + b[2] + c[2]) / 3.0};
            bool seen = false;
            for (const sfm::Pose& pose : visible_from) {
                const Vec3 to_camera = subtract(pose.camera_centre_m, centre);
                if (geometry::dot3(normal, to_camera) > 0.0) {
                    seen = true;
                    break;
                }
            }
            observable[index] = seen ? 1 : 0;
        }
    }

    std::vector<double> cumulative_area(mesh.triangles.size(), 0.0);
    double total_area = 0.0;
    for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
        if (!observable[index]) {
            cumulative_area[index] = total_area;
            continue;
        }
        const synthetic::Triangle& triangle = mesh.triangles[index];
        const Vec3& a = mesh.vertices[triangle.vertex_index[0]].position_m;
        const Vec3& b = mesh.vertices[triangle.vertex_index[1]].position_m;
        const Vec3& c = mesh.vertices[triangle.vertex_index[2]].position_m;
        const Vec3 ab = subtract(b, a);
        const Vec3 ac = subtract(c, a);
        const Vec3 cross = geometry::cross3(ab, ac);
        total_area += 0.5 * std::sqrt(squaredLength(cross));
        cumulative_area[index] = total_area;
    }
    result.measured_area_m2 = total_area;

    double minimum_cloud_extent = std::numeric_limits<double>::infinity();
    for (std::size_t axis = 0; axis < 3; ++axis) {
        double low = std::numeric_limits<double>::infinity();
        double high = -std::numeric_limits<double>::infinity();
        for (const Vec3& point : points) {
            low = std::min(low, point[axis]);
            high = std::max(high, point[axis]);
        }
        minimum_cloud_extent = std::min(minimum_cloud_extent, high - low);
    }
    const double cell_m = std::max(0.002, minimum_cloud_extent / 48.0);
    const detail::CloudIndex index(points, cell_m);

    std::uint32_t state = 0x1234567u;
    auto next = [&state]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<double>(state >> 8) / static_cast<double>(1u << 24);
    };

    double completeness_sum = 0.0;
    int measured_samples = 0;
    for (int sample = 0; sample < surface_samples; ++sample) {
        const double target = next() * total_area;
        const auto found = std::upper_bound(cumulative_area.begin(), cumulative_area.end(), target);
        const std::size_t triangle_index = std::min(
            static_cast<std::size_t>(std::distance(cumulative_area.begin(), found)),
            mesh.triangles.size() - 1);
        if (!observable[triangle_index]) {
            continue;
        }
        const synthetic::Triangle& triangle = mesh.triangles[triangle_index];
        const Vec3& a = mesh.vertices[triangle.vertex_index[0]].position_m;
        const Vec3& b = mesh.vertices[triangle.vertex_index[1]].position_m;
        const Vec3& c = mesh.vertices[triangle.vertex_index[2]].position_m;
        double u = next();
        double v = next();
        if (u + v > 1.0) {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        const Vec3 point{
            a[0] + (b[0] - a[0]) * u + (c[0] - a[0]) * v,
            a[1] + (b[1] - a[1]) * u + (c[1] - a[1]) * v,
            a[2] + (b[2] - a[2]) * u + (c[2] - a[2]) * v};
        completeness_sum += index.nearestDistance(point);
        ++measured_samples;
    }
    result.completeness_m = measured_samples > 0 ? completeness_sum / measured_samples
                                                 : std::numeric_limits<double>::infinity();
    result.symmetric_m = 0.5 * (result.accuracy_m + result.completeness_m);
    return result;
}


inline ChamferResult chamferToMesh(
    const std::vector<Vec3>& points,
    const synthetic::Mesh& mesh,
    int surface_samples) {
    return chamferToMesh(points, mesh, std::vector<sfm::Pose>{}, surface_samples);
}

}  // namespace scand3d::testutil
