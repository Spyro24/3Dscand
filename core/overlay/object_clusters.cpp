#include "overlay/object_clusters.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace scand3d::overlay {
namespace {

/// Evenly spaced hues for the objects: consecutive golden-angle steps never put two neighbours on
/// the same shade, whatever order the clusters come back in.
constexpr double kHueStep = 0.6180339887498949;

void setHue(float hue, float saturation, float value, float rgb[3]) {
    const float scaled = (hue - std::floor(hue)) * 6.0f;
    const int sector = static_cast<int>(scaled) % 6;
    const float fraction = scaled - std::floor(scaled);
    const float p = value * (1.0f - saturation);
    const float q = value * (1.0f - saturation * fraction);
    const float t = value * (1.0f - saturation * (1.0f - fraction));
    switch (sector) {
        case 0:
            rgb[0] = value;
            rgb[1] = t;
            rgb[2] = p;
            break;
        case 1:
            rgb[0] = q;
            rgb[1] = value;
            rgb[2] = p;
            break;
        case 2:
            rgb[0] = p;
            rgb[1] = value;
            rgb[2] = t;
            break;
        case 3:
            rgb[0] = p;
            rgb[1] = q;
            rgb[2] = value;
            break;
        case 4:
            rgb[0] = t;
            rgb[1] = p;
            rgb[2] = value;
            break;
        default:
            rgb[0] = value;
            rgb[1] = p;
            rgb[2] = q;
            break;
    }
}

int cellIndex(double value, double cell_size) {
    return static_cast<int>(std::floor(value / cell_size));
}

std::int64_t cellKey(int x, int y, int z) {
    return (static_cast<std::int64_t>(x) * 73856093) ^ (static_cast<std::int64_t>(y) * 19349663) ^
           (static_cast<std::int64_t>(z) * 83492791);
}

}  // namespace

ObjectClusterResult colourByObject(
    std::vector<OverlayPoint>& points, const ObjectClusterOptions& options) {
    ObjectClusterResult result;
    if (points.empty() || !(options.link_distance_m > 0.0) || options.min_points < 1) {
        return result;
    }

    const std::size_t count = points.size();
    std::vector<int> parent(count);
    for (std::size_t index = 0; index < count; ++index) {
        parent[index] = static_cast<int>(index);
    }

    auto find_root = [&parent](int index) {
        while (parent[static_cast<std::size_t>(index)] != index) {
            parent[static_cast<std::size_t>(index)] = parent[static_cast<std::size_t>(
                parent[static_cast<std::size_t>(index)])];
            index = parent[static_cast<std::size_t>(index)];
        }
        return index;
    };
    auto unite = [&parent, &find_root](int a, int b) {
        const int root_a = find_root(a);
        const int root_b = find_root(b);
        if (root_a != root_b) {
            parent[static_cast<std::size_t>(root_b)] = root_a;
        }
    };

    // Spatial hash: every point only has to test its own and the neighbouring cells, so the
    // clustering stays linear in the point count.
    const double cell_size = options.link_distance_m;
    const double link_squared = options.link_distance_m * options.link_distance_m;
    std::vector<int> cell_x(count);
    std::vector<int> cell_y(count);
    std::vector<int> cell_z(count);
    std::unordered_map<std::int64_t, std::vector<int>> cells;
    cells.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const Vec3& position = points[index].position_m;
        cell_x[index] = cellIndex(position[0], cell_size);
        cell_y[index] = cellIndex(position[1], cell_size);
        cell_z[index] = cellIndex(position[2], cell_size);
        cells[cellKey(cell_x[index], cell_y[index], cell_z[index])].push_back(
            static_cast<int>(index));
    }

    for (std::size_t index = 0; index < count; ++index) {
        const Vec3& position = points[index].position_m;
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto found = cells.find(
                        cellKey(cell_x[index] + dx, cell_y[index] + dy, cell_z[index] + dz));
                    if (found == cells.end()) {
                        continue;
                    }
                    for (const int other : found->second) {
                        if (other <= static_cast<int>(index)) {
                            continue;
                        }
                        const Vec3& neighbour = points[static_cast<std::size_t>(other)].position_m;
                        const double offset_x = neighbour[0] - position[0];
                        const double offset_y = neighbour[1] - position[1];
                        const double offset_z = neighbour[2] - position[2];
                        if (offset_x * offset_x + offset_y * offset_y + offset_z * offset_z <=
                            link_squared) {
                            unite(static_cast<int>(index), other);
                        }
                    }
                }
            }
        }
    }

    std::vector<int> sizes(count, 0);
    for (std::size_t index = 0; index < count; ++index) {
        ++sizes[static_cast<std::size_t>(find_root(static_cast<int>(index)))];
    }
    std::vector<int> roots;
    for (std::size_t index = 0; index < count; ++index) {
        if (sizes[index] >= options.min_points) {
            roots.push_back(static_cast<int>(index));
        }
    }
    std::sort(roots.begin(), roots.end(), [&sizes](int a, int b) {
        return sizes[static_cast<std::size_t>(a)] > sizes[static_cast<std::size_t>(b)];
    });

    for (std::size_t rank = 0; rank < roots.size(); ++rank) {
        float rgb[3] = {1.0f, 1.0f, 1.0f};
        setHue(
            static_cast<float>(std::fmod(kHueStep * static_cast<double>(rank), 1.0)),
            options.saturation, options.value, rgb);
        for (std::size_t index = 0; index < count; ++index) {
            if (find_root(static_cast<int>(index)) != roots[rank]) {
                continue;
            }
            points[index].rgb[0] = rgb[0];
            points[index].rgb[1] = rgb[1];
            points[index].rgb[2] = rgb[2];
            ++result.coloured_points;
        }
    }
    result.objects = static_cast<int>(roots.size());
    result.largest_object = roots.empty() ? 0 : sizes[static_cast<std::size_t>(roots[0])];
    return result;
}

}  // namespace scand3d::overlay
