#include "sfm/map_file.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

#include "feature/feature_store.hpp"

namespace scand3d::sfm {
namespace {

constexpr char kMagic[8] = {'S', '3', 'D', 'M', 'A', 'P', '0', '1'};
constexpr std::uint32_t kVersion = 1;
constexpr std::uint32_t kMaxViews = 10'000;
constexpr std::uint32_t kMaxPoints = 5'000'000;
constexpr std::uint32_t kMaxObservationsPerPoint = 64;

std::filesystem::path featurePath(const std::string& directory, std::size_t index) {
    return std::filesystem::path(directory) / ("view_" + std::to_string(index) + ".feat");
}

}  // namespace

bool saveMapToDirectory(const std::string& directory, const StoredMap& map) {
    if (map.views.empty() ||
        map.views.size() > static_cast<std::size_t>(kMaxViews) ||
        map.points.size() > static_cast<std::size_t>(kMaxPoints)) {
        return false;
    }

    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        return false;
    }

    for (std::size_t index = 0; index < map.views.size(); ++index) {
        if (!feature::saveFeatures(featurePath(directory, index).string(),
                                   map.views[index].features)) {
            return false;
        }
    }

    std::ofstream out(std::filesystem::path(directory) / "map.bin",
                      std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }

    const auto view_count = static_cast<std::uint32_t>(map.views.size());
    const auto point_count = static_cast<std::uint32_t>(map.points.size());
    out.write(kMagic, sizeof(kMagic));
    out.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));
    out.write(reinterpret_cast<const char*>(&view_count), sizeof(view_count));
    out.write(reinterpret_cast<const char*>(&point_count), sizeof(point_count));

    for (const StoredMapView& view : map.views) {
        const std::array<double, 3> camera{view.camera.focal_px, view.camera.cx_px,
                                           view.camera.cy_px};
        out.write(reinterpret_cast<const char*>(camera.data()),
                  static_cast<std::streamsize>(sizeof(double) * camera.size()));
        out.write(reinterpret_cast<const char*>(view.pose.rotation.data()),
                  static_cast<std::streamsize>(sizeof(double) * 9));
        out.write(reinterpret_cast<const char*>(view.pose.camera_centre_m.data()),
                  static_cast<std::streamsize>(sizeof(double) * 3));
        const std::uint8_t registered = view.registered ? 1 : 0;
        out.write(reinterpret_cast<const char*>(&registered), sizeof(registered));
    }

    for (const MapPoint& point : map.points) {
        out.write(reinterpret_cast<const char*>(point.position_m.data()),
                  static_cast<std::streamsize>(sizeof(double) * 3));
        const auto observations = static_cast<std::uint32_t>(point.observations.size());
        out.write(reinterpret_cast<const char*>(&observations), sizeof(observations));
        for (const Observation& observation : point.observations) {
            const std::array<std::int32_t, 2> ids{observation.view, observation.feature};
            out.write(reinterpret_cast<const char*>(ids.data()),
                      static_cast<std::streamsize>(sizeof(std::int32_t) * ids.size()));
            const std::array<double, 2> pixel{observation.pixel[0], observation.pixel[1]};
            out.write(reinterpret_cast<const char*>(pixel.data()),
                      static_cast<std::streamsize>(sizeof(double) * pixel.size()));
        }
        // Points that failed triangulation or outlier removal are kept with their flag, so a
        // loaded map reports exactly the numbers the session reported before it was stored.
        const std::uint8_t valid = point.valid ? 1 : 0;
        out.write(reinterpret_cast<const char*>(&valid), sizeof(valid));
    }
    return out.good();
}

bool loadMapFromDirectory(const std::string& directory, StoredMap& map) {
    std::ifstream in(std::filesystem::path(directory) / "map.bin", std::ios::binary);
    if (!in) {
        return false;
    }

    char magic[8] = {};
    std::uint32_t version = 0;
    std::uint32_t view_count = 0;
    std::uint32_t point_count = 0;
    in.read(magic, sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&view_count), sizeof(view_count));
    in.read(reinterpret_cast<char*>(&point_count), sizeof(point_count));
    if (!in || std::string(magic, sizeof(magic)) != std::string(kMagic, sizeof(kMagic)) ||
        version != kVersion || view_count == 0 || view_count > kMaxViews ||
        point_count > kMaxPoints) {
        return false;
    }

    StoredMap loaded;
    loaded.views.reserve(view_count);
    for (std::uint32_t index = 0; index < view_count; ++index) {
        StoredMapView view;
        std::array<double, 3> camera{};
        in.read(reinterpret_cast<char*>(camera.data()),
                static_cast<std::streamsize>(sizeof(double) * camera.size()));
        in.read(reinterpret_cast<char*>(view.pose.rotation.data()),
                static_cast<std::streamsize>(sizeof(double) * 9));
        in.read(reinterpret_cast<char*>(view.pose.camera_centre_m.data()),
                static_cast<std::streamsize>(sizeof(double) * 3));
        std::uint8_t registered = 0;
        in.read(reinterpret_cast<char*>(&registered), sizeof(registered));
        if (!in) {
            return false;
        }
        view.camera = CameraModel{camera[0], camera[1], camera[2]};
        view.registered = registered != 0;
        if (!feature::loadFeatures(featurePath(directory, index).string(), view.features)) {
            return false;
        }
        loaded.views.push_back(std::move(view));
    }

    loaded.points.reserve(point_count);
    for (std::uint32_t index = 0; index < point_count; ++index) {
        MapPoint point;
        in.read(reinterpret_cast<char*>(point.position_m.data()),
                static_cast<std::streamsize>(sizeof(double) * 3));
        std::uint32_t observations = 0;
        in.read(reinterpret_cast<char*>(&observations), sizeof(observations));
        if (!in || observations > kMaxObservationsPerPoint) {
            return false;
        }
        point.observations.reserve(observations);
        for (std::uint32_t observation_index = 0; observation_index < observations;
             ++observation_index) {
            std::array<std::int32_t, 2> ids{};
            std::array<double, 2> pixel{};
            in.read(reinterpret_cast<char*>(ids.data()),
                    static_cast<std::streamsize>(sizeof(std::int32_t) * ids.size()));
            in.read(reinterpret_cast<char*>(pixel.data()),
                    static_cast<std::streamsize>(sizeof(double) * pixel.size()));
            if (!in || ids[0] < 0 || ids[1] < 0 ||
                static_cast<std::uint32_t>(ids[0]) >= view_count) {
                return false;
            }
            point.observations.push_back(
                Observation{ids[0], ids[1], Vec2{pixel[0], pixel[1]}});
        }
        std::uint8_t valid = 0;
        in.read(reinterpret_cast<char*>(&valid), sizeof(valid));
        if (!in) {
            return false;
        }
        point.valid = valid != 0;
        loaded.points.push_back(std::move(point));
    }

    map = std::move(loaded);
    return true;
}

}  // namespace scand3d::sfm
