#include "feature/feature_store.hpp"

#include <cstdint>
#include <fstream>
#include <vector>

namespace scand3d::feature {
namespace {

constexpr char kMagic[8] = {'S', '3', 'D', 'F', 'E', 'A', 'T', '1'};
constexpr std::uint32_t kVersion = 2;
constexpr std::uint32_t kMaxFeatures = 1'000'000;

}  // namespace

bool saveFeatures(const std::string& path, const FeatureSet& features) {
    if (features.keypoints.size() != features.descriptors.size()) {
        return false;  // the two arrays are one dataset; a mismatch is a caller bug
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }

    const auto count = static_cast<std::uint32_t>(features.keypoints.size());
    out.write(kMagic, sizeof(kMagic));
    out.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));

    for (std::size_t i = 0; i < features.keypoints.size(); ++i) {
        const Keypoint& keypoint = features.keypoints[i];
        const auto record = std::array<float, 7>{
            static_cast<float>(keypoint.x_px),
            static_cast<float>(keypoint.y_px),
            static_cast<float>(keypoint.scale),
            static_cast<float>(keypoint.angle_rad),
            static_cast<float>(keypoint.response),
            static_cast<float>(keypoint.edge_direction_rad),
            keypoint.is_edge ? 1.0f : 0.0f,
        };
        out.write(reinterpret_cast<const char*>(record.data()),
                  static_cast<std::streamsize>(sizeof(float) * record.size()));
        out.write(reinterpret_cast<const char*>(features.descriptors[i].data()),
                  static_cast<std::streamsize>(features.descriptors[i].size()));
    }
    return out.good();
}

bool loadFeatures(const std::string& path, FeatureSet& features) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }

    char magic[8] = {};
    std::uint32_t version = 0;
    std::uint32_t count = 0;
    in.read(magic, sizeof(magic));
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!in || std::string(magic, sizeof(magic)) != std::string(kMagic, sizeof(kMagic)) ||
        version != kVersion || count > kMaxFeatures) {
        return false;
    }

    constexpr std::streamoff kHeaderBytes = 8 + 4 + 4;
    constexpr std::streamoff kRecordBytes = sizeof(float) * 7 + 32;
    in.seekg(0, std::ios::end);
    const std::streamoff file_bytes = in.tellg();
    if (file_bytes != kHeaderBytes + static_cast<std::streamoff>(count) * kRecordBytes) {
        return false;
    }
    in.seekg(kHeaderBytes, std::ios::beg);

    FeatureSet loaded;
    loaded.keypoints.reserve(count);
    loaded.descriptors.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        std::array<float, 7> record{};
        in.read(reinterpret_cast<char*>(record.data()),
                static_cast<std::streamsize>(sizeof(float) * record.size()));
        Descriptor descriptor{};
        in.read(reinterpret_cast<char*>(descriptor.data()),
                static_cast<std::streamsize>(descriptor.size()));
        if (!in) {
            return false;
        }
        Keypoint keypoint;
        keypoint.x_px = record[0];
        keypoint.y_px = record[1];
        keypoint.scale = record[2];
        keypoint.angle_rad = record[3];
        keypoint.response = record[4];
        keypoint.edge_direction_rad = record[5];
        keypoint.is_edge = record[6] != 0.0f;
        loaded.keypoints.push_back(keypoint);
        loaded.descriptors.push_back(descriptor);
    }

    features = std::move(loaded);
    return true;
}

}  // namespace scand3d::feature
