#include "tools/pgm.hpp"

#include <fstream>
#include <sstream>

namespace scand3d::tools {

bool writePgm(const std::string& path, int width_px, int height_px,
              const std::vector<std::uint8_t>& pixels) {
    if (width_px <= 0 || height_px <= 0 ||
        pixels.size() != static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px)) {
        return false;
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    out << "P5\n" << width_px << " " << height_px << "\n255\n";
    out.write(reinterpret_cast<const char*>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
    return out.good();
}

bool readPgm(const std::string& path, int& width_px, int& height_px,
             std::vector<std::uint8_t>& pixels) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }

    std::string magic;
    in >> magic;
    if (magic != "P5") {
        return false;
    }

    // Tolerate comments between the header fields, as the PGM format allows.
    auto next_header_value = [&in]() {
        std::string token;
        while (in >> token) {
            if (!token.empty() && token[0] == '#') {
                std::string ignored;
                std::getline(in, ignored);
                continue;
            }
            return token;
        }
        return std::string();
    };

    const std::string width_token = next_header_value();
    const std::string height_token = next_header_value();
    const std::string max_token = next_header_value();
    if (width_token.empty() || height_token.empty() || max_token != "255") {
        return false;
    }
    width_px = std::stoi(width_token);
    height_px = std::stoi(height_token);
    if (width_px <= 0 || height_px <= 0) {
        return false;
    }

    in.get();  // single whitespace after the max value
    std::vector<std::uint8_t> loaded(
        static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px));
    in.read(reinterpret_cast<char*>(loaded.data()),
            static_cast<std::streamsize>(loaded.size()));
    if (!in) {
        return false;
    }
    pixels = std::move(loaded);
    return true;
}

}  // namespace scand3d::tools
