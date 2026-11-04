#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Minimal binary PGM (P5) support. Test and debug tooling only: nothing here is compiled into
// the APK, and the app's real image path is JPEG through Camera2.
namespace scand3d::tools {

bool writePgm(const std::string& path, int width_px, int height_px,
              const std::vector<std::uint8_t>& pixels);

bool readPgm(const std::string& path, int& width_px, int& height_px,
             std::vector<std::uint8_t>& pixels);

}  // namespace scand3d::tools
