// Desktop debugging tool (roadmap 3.5): reads two PGM images, runs the Phase 3 pipeline and
// writes a PPM with the images side by side and one coloured line per match.
//
//   match_visualizer <first.pgm> <second.pgm> <output.ppm>

#include <cstdio>
#include <string>
#include <vector>

#include "feature/feature_extractor.hpp"
#include "feature/gray_image.hpp"
#include "feature/matcher.hpp"
#include "tools/match_overlay.hpp"
#include "tools/pgm.hpp"

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr, "usage: %s <first.pgm> <second.pgm> <output.ppm>\n", argv[0]);
        return 2;
    }

    int first_width = 0;
    int first_height = 0;
    std::vector<std::uint8_t> first_pixels;
    if (!scand3d::tools::readPgm(argv[1], first_width, first_height, first_pixels)) {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }

    int second_width = 0;
    int second_height = 0;
    std::vector<std::uint8_t> second_pixels;
    if (!scand3d::tools::readPgm(argv[2], second_width, second_height, second_pixels)) {
        std::fprintf(stderr, "cannot read %s\n", argv[2]);
        return 1;
    }

    scand3d::feature::GrayImage first{first_width, first_height, std::move(first_pixels)};
    scand3d::feature::GrayImage second{second_width, second_height, std::move(second_pixels)};

    const scand3d::feature::FeatureSet first_features = scand3d::feature::extractFeatures(first);
    const scand3d::feature::FeatureSet second_features = scand3d::feature::extractFeatures(second);
    const auto matches = scand3d::feature::matchBruteForce(first_features, second_features);

    const auto overlay =
        scand3d::tools::renderMatchOverlay(first, second, first_features, second_features, matches);
    if (!scand3d::tools::writePpm(argv[3], overlay)) {
        std::fprintf(stderr, "cannot write %s\n", argv[3]);
        return 1;
    }

    std::printf("features: %zu / %zu, matches: %zu -> %s (%dx%d)\n",
                first_features.keypoints.size(), second_features.keypoints.size(), matches.size(),
                argv[3], overlay.width_px, overlay.height_px);
    return 0;
}
