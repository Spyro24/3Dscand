// Desktop-only report (never part of the APK): replays a folder of gray frames through the sparse
// map and then the tracker, so the real-data tracking behaviour can be measured without a phone in
// hand. Frames are PGM (P5) files named so that a lexical sort is capture order.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "sfm/session.hpp"
#include "sfm/tracker.hpp"
#include "tools/pgm.hpp"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::printf("usage: tracking_report <dir> <map_views> <focal_px> [max_tracked] [no_edges]\n");
        return 2;
    }
    const std::string directory = argv[1];
    const int map_views = std::atoi(argv[2]);
    const double focal_px = std::atof(argv[3]);
    const int max_tracked = argc > 4 ? std::atoi(argv[4]) : 1000000;

    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pgm") {
            paths.push_back(entry.path().string());
        }
    }
    std::sort(paths.begin(), paths.end());
    if (paths.empty()) {
        std::printf("no pgm frames in %s\n", directory.c_str());
        return 1;
    }

    std::vector<int> widths;
    std::vector<int> heights;
    std::vector<std::vector<std::uint8_t>> frames;
    for (const std::string& path : paths) {
        int width_px = 0;
        int height_px = 0;
        std::vector<std::uint8_t> pixels;
        if (!scand3d::tools::readPgm(path, width_px, height_px, pixels)) {
            std::printf("cannot read %s\n", path.c_str());
            continue;
        }
        widths.push_back(width_px);
        heights.push_back(height_px);
        frames.push_back(std::move(pixels));
    }
    if (static_cast<int>(frames.size()) <= map_views) {
        std::printf("only %zu frames, need more than %d map views\n", frames.size(), map_views);
        return 1;
    }
    std::printf("frames %zu at %dx%d, focal %.1f px, map from the first %d\n", frames.size(),
                widths[0], heights[0], focal_px, map_views);

    // Contour points are extracted on every frame; passing 1 here turns them off, which is how the
    // corner-first path is compared against the contour fallback (D055).
    const int disable_edges = argc > 5 ? std::atoi(argv[5]) : 0;
    scand3d::feature::FeatureParams feature_params;
    if (disable_edges != 0) {
        feature_params.max_edge_points_per_level = 0;
    }
    scand3d::sfm::Session session(scand3d::sfm::SfmOptions{}, feature_params);
    for (int index = 0; index < map_views; ++index) {
        if (session.addView(widths[static_cast<std::size_t>(index)],
                            heights[static_cast<std::size_t>(index)],
                            frames[static_cast<std::size_t>(index)].data(), focal_px,
                            widths[static_cast<std::size_t>(index)] / 2.0,
                            heights[static_cast<std::size_t>(index)] / 2.0) < 0) {
            std::printf("map view %d rejected\n", index);
        }
    }
    const scand3d::sfm::ReconstructionReport report = session.run();
    std::printf("map: %d views, %d registered, %d points\n", session.viewCount(),
                report.views_registered, session.pointCount());
    // Feature counts separate "the image is too dark to detect corners" from "matching failed".
    std::printf("feature counts:");
    for (const auto& set : session.features()) {
        std::printf(" %zu", set.size());
    }
    std::printf("\n");

    const std::shared_ptr<const scand3d::sfm::Tracker> tracker = session.snapshot();
    if (tracker == nullptr || tracker->empty()) {
        std::printf("no snapshot to track against\n");
        return 1;
    }

    int tracked = 0;
    int tracked_frames = 0;
    int attempted = 0;
    for (std::size_t index = 0; index < frames.size() && attempted < max_tracked; ++index) {
        const bool in_map = index < static_cast<std::size_t>(map_views);
        if (!in_map) {
            ++attempted;
        }
        scand3d::sfm::Pose pose;
        int inliers = 0;
        const bool placed = tracker->track(
            widths[index], heights[index], frames[index].data(), focal_px, widths[index] / 2.0,
            heights[index] / 2.0, pose, inliers);
        if (!in_map || inliers > 0) {
            std::printf("frame %2zu %s inliers %3d centre (%7.3f %7.3f %7.3f)\n", index,
                        in_map ? "map " : "live", placed ? inliers : 0,
                        pose.camera_centre_m[0], pose.camera_centre_m[1],
                        pose.camera_centre_m[2]);
        }
        if (!in_map) {
            ++tracked_frames;
            if (placed && inliers > 0) {
                ++tracked;
            }
        }
    }
    std::printf("tracked %d of %d live frames\n", tracked, tracked_frames);
    return tracked > 0 ? 0 : 1;
}
