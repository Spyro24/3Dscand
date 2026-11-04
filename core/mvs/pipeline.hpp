#pragma once

#include <atomic>
#include <functional>
#include <vector>

#include "mvs/consistency.hpp"
#include "mvs/fusion.hpp"
#include "mvs/patchmatch.hpp"
#include "mvs/types.hpp"

namespace scand3d::mvs {

/// Quality preset (roadmap 6.4). The working width is the long edge the depth estimation runs at;
/// the captured frames stay untouched, so a draft and a high run can be compared on one capture.
enum class MvsQuality {
    Draft,
    Normal,
    High,
};

/// Long edge, in pixels, that the depth estimation runs at for a preset.
int workingWidthPx(MvsQuality quality);

/// One captured view for the dense stage: the gray image at capture resolution plus its pose.
struct MvsViewInput {
    feature::GrayImage gray;
    CameraModel camera;
    Pose pose;
};

struct MvsOptions {
    int working_width_px = 320;
    PatchMatchOptions patch_match{};
    ConsistencyOptions consistency{};
    FusionOptions fusion{};
    /// Sparse map points, when the caller has them: they narrow the depth search to the range the
    /// reconstruction actually occupies. Empty falls back to the patch-match depth range.
    std::vector<Vec3> depth_hint_points_m;
    /// The hint range is expanded by this factor on both sides before it is used.
    double hint_spread = 1.3;
};

struct MvsStats {
    int views = 0;
    int processed_views = 0;
    int depth_pixels = 0;
    int consistent_pixels = 0;
    int fused_points = 0;
    double milliseconds = 0.0;
};

struct MvsResult {
    std::vector<FusedPoint> points;
    std::vector<DepthView> views;
    MvsStats stats;
    bool cancelled = false;
};

/// Cancellation and progress (roadmap 6.5). `progress` receives a value in 0..1; `cancel` is polled
/// between views and inside the patch-match sweeps.
struct MvsControl {
    const std::atomic<bool>* cancel = nullptr;
    std::function<void(double)> progress = nullptr;
};

/// Accepts already-oriented views: downscales them to the working resolution, estimates a depth
/// map per view against its nearest neighbour, filters by consistency and fuses the result.
MvsResult reconstructDense(
    const std::vector<MvsViewInput>& views,
    const MvsOptions& options = {},
    const MvsControl& control = {});

}  // namespace scand3d::mvs
