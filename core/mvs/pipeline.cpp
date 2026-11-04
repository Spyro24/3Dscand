#include "mvs/pipeline.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

#include "feature/gray_image.hpp"
#include "sfm/types.hpp"

namespace scand3d::mvs {
namespace {

/// Box-resamples to an exact width, keeping the aspect ratio. A box average is the anti-alias
/// filter patch matching needs, and an exact width makes a quality preset mean what it says.
feature::GrayImage resampleToWidth(const feature::GrayImage& image, int target_width_px) {
    if (image.width_px <= target_width_px || target_width_px <= 0) {
        return image;
    }
    feature::GrayImage scaled;
    scaled.width_px = target_width_px;
    scaled.height_px = std::max(
        1, static_cast<int>(std::lround(
               static_cast<double>(image.height_px) * target_width_px / image.width_px)));
    scaled.pixels.assign(
        static_cast<std::size_t>(scaled.width_px) * scaled.height_px, 0);

    const double step_x = static_cast<double>(image.width_px) / scaled.width_px;
    const double step_y = static_cast<double>(image.height_px) / scaled.height_px;
    for (int y = 0; y < scaled.height_px; ++y) {
        const int y0 = static_cast<int>(y * step_y);
        const int y1 = std::min(
            image.height_px, std::max(y0 + 1, static_cast<int>((y + 1) * step_y)));
        for (int x = 0; x < scaled.width_px; ++x) {
            const int x0 = static_cast<int>(x * step_x);
            const int x1 = std::min(
                image.width_px, std::max(x0 + 1, static_cast<int>((x + 1) * step_x)));
            int sum = 0;
            int count = 0;
            for (int source_y = y0; source_y < y1; ++source_y) {
                for (int source_x = x0; source_x < x1; ++source_x) {
                    sum += image.at(source_x, source_y);
                    ++count;
                }
            }
            scaled.pixels[static_cast<std::size_t>(y) * scaled.width_px + x] =
                static_cast<std::uint8_t>(count > 0 ? (sum + count / 2) / count : 0);
        }
    }
    return scaled;
}

/// Depth range from the sparse map: the range the reconstruction covers, widened per view. Fewer
/// than three points in front of a view falls back to the patch-match defaults.
bool hintRangeFor(
    const std::vector<Vec3>& hints,
    const CameraModel& camera,
    const Pose& pose,
    int width_px,
    int height_px,
    double spread,
    double& out_min_depth,
    double& out_max_depth) {
    if (hints.size() < 3 || !(spread >= 1.0)) {
        return false;
    }
    std::vector<double> depths;
    depths.reserve(hints.size());
    for (const Vec3& point : hints) {
        geometry::Vec2 pixel{};
        double depth = 0.0;
        if (!sfm::projectPoint(camera, pose, point, pixel, depth)) {
            continue;
        }
        if (pixel[0] < 0.0 || pixel[1] < 0.0 || pixel[0] >= width_px || pixel[1] >= height_px) {
            continue;
        }
        depths.push_back(depth);
    }
    if (depths.size() < 3) {
        return false;
    }
    std::sort(depths.begin(), depths.end());
    const std::size_t margin = depths.size() / 20;
    const double low = depths[margin];
    const double high = depths[depths.size() - 1 - margin];
    out_min_depth = low / spread;
    out_max_depth = high * spread;
    return out_min_depth > 0.0 && out_max_depth > out_min_depth;
}

int nearestView(const std::vector<MvsViewInput>& views, int index) {
    int best = -1;
    double best_distance = 0.0;
    for (int other = 0; other < static_cast<int>(views.size()); ++other) {
        if (other == index) {
            continue;
        }
        const Vec3& a = views[index].pose.camera_centre_m;
        const Vec3& b = views[other].pose.camera_centre_m;
        const double dx = a[0] - b[0];
        const double dy = a[1] - b[1];
        const double dz = a[2] - b[2];
        const double distance = dx * dx + dy * dy + dz * dz;
        if (best < 0 || distance < best_distance) {
            best = other;
            best_distance = distance;
        }
    }
    return best;
}

/// Runs `task(0..count-1)` on a small pool. Both depth estimation and consistency are per view and
/// independent, and every estimate seeds its own random sequence, so the result does not depend on
/// the thread count. agents.md lists the MVS worker pool among the app's threads.
template <typename Task>
void runParallel(int count, Task task) {
    if (count <= 1) {
        for (int index = 0; index < count; ++index) {
            task(index);
        }
        return;
    }
    const unsigned reported = std::thread::hardware_concurrency();
    const int workers = std::min(
        count, std::max(1, std::min(8, static_cast<int>(reported == 0u ? 1u : reported))));
    if (workers <= 1) {
        for (int index = 0; index < count; ++index) {
            task(index);
        }
        return;
    }
    std::atomic<int> next{0};
    std::vector<std::thread> threads;
    threads.reserve(static_cast<std::size_t>(workers));
    for (int worker = 0; worker < workers; ++worker) {
        threads.emplace_back([&task, &next, count]() {
            for (;;) {
                const int index = next.fetch_add(1);
                if (index >= count) {
                    return;
                }
                task(index);
            }
        });
    }
    for (std::thread& thread : threads) {
        thread.join();
    }
}

int countValid(const DepthMap& map) {
    int count = 0;
    for (int y = 0; y < map.height_px; ++y) {
        for (int x = 0; x < map.width_px; ++x) {
            if (map.valid(x, y)) {
                ++count;
            }
        }
    }
    return count;
}

}  // namespace

int workingWidthPx(MvsQuality quality) {
    switch (quality) {
        case MvsQuality::Draft:
            return 160;
        case MvsQuality::High:
            return 480;
        case MvsQuality::Normal:
        default:
            return 320;
    }
}

MvsResult reconstructDense(
    const std::vector<MvsViewInput>& views,
    const MvsOptions& options,
    const MvsControl& control) {
    MvsResult result;
    const auto started = std::chrono::steady_clock::now();
    result.stats.views = static_cast<int>(views.size());
    if (views.size() < 2) {
        return result;
    }
    auto cancelled = [&control]() {
        return control.cancel != nullptr && control.cancel->load();
    };
    auto report = [&control](double fraction) {
        if (control.progress) {
            control.progress(std::min(1.0, std::max(0.0, fraction)));
        }
    };

    // Working resolution: one reduction for every view, because the depth maps are compared
    // across views (roadmap 6.4).
    const int target = std::max(32, options.working_width_px);
    result.views.resize(views.size());
    for (std::size_t index = 0; index < views.size(); ++index) {
        const feature::GrayImage scaled_gray = resampleToWidth(views[index].gray, target);
        const double factor =
            views[index].gray.empty()
                ? 1.0
                : static_cast<double>(scaled_gray.width_px) / views[index].gray.width_px;
        DepthView& depth_view = result.views[index];
        depth_view.gray = scaled_gray;
        depth_view.camera = scaledCamera(views[index].camera, factor);
        depth_view.pose = views[index].pose;
        depth_view.depth = DepthMap::sized(scaled_gray.width_px, scaled_gray.height_px);
    }

    // Phase 1: one depth map per view against its nearest neighbour. Each view owns its own depth
    // map, so the views run concurrently; progress is reported as views finish.
    const int view_count = static_cast<int>(result.views.size());
    std::mutex progress_mutex;
    std::atomic<int> processed{0};
    std::atomic<int> estimated_pixels{0};
    std::atomic<int> finished{0};
    runParallel(view_count, [&](int index) {
        if (cancelled()) {
            return;
        }
        const int neighbour = nearestView(views, index);
        if (neighbour < 0) {
            return;
        }
        DepthView& view = result.views[static_cast<std::size_t>(index)];
        const DepthView& other = result.views[static_cast<std::size_t>(neighbour)];
        if (view.gray.empty() || other.gray.empty()) {
            return;
        }

        PatchMatchOptions patch_options = options.patch_match;
        double minimum_depth = patch_options.min_depth_m;
        double maximum_depth = patch_options.max_depth_m;
        if (hintRangeFor(
                options.depth_hint_points_m, view.camera, view.pose, view.gray.width_px,
                view.gray.height_px, options.hint_spread, minimum_depth, maximum_depth)) {
            patch_options.min_depth_m = minimum_depth;
            patch_options.max_depth_m = maximum_depth;
        }
        const DepthMap depth = estimateDepth(
            view.gray, other.gray, view.camera, other.camera, relativePose(view.pose, other.pose),
            patch_options, control.cancel);
        if (cancelled()) {
            return;
        }
        view.depth = depth;
        ++processed;
        estimated_pixels += countValid(view.depth);
        std::lock_guard<std::mutex> lock(progress_mutex);
        report(0.6 * static_cast<double>(++finished) / view_count);
    });
    result.stats.processed_views = processed.load();
    result.stats.depth_pixels = estimated_pixels.load();
    if (cancelled()) {
        result.cancelled = true;
    }

    // Phase 2: geometric consistency against the same neighbour, one map per view again.
    if (!result.cancelled) {
        std::atomic<int> consistent{0};
        std::atomic<int> checked{0};
        runParallel(view_count, [&](int index) {
            if (cancelled()) {
                return;
            }
            const int neighbour = nearestView(views, index);
            if (neighbour < 0) {
                return;
            }
            DepthView& view = result.views[static_cast<std::size_t>(index)];
            const DepthView& other = result.views[static_cast<std::size_t>(neighbour)];
            if (view.depth.empty() || other.depth.empty()) {
                return;
            }
            filterByConsistency(
                view.depth, other.depth, view.camera, other.camera, view.pose, other.pose,
                options.consistency);
            consistent += countValid(view.depth);
            std::lock_guard<std::mutex> lock(progress_mutex);
            report(0.6 + 0.3 * static_cast<double>(++checked) / view_count);
        });
        result.stats.consistent_pixels = consistent.load();
        if (cancelled()) {
            result.cancelled = true;
        }
    }

    // Phase 3: fuse into one cloud.
    if (!result.cancelled) {
        FusionResult fused = fuseDepths(result.views, options.fusion);
        result.points = std::move(fused.points);
        result.stats.fused_points = fused.fused_points;
        report(1.0);
    }

    result.stats.milliseconds = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    return result;
}

}  // namespace scand3d::mvs
