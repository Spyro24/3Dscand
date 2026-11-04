#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "sfm/pipeline.hpp"
#include "synthetic/sfm_scan.hpp"

// Desktop-only Phase 4 report (roadmap 4.6): reconstructs the synthetic turntable scan and prints
// the numbers the acceptance line is judged on, including the measured pose error. Built only in
// `nativeTest` and never packaged into the APK.
int main(int argc, char** argv) {
    const int view_count = argc > 1 ? std::atoi(argv[1]) : 16;
    const double noise_px = argc > 2 ? std::atof(argv[2]) : 0.5;
    if (view_count < 3) {
        std::fprintf(stderr, "view count must be at least 3\n");
        return 2;
    }

    const scand3d::synthetic::SfmScan scan =
        scand3d::synthetic::makeLatticeScan(view_count, noise_px);

    scand3d::sfm::SfmOptions options;
    options.two_view.threshold_px = 2.0;
    options.pnp.threshold_px = 3.0;
    options.bundle_adjustment.max_iterations = 30;
    options.keyframe.min_overlap = 0.2;
    options.outlier_reprojection_px = 4.0;

    const auto start = std::chrono::steady_clock::now();
    const scand3d::sfm::SfmResult result =
        scand3d::sfm::reconstruct(scan.frames, scan.pairs, options);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - start)
                                  .count();

    std::printf("views             %d\n", result.report.views_total);
    std::printf("registered        %d\n", result.report.views_registered);
    std::printf("points            %d\n", result.report.points);
    std::printf("observations      %d\n", result.report.observations);
    std::printf("weak points       %d\n", result.report.weakly_supported_points);
    std::printf("reprojection px   mean %.3f median %.3f max %.3f\n",
                result.report.mean_reprojection_px, result.report.median_reprojection_px,
                result.report.max_reprojection_px);
    std::printf("elapsed ms        %.1f\n", elapsed_ms);
    std::printf("summary           %s\n", result.report.summary().c_str());

    if (result.registration_order.size() < 2) {
        std::fprintf(stderr, "no seed pair: reconstruction failed\n");
        return 1;
    }

    std::printf("registration     ");
    for (const int view : result.registration_order) {
        std::printf(" %d", view);
    }
    std::printf("\n");

    const int seed = result.registration_order.front();
    double worst_rotation_deg = 0.0;
    double worst_direction_deg = 0.0;
    for (const int view : result.registration_order) {
        if (view == seed) {
            continue;
        }
        worst_rotation_deg =
            std::max(worst_rotation_deg,
                     scand3d::synthetic::relativeRotationErrorDeg(
                         result.reconstruction, scan.ground_truth, seed, view));
        worst_direction_deg =
            std::max(worst_direction_deg,
                     scand3d::synthetic::relativeCentreDirectionErrorDeg(
                         result.reconstruction, scan.ground_truth, seed, view));
    }
    std::printf("pose error        worst rotation %.3f deg, worst centre direction %.3f deg\n",
                worst_rotation_deg, worst_direction_deg);

    const int accepted = static_cast<int>(std::count_if(
        result.decisions.begin(), result.decisions.end(),
        [](const scand3d::sfm::KeyframeDecision& decision) { return decision.accept; }));
    std::printf("keyframes         %d accepted of %d candidates\n", accepted,
                static_cast<int>(result.decisions.size()));
    for (const scand3d::sfm::KeyframeDecision& decision : result.decisions) {
        std::printf("  view %2d %s rot %.3f base %.3f overlap %.2f %s\n", decision.view,
                    decision.accept ? "accept" : "reject", decision.rotation_rad,
                    decision.baseline_m, decision.overlap, decision.reason);
    }
    return 0;
}
