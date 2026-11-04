#include "sfm/report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace scand3d::sfm {

ReconstructionReport summarize(const Reconstruction& reconstruction) {
    ReconstructionReport report;
    report.views_total = static_cast<int>(reconstruction.views.size());

    std::vector<double> errors;
    for (const ViewState& view : reconstruction.views) {
        if (view.registered) {
            ++report.views_registered;
        }
    }

    for (const MapPoint& point : reconstruction.points) {
        if (!point.valid) {
            continue;
        }
        ++report.points;
        report.observations += static_cast<int>(point.observations.size());
        if (point.observations.size() < 3) {
            ++report.weakly_supported_points;
        }
        for (const Observation& observation : point.observations) {
            const ViewState& view = reconstruction.views[static_cast<std::size_t>(observation.view)];
            if (!view.registered) {
                continue;
            }
            const double squared =
                squaredReprojectionPx(view.camera, view.pose, point.position_m, observation.pixel);
            if (squared < 1e18) {
                errors.push_back(std::sqrt(squared));
            }
        }
    }

    if (!errors.empty()) {
        double sum = 0.0;
        double maximum = 0.0;
        for (const double error : errors) {
            sum += error;
            maximum = std::max(maximum, error);
        }
        std::sort(errors.begin(), errors.end());
        report.mean_reprojection_px = sum / static_cast<double>(errors.size());
        report.median_reprojection_px = errors[errors.size() / 2];
        report.max_reprojection_px = maximum;
    }

    return report;
}

std::string ReconstructionReport::summary() const {
    char buffer[320] = {};
    std::snprintf(buffer, sizeof(buffer),
                  "views %d/%d, points %d (weak %d), observations %d, reprojection mean %.3f px "
                  "median %.3f px max %.3f px",
                  views_registered, views_total, points, weakly_supported_points, observations,
                  mean_reprojection_px, median_reprojection_px, max_reprojection_px);
    return std::string(buffer);
}

}  // namespace scand3d::sfm
