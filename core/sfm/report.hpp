#pragma once

#include <string>

#include "sfm/types.hpp"

namespace scand3d::sfm {

/// Readout of one reconstruction (roadmap 4.6). Every number is derived from the reconstruction
/// itself, never from the inputs, so it reports what was actually estimated.
struct ReconstructionReport {
    int views_total = 0;
    int views_registered = 0;
    int points = 0;
    int observations = 0;
    double mean_reprojection_px = 0.0;
    double median_reprojection_px = 0.0;
    double max_reprojection_px = 0.0;
    /// Points seen by fewer than three views; they carry no redundancy.
    int weakly_supported_points = 0;

    /// One-line, stable text form for logs and the reconstruction notes.
    std::string summary() const;
};

ReconstructionReport summarize(const Reconstruction& reconstruction);

}  // namespace scand3d::sfm
