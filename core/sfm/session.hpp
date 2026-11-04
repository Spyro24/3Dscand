#pragma once

#include <memory>
#include <string>
#include <vector>

#include "feature/feature_extractor.hpp"
#include "feature/features.hpp"
#include "sfm/pipeline.hpp"
#include "sfm/pnp.hpp"

namespace scand3d::sfm {

class Tracker;

/// Turns captured gray frames into a sparse map and tracks a new frame against it: the
/// features -> matching -> reconstruction orchestration the app was missing (D030). It lives with
/// sfm because that is the module that owns sparse reconstruction; it composes the feature and
/// geometry modules rather than duplicating them.
class Session {
public:
    explicit Session(SfmOptions options = {}, feature::FeatureParams feature_params = {});

    /// Adds a captured view from an 8-bit row-major gray image. Returns its view index, or -1 when
    /// the image is unusable or yields no features.
    int addView(
        int width_px,
        int height_px,
        const unsigned char* gray,
        double focal_px,
        double cx_px,
        double cy_px);

    int viewCount() const;

    /// Extracts features, matches every pair, reconstructs and stores the result. Returns the
    /// report; a view count below two, or no usable pair, yields an empty one.
    ReconstructionReport run();

    const Reconstruction& reconstruction() const { return result_.reconstruction; }

    /// Why each candidate view of the last run did or did not enter the map (empty before a run).
    const std::vector<std::string>& growthTrace() const { return result_.growth_trace; }
    const std::vector<feature::FeatureSet>& features() const { return features_; }

    int pointCount() const;

    /// Writes valid points as xyz, up to `max_points`. Returns how many were written.
    int copyPoints(double* positions_m, int max_points) const;

    int cameraCount() const;

    /// Copies one view's estimated pose. False for an out-of-range index.
    bool copyCamera(int index, double out_rotation[9], double out_centre_m[3]) const;

    /// Writes the reconstructed map to `directory` (map file plus one feature file per view), so
    /// another process can relocalise against a scan it did not build (D058). False when no view
    /// was registered or the directory cannot be written.
    bool saveMap(const std::string& directory) const;

    /// Replaces this session with the map stored in `directory`: views, features, poses and
    /// points. Tracking then works exactly as after a run. False leaves the session untouched.
    bool loadMap(const std::string& directory);

    /// Report of the last run (or of the loaded map).
    const ReconstructionReport& report() const { return result_.report; }

    /// Immutable copy of the map, for tracking while this session runs the next one (D051).
    std::shared_ptr<const Tracker> snapshot() const;

    /// Extracts features from a fresh frame, matches them against `view_hint` in the map (or the
    /// last registered view) and estimates the pose by PnP. False when too few correspondences
    /// survive; `out_inliers` is 0 in that case.
    bool trackPose(
        int width_px,
        int height_px,
        const unsigned char* gray,
        double focal_px,
        double cx_px,
        double cy_px,
        Pose& out_pose,
        int& out_inliers,
        int view_hint = -1) const;

private:
    SfmOptions options_;
    feature::FeatureParams feature_params_;
    std::vector<CameraModel> cameras_;
    std::vector<feature::FeatureSet> features_;
    std::vector<std::vector<Vec2>> pixels_;
    SfmResult result_;
    bool ran_ = false;
};

}  // namespace scand3d::sfm
