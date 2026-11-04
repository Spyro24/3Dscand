#pragma once

#include <cstdint>
#include <string>

namespace scand3d::bridge {

/// Version of the native core. Single source of truth for the value the UI displays and the
/// native smoke test checks.
const char* coreVersion();

/// Opaque handle to one sensor-fusion session. 0 is never a valid handle.
using ImuHandle = std::uint64_t;

/// Number of doubles one sample occupies:
/// [timestamp_ns, gx, gy, gz, ax, ay, az, mx, my, mz, flags]
/// with flags bit0 = gyroscope, bit1 = accelerometer, bit2 = magnetometer present.
constexpr int kImuSampleValueCount = 11;

/// Starts a fusion session for the given sensor rate. The rate only seeds the first time step;
/// afterwards the sample timestamps decide dt, so a jittery sensor does not skew integration.
ImuHandle imuCreate(double sample_rate_hz);

/// Frees a session. Destroying an unknown or already-destroyed handle is a no-op.
void imuDestroy(ImuHandle handle);

/// Returns the session to its initial state, e.g. when the camera restarts.
void imuReset(ImuHandle handle);

/// Feeds one sample. Returns false when the handle is unknown or the values are unusable, in
/// which case the previous orientation is kept.
bool imuUpdate(ImuHandle handle, const double* values, int value_count);

/// Copies the orientation as {w, x, y, z} into `out_quaternion`. False until a first usable
/// orientation exists.
bool imuOrientation(ImuHandle handle, double out_quaternion[4]);

/// Rotation angle between two quaternions in radians, or NaN for non-finite input.
double angularDisplacementRad(const double a[4], const double b[4]);

/// True when the rotation between the two orientations falls inside the keyframe window.
bool keyframeBaselineAccept(
    const double reference[4],
    const double candidate[4],
    double min_baseline_rad,
    double max_baseline_rad);

/// Opaque handle to one reconstruction session: captured gray views in, sparse map out.
using ReconstructionHandle = std::uint64_t;

/// Opaque handle to one overlay scene. 0 is never a valid handle.
using OverlayHandle = std::uint64_t;

/// Opens an empty reconstruction session. The caller owns the handle.
ReconstructionHandle reconstructionCreate();

/// Frees a session. Destroying an unknown or already-destroyed handle is a no-op.
void reconstructionDestroy(ReconstructionHandle handle);

/// Adds a view from an 8-bit row-major gray image. Returns its index, or -1 when the image is
/// unusable or yields no features.
int reconstructionAddView(
    ReconstructionHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px);

/// Extracts features, matches every pair and reconstructs. Returns the number of registered views,
/// or -1 for an unknown handle.
int reconstructionRun(ReconstructionHandle handle);

int reconstructionPointCount(ReconstructionHandle handle);

/// Writes valid points as xyz into `positions_m`; returns how many were written.
int reconstructionCopyPoints(ReconstructionHandle handle, double* positions_m, int max_points);

/// For each valid point, in `reconstructionCopyPoints` order, the view index of the keyframe that
/// first observed it, so the caller can draw one cloud per keyframe (D059). Returns how many ids
/// were written; -1 for an unknown handle.
int reconstructionCopyKeyframeIds(ReconstructionHandle handle, int* out_ids, int max_ids);

/// Estimated pose of one reconstruction view. False for an out-of-range index.
bool reconstructionCopyCamera(
    ReconstructionHandle handle,
    int index,
    double out_rotation[9],
    double out_centre_m[3]);

/// Tracks a gray frame against the map by PnP. Returns the inlier count, 0 when the frame did not
/// register; the pose is written only on success.
int reconstructionTrackPose(
    ReconstructionHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px,
    double out_rotation[9],
    double out_centre_m[3]);

/// Writes the session's map, features included, into `directory` so a later process can
/// relocalise against a scan it did not build (D058). False when no view is registered or the
/// directory cannot be written.
bool reconstructionSaveMap(ReconstructionHandle handle, const std::string& directory);

/// Replaces a session with the map stored in `directory`. False leaves the session untouched.
bool reconstructionLoadMap(ReconstructionHandle handle, const std::string& directory);

/// Opaque handle to one immutable map snapshot, used to track live frames (D051).
using TrackerHandle = std::uint64_t;

/// Copies a session's current map into a tracker. Tracking then runs without the reconstruction
/// lock, so a preview frame does not wait the seconds a map run takes. Returns 0 for an unknown
/// handle or a session that holds no views.
TrackerHandle trackerCreate(ReconstructionHandle handle);

/// Tracks a gray frame against the snapshot by PnP. Returns the inlier count, 0 when the frame did
/// not register; the pose is written only on success.
int trackerTrack(
    TrackerHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px,
    double out_rotation[9],
    double out_centre_m[3]);

/// Frees a snapshot. Destroying an unknown or already-destroyed handle is a no-op.
void trackerDestroy(TrackerHandle handle);

/// Opaque handle to one dense session: posed gray views in, fused point cloud out (roadmap 6.1-6.5).
using DenseHandle = std::uint64_t;

/// Opens an empty dense session. The caller owns the handle.
DenseHandle denseCreate();

/// Frees a session. Destroying an unknown or already-destroyed handle is a no-op.
void denseDestroy(DenseHandle handle);

/// Adds one view at the resolution of `gray`: intrinsics for those pixels and the camera-to-world
/// pose the sparse stage estimated (`rotation` row-major 3x3, `centre_m`). Returns its index, or -1
/// when the input is unusable. Views from a reconstruction that did not register are not added.
int denseAddView(
    DenseHandle handle,
    const unsigned char* gray,
    int width_px,
    int height_px,
    double focal_px,
    double cx_px,
    double cy_px,
    const double rotation[9],
    const double centre_m[3]);

/// Sparse map points, in the same frame as the view poses, that narrow the depth search to the
/// range the reconstruction occupies. Replaces any earlier hints; may be empty.
void denseSetHints(DenseHandle handle, const double* positions_m, int count);

/// Runs the dense stage at `working_width_px` (roadmap 6.4). Views whose hints leave the search
/// empty fall back to [min_depth_m, max_depth_m]. Returns the fused point count, 0 when the run was
/// cancelled or produced nothing, -1 for an unknown handle. Blocking.
int denseRun(
    DenseHandle handle,
    int working_width_px,
    double min_depth_m,
    double max_depth_m);

int densePointCount(DenseHandle handle);

/// Writes points as xyz into `positions_m`; returns how many were written.
int denseCopyPoints(DenseHandle handle, double* positions_m, int max_points);

/// Progress of the current or last run, 0..1 (roadmap 6.5). Readable while `denseRun` blocks.
double denseProgress(DenseHandle handle);

/// Asks the running dense stage to stop; the run then returns 0 and publishes no cloud.
void denseCancel(DenseHandle handle);

/// Floats per vertex written by overlayProject: {x_ndc, y_ndc, r, g, b} (roadmap 5.1).
constexpr int kOverlayVertexValueCount = 5;

/// Camera and image the overlay is projected into. `rotation` is camera-to-world, row-major 3x3;
/// the pinhole model is in displayed preview pixels, and the image size is the one those
/// intrinsics describe (D031).
struct OverlayCamera {
    double rotation[9];
    double centre_m[3];
    double focal_px;
    double cx_px;
    double cy_px;
    int image_width_px;
    int image_height_px;
};

/// Creates an empty scene. The caller owns the handle and must destroy it.
OverlayHandle overlayCreate();

/// Frees a scene. Destroying an unknown or already-destroyed handle is a no-op.
void overlayDestroy(OverlayHandle handle);

/// Replaces every point: `positions_m` is xyz per point, `rgb` is 0..1 per point. False on
/// non-finite or negative input, which leaves the scene untouched.
bool overlaySetPoints(
    OverlayHandle handle,
    const double* positions_m,
    const float* rgb,
    int count);

/// Number of points in the scene, or 0 for an unknown handle.
int overlayPointCount(OverlayHandle handle);

/// Writes up to `max_vertices` visible vertices, far to near. Returns the number written.
int overlayProject(
    OverlayHandle handle,
    const OverlayCamera& camera,
    float* out_vertices,
    int max_vertices);

/// Replaces each point's colour with a median sample of the keyframe pixels it projects onto
/// (roadmap 5.3). `rgb` is packed three bytes per pixel at `image_width_px` x
/// `image_height_px` in the camera's own pixel space. Returns how many points were sampled.
int overlaySampleColors(
    OverlayHandle handle,
    const OverlayCamera& camera,
    const unsigned char* rgb,
    int image_width_px,
    int image_height_px);

}  // namespace scand3d::bridge
