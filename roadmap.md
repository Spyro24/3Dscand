# roadmap.md - 3D Scan (Android)

Read `agents.md` first. Work top to bottom. A task is done only when its acceptance test passes.

Legend: `[ ]` open, `[x]` done, `[~]` in progress, `[s]` stub (marked `// STUB` in code)

---

## Phase 0 - Foundation
Goal: an empty app that builds, runs native code and runs tests.

- [x] 0.1 Gradle project with Kotlin, Compose and minimum SDK 29
- [x] 0.2 CMake/NDK setup for arm64-v8a, native library loads at app start
- [x] 0.3 JNI bridge skeleton in `core/bridge/`
- [x] 0.4 Test harness: gtest for C++, JUnit for Kotlin, both run locally (CI wiring deferred to 12.1)
- [x] 0.5 Synthetic scene generator (known mesh, simulated cameras) as a test utility
- [x] 0.6 Benchmark logger (frame times, memory, thermal state) writing to `docs/benchmarks/`
- [x] 0.7 `DECISIONS.md` and `docs/architecture.md` created

**Acceptance:** APK builds with `assembleDebug` (CI runs only at public release, see 12.1), native
library loads on the connected test device, all unit tests pass.

---

## Phase 1 - Capture
Goal: a stable camera session with a monochrome live preview and saved photos.

- [x] 1.1 Camera2 session with preview and full-resolution still capture
- [x] 1.2 Lock auto-exposure, auto-white-balance and focus after warm-up
- [x] 1.3 Frame ring buffer with timestamp, exposure, ISO and focus metadata
- [x] 1.4 Monochrome preview shader (luminance only)
- [x] 1.5 Save frames as JPEG with EXIF and a session manifest (relative paths only)
- [x] 1.6 Lost-camera and permission-denied handling

**Acceptance:** preview runs at 30 fps or more on the target device; 100 frames saved with correct metadata; preview is strictly grayscale; a camera loss shows the CameraLost screen and Retry restores a locked preview that keeps saving frames (observed on the test device, see `docs/benchmarks/phase1-capture-benchmark.md`).

---

## Phase 2 - IMU orientation
Goal: reliable phone orientation with timestamps synced to frames.

- [~] 2.1 Read gyroscope, accelerometer, magnetometer and rotation vector sensors
- [~] 2.2 Implement Madgwick or Mahony filter in `core/sensors/`
- [~] 2.3 Sync sensor timestamps with camera timestamps (same clock, measured offset)
- [~] 2.4 Angular-rate monitor: warn when motion will cause blur
- [~] 2.5 Fallback path for devices without a usable gyroscope
- [~] 2.6 Use IMU rotation as the prior for keyframe selection

**Acceptance:** orientation error below 2 degrees against a reference rig; frame-to-sensor time offset measured and stored per device; app runs in the fallback path on a test device without a gyroscope.

**Status:** implementation, host tests and one device smoke run are complete for all six tasks (36
new JUnit cases, 18 new gtest cases, `assembleDebug` green). On the SM-G930F the run confirmed the
sensor set and rates (accel ~100 Hz, gyro ~199 Hz), schema-2 manifests with per-frame IMU data, a
measured camera/sensor offset of -0.37 to -0.70 ms with ~1 ms MAD, the persisted per-device
profile, and 27.1 fps preview during capture. It also exposed a real defect: the final session of a
run lost its `clock`/`imu` blocks when a late frame rewrote the manifest, now fixed. Two acceptance
items remain **unmeasured**: orientation error against a reference rig (the planned 90 degree table
check was declined, so no accuracy number exists) and the no-gyroscope fallback on hardware (the
SM-G930F has a gyro, D010 rules out a second device). `[~]` therefore means "code and device smoke
test done, reference measurement pending" (D021, D027).
`docs/benchmarks/phase2-imu-verification.md` records the measurements and the exact procedures that
close the phase.

---

## Phase 3 - Features and matching
Goal: our own feature pipeline, verified on known data.

- [~] 3.1 Scale-space keypoint detector (SIFT-like or ORB, see agents.md section 7)
- [~] 3.2 Descriptor computation and storage
- [~] 3.3 Brute-force and approximate nearest-neighbor matching with ratio test
- [~] 3.4 RANSAC for essential and fundamental matrix estimation
- [~] 3.5 Match visualization tool for debugging (desktop-side test tool)

**Acceptance:** on synthetic image pairs with known ground truth, at least 95 percent of inlier matches are correct; on real object pairs, at least 100 verified inliers.

**Status:** implementation and host tests are complete for all five tasks (19 new gtest cases; 48
native tests in total, 37 JUnit cases unchanged, `assembleDebug` green). The synthetic half of the
acceptance line is met and measured: 95.7 percent of the RANSAC inliers (22 of 23) map to the same
known 3D point across two views. The real-object half (at least 100 verified inliers) has **not**
been run: no phone and no real image pair were available (D026). `[~]` therefore means "code done,
acceptance pending", exactly as in Phase 2. The pipeline is not yet reachable from the app; the
bridge wiring belongs to Phase 4, which is also the first consumer of the matches.
`docs/benchmarks/phase3-feature-verification.md` lists the measured numbers, the limits of that
evidence, and what a real-image run still has to show.

---

## Phase 4 - Incremental SfM
Goal: camera poses and a sparse point cloud from a capture session.

- [~] 4.1 Two-view initialization with triangulation
- [~] 4.2 Triangulation and PnP with RANSAC
- [~] 4.3 Keyframe selection (baseline, overlap, IMU prior)
- [~] 4.4 Sparse bundle adjustment (Gauss-Newton or Levenberg-Marquardt with Schur complement)
- [~] 4.5 Map store with track management and outlier removal
- [~] 4.6 Reconstruction report (registered images, points, reprojection error)

**Acceptance:** synthetic turntable scan reconstructs with mean reprojection error below 1 px and pose error below 1 degree; a real object scan registers at least 90 percent of keyframes.

**Status:** implementation and host tests are complete for all six tasks (9 new gtest cases; 57
native tests in total, 37 JUnit cases unchanged, `assembleDebug` green), plus a desktop report
tool. The synthetic half of the acceptance line is met and measured on a 16-view turntable scan: 16
of 16 views registered, mean reprojection error 0.588 px, worst pose error 0.261 degrees. The
real-object half (at least 90 percent of keyframes) has **not** been run, because the app has no
path from captured frames to matches yet (D030). `[~]` therefore means "code done, acceptance
pending", the same convention as Phases 2 and 3. Measured numbers, the view-count and noise
sweeps, and the limits of that evidence are in `docs/benchmarks/phase4-sfm-verification.md`.

---

## Phase 5 - Live color overlay
Goal: the reconstruction appears in color over the monochrome preview, aligned to the live view.

- [~] 5.1 Render sparse points over the grayscale camera background
- [~] 5.2 Use live IMU and tracking pose for the overlay camera
- [~] 5.3 Per-point color sampled from keyframes
- [~] 5.4 Overlay opacity slider and on/off toggle
- [ ] 5.5 Drift check: overlay stays aligned after motion

**Acceptance:** overlay stays within about 10 px of the real object after 10 seconds of handheld motion; overlay toggle is instant (under 100 ms).

**Status:** 5.1 is implemented: `core/overlay/` projects coloured points through a pinhole camera
in displayed-preview pixels and returns NDC vertices (D031), the bridge exposes the scene and the
projection, and the preview renderer draws them as point sprites over the grayscale background in
the same letterboxed rectangle, blending far to near without a depth buffer. Host tests cover the
projection (centre, axis directions, behind-camera culling, colour, ordering, pose, degenerate
cameras) and the C API round trip. The app has no reconstruction output yet (D030), so the render
path is driven by a debug-only canned cloud, requested with `adb shell am broadcast -a
com.scand3d.debug.CAPTURE --es cmd overlay --es value on`. That check ran on the SM-G930F: with the
1280x720 preview running and exposure locked, all 64 points were projected and the ring drew in
colour over the monochrome image, correctly placed inside the letterbox. `[~]` stands only because
real points are not fed in until the Phase 4 app wiring exists (D030).

5.2 is implemented as far as the available pose goes: the overlay camera takes the camera's own
focal length (`LENS_INFO_AVAILABLE_FOCAL_LENGTHS` scaled by `SENSOR_INFO_PHYSICAL_SIZE`, principal
point at the image centre), the display orientation as the body-to-camera extrinsic, and the live
fused quaternion relative to the pose the overlay was anchored at, re-projected at 33 ms intervals
and covered by nine JUnit cases (`OverlayGeometryTest`). Tracking pose does not exist yet, so the
camera is rotation-only: translation is not compensated (D032).

The app-side path from captured frames to a sparse map now exists, which is the wiring D030
recorded as missing: `sfm::Session` runs features, geometric pair gating, a two-view seed and
incremental registration, the bridge and JNI expose it (`reconstructionCreate/AddView/Run/
PointCount/CopyPoints/TrackPose`), and `ReconstructionRunner` decodes the newest session's keyframes
to 480 px gray, scales the overlay intrinsics to that size, reconstructs and loads the points into
the overlay. It is requested with `adb shell am broadcast -a com.scand3d.debug.CAPTURE --es cmd
reconstruct`. What it measures on the synthetic turntable is three of twenty-four views at 0.375 px
mean reprojection: the seed pair is chosen because a neighbour can actually use its points, and
further views chain the pair's own relative pose where the map is too thin for PnP (D037). Growth
stops at the ring's weakest neighbouring pair, which yields only about 16 geometrically consistent
matches, so the speckle fixture cannot feed a full scan (D035). The overlay therefore still draws
the debug cloud by default, and the tracking half of 5.2 stays `[~]` until a real scan registers.

A second fixture shades the box with continuous value noise instead of flat cells (D038). It raises
a neighbour pair from 34 to 296 geometrically consistent matches, and on it the map grows to three
views and 114 points at 0.375 px mean reprojection. Growth then stops for a reason unrelated to
matches: the map from a box face is coplanar, and the six-point PnP DLT is degenerate on coplanar
points, so the next view cannot be placed. Planar PnP is the open piece before the loop can be
measured on a whole scan.

Reconstruction now runs during the capture itself rather than on a finished session: every keyframe
the camera writes is decoded and added to a live session, which re-runs the sparse map on a worker
at most every 1.5 s and redraws the overlay from it (D039). A 22 s capture on the SM-G930F reached
eight views at 0.6-3 s per update, and all of its points projected into the preview. The view cap is
eight because every update re-matches every pair; a full scan needs the pair cache that turns the
loop incremental. Each run is re-expressed in the first keyframe's frame (D043), so the cloud keeps its place
on the scene as the map grows, and the preview follows the camera's own texture transform
instead of the sensor angle (D042).

5.3 is implemented natively: `overlay::samplePointColors` takes the per-channel median of a 3x3
keyframe patch around each point's projection, and `KeyframeColorSampler` decodes the newest saved
JPEG, packs it to RGB and calls it through the bridge (D033). Six gtest cases cover it. With no
reconstruction the sampling runs on points whose geometry is still the debug cloud.

5.4 is implemented end to end: a switch and an opacity slider in the capture overlay drive
`OverlayFeed.setStyle`, which reaches the renderer's blend uniform without re-projecting, so the
toggle costs one frame.

5.5 stays `[ ]`: the acceptance line is alignment to the real object within about 10 px after 10
seconds of handheld motion, and that needs a real cloud plus a tracked pose. What exists is the
instrumentation: while the overlay is on, the view model logs once a second the angle between the
anchored pose and the live one and the projected centre of the cloud, in NDC (D032).


---

## Phase 6 - Dense reconstruction (MVS)
Goal: dense depth and a dense colored point cloud.

- [ ] 6.1 PatchMatch stereo depth estimation per keyframe
- [ ] 6.2 Geometric consistency filter across views
- [ ] 6.3 Depth fusion into a point cloud
- [ ] 6.4 Processing at reduced resolution with a quality setting
- [ ] 6.5 Cancellation and progress reporting

**Acceptance:** Chamfer distance to synthetic ground truth below the threshold in `docs/benchmarks/`; 100 images finish within the performance budget in agents.md.

---

## Phase 7 - Object detection and segmentation
Goal: isolate the target object from the background.

- [ ] 7.1 Select and license-check a segmentation model; add it to `assets/` (under 50 MB)
- [ ] 7.2 TFLite inference with NNAPI or GPU delegate, with CPU fallback
- [ ] 7.3 Object mask per frame, with propagation between frames to save compute
- [ ] 7.4 Use masks to restrict feature extraction and depth estimation to the object
- [ ] 7.5 Remove background points after reconstruction
- [ ] 7.6 User tap to select the object when several are detected

**Acceptance:** mask IoU at least 0.85 on a labeled test set of 50 frames; background points removed from the final cloud; detection runs on the target device without dropping preview below 20 fps.

---

## Phase 8 - Meshing and texturing
Goal: a clean, textured mesh of the single selected object.

- [ ] 8.1 TSDF volume fusion from depth maps (default, see agents.md section 7)
- [ ] 8.2 Marching cubes mesh extraction
- [ ] 8.3 Cleanup: keep largest connected component, remove floaters, fill small holes
- [ ] 8.4 Decimation with a target face count setting
- [ ] 8.5 UV unwrapping
- [ ] 8.6 Texture atlas with view selection and seam reduction

**Acceptance:** synthetic object mesh within the Chamfer threshold; real object mesh has no floating components; texture has no visible seams from the first capture angles.

---

## Phase 9 - Export
Goal: export one object in standard formats.

- [ ] 9.1 OBJ with MTL and PNG texture
- [ ] 9.2 GLB (binary glTF)
- [ ] 9.3 PLY point cloud and mesh
- [ ] 9.4 Single-object export: only the selected object, with its scale in meters
- [ ] 9.5 Share sheet and Storage Access Framework save location
- [ ] 9.6 Export validation (file opens and reloads without errors)

**Acceptance:** exported OBJ and GLB open in Blender and a glTF viewer with correct texture and scale.

---

## Phase 10 - Interface polish
Goal: a clean, easy app.

- [ ] 10.1 Onboarding: three short screens, skippable
- [ ] 10.2 Capture screen: coverage ring, one primary button, guidance messages
- [ ] 10.3 Review screen: orbit 3D viewer with color overlay toggle
- [ ] 10.4 Export screen with format and quality choice
- [ ] 10.5 Session list: rename, delete with confirmation
- [ ] 10.6 Settings: quality preset, units, overlay defaults
- [ ] 10.7 Accessibility pass: TalkBack, contrast, touch targets

**Acceptance:** a first-time user completes a scan and export without reading documentation; accessibility checks pass on both test devices.

---

## Phase 11 - Robustness and performance
Goal: the app survives real use.

- [ ] 11.1 Save and resume sessions after app kill or call interruption
- [ ] 11.2 Low-memory handling: reduce keyframes or resolution instead of crashing
- [ ] 11.3 Thermal handling: lower processing load when throttled
- [ ] 11.4 Crash log collection (local only)
- [ ] 11.5 Profiling pass on the hot paths (features, matching, bundle adjustment, MVS)

**Acceptance:** 20 consecutive scan-and-reconstruct runs without crash on each test device; resume works after a forced kill.

---

## Phase 12 - Release
- [ ] 12.1 Signing, versioning and release build in CI
- [ ] 12.2 Test matrix report (devices, results, benchmarks)
- [ ] 12.3 User guide: capture tips and export instructions
- [ ] 12.4 Licenses screen listing all dependencies and models

**Acceptance:** release build installs and completes an end-to-end scan and export on every device in the test matrix.

---

## Milestones
- **M1 (Phases 0-2):** monochrome preview and IMU orientation work.
- **M2 (Phases 3-5):** sparse reconstruction with a live color overlay.
- **M3 (Phases 6-8):** single-object dense mesh with texture.
- **M4 (Phases 9-12):** export, polished interface, release.
