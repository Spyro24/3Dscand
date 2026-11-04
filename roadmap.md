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

- [ ] 1.1 Camera2 session with preview and full-resolution still capture
- [ ] 1.2 Lock auto-exposure, auto-white-balance and focus after warm-up
- [ ] 1.3 Frame ring buffer with timestamp, exposure, ISO and focus metadata
- [ ] 1.4 Monochrome preview shader (luminance only)
- [ ] 1.5 Save frames as JPEG with EXIF and a session manifest (relative paths only)
- [ ] 1.6 Lost-camera and permission-denied handling

**Acceptance:** preview runs at 30 fps or more on the target device; 100 frames saved with correct metadata; preview is strictly grayscale.

---

## Phase 2 - IMU orientation
Goal: reliable phone orientation with timestamps synced to frames.

- [ ] 2.1 Read gyroscope, accelerometer, magnetometer and rotation vector sensors
- [ ] 2.2 Implement Madgwick or Mahony filter in `core/sensors/`
- [ ] 2.3 Sync sensor timestamps with camera timestamps (same clock, measured offset)
- [ ] 2.4 Angular-rate monitor: warn when motion will cause blur
- [ ] 2.5 Fallback path for devices without a usable gyroscope
- [ ] 2.6 Use IMU rotation as the prior for keyframe selection

**Acceptance:** orientation error below 2 degrees against a reference rig; frame-to-sensor time offset measured and stored per device; app runs in the fallback path on a test device without a gyroscope.

---

## Phase 3 - Features and matching
Goal: our own feature pipeline, verified on known data.

- [ ] 3.1 Scale-space keypoint detector (SIFT-like or ORB, see agents.md section 7)
- [ ] 3.2 Descriptor computation and storage
- [ ] 3.3 Brute-force and approximate nearest-neighbor matching with ratio test
- [ ] 3.4 RANSAC for essential and fundamental matrix estimation
- [ ] 3.5 Match visualization tool for debugging (desktop-side test tool)

**Acceptance:** on synthetic image pairs with known ground truth, at least 95 percent of inlier matches are correct; on real object pairs, at least 100 verified inliers.

---

## Phase 4 - Incremental SfM
Goal: camera poses and a sparse point cloud from a capture session.

- [ ] 4.1 Two-view initialization with triangulation
- [ ] 4.2 Triangulation and PnP with RANSAC
- [ ] 4.3 Keyframe selection (baseline, overlap, IMU prior)
- [ ] 4.4 Sparse bundle adjustment (Gauss-Newton or Levenberg-Marquardt with Schur complement)
- [ ] 4.5 Map store with track management and outlier removal
- [ ] 4.6 Reconstruction report (registered images, points, reprojection error)

**Acceptance:** synthetic turntable scan reconstructs with mean reprojection error below 1 px and pose error below 1 degree; a real object scan registers at least 90 percent of keyframes.

---

## Phase 5 - Live color overlay
Goal: the reconstruction appears in color over the monochrome preview, aligned to the live view.

- [ ] 5.1 Render sparse points over the grayscale camera background
- [ ] 5.2 Use live IMU and tracking pose for the overlay camera
- [ ] 5.3 Per-point color sampled from keyframes
- [ ] 5.4 Overlay opacity slider and on/off toggle
- [ ] 5.5 Drift check: overlay stays aligned after motion

**Acceptance:** overlay stays within about 10 px of the real object after 10 seconds of handheld motion; overlay toggle is instant (under 100 ms).

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
