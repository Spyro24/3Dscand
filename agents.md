# agents.md - 3Dscand - a 3d scan app (Android)

## 1. Purpose
An Android app that reconstructs real objects in 3D from photos taken with the phone camera, using photogrammetry. The app is built from scratch.

Core features:
- Monochrome live camera preview, with the color reconstruction overlaid on top of it.
- IMU sensors (gyroscope, accelerometer, magnetometer, rotation vector) determine the phone orientation to improve pose accuracy, when available.
- Object detection and segmentation isolate the target object for better accuracy and for single-object export.
- Export of one object as a textured mesh.

## 2. Hard constraints
- **From scratch.** We write our own feature extraction, matching, structure-from-motion (SfM), multi-view stereo (MVS), meshing and texturing.
- **Not allowed as the reconstruction core:** COLMAP, Meshroom, OpenMVG, OpenMVS, Open3D reconstruction, ARCore Depth or Scene Reconstruction APIs, or any other complete photogrammetry pipeline.
- **Allowed:** Android platform APIs (Camera2, SensorManager, NDK, Vulkan or OpenGL ES, MediaStore, Storage Access Framework), general math libraries (Eigen, GLM), TensorFlow Lite for model inference, and image codecs (libjpeg, libpng).
- Any new third-party dependency must be recorded in `DECISIONS.md` with its version, license and reason.
- Minimum Android 10 (API 29), target arm64-v8a. Test on at least two physical phones before a release.
- Kotlin for the UI and app shell. C++17 (NDK, CMake) for the reconstruction core, image processing and rendering.
- Fully offline. No network calls during capture or reconstruction. Models ship inside `assets/`.
- Photos and models stay on the device unless the user explicitly exports them.
- every change has to be commited after testing with a semantig commit with the curent users git config.

## 3. Architecture

```
app/                 Compose UI, navigation, ViewModels (Kotlin)
core/capture/        Camera2 session, exposure/focus/white-balance locking, frame ring buffer
core/sensors/        IMU fusion (rotation vector, or Madgwick/Mahony from gyro+accel+mag), timestamp sync
core/detect/         On-device detection and segmentation (TFLite + NNAPI/GPU delegate), outputs masks
core/feature/        Own feature detector and descriptor (SIFT-like or ORB), matching, ratio test
core/geometry/       Essential/fundamental matrix, RANSAC, triangulation, PnP, pose utilities
core/sfm/            Incremental SfM, initialization, bundle adjustment (sparse Gauss-Newton / LM), map store
core/mvs/            PatchMatch depth estimation, geometric consistency, depth fusion
core/mesh/           Poisson or TSDF meshing, cleanup, decimation, single-object extraction
core/texture/        UV unwrapping, view selection, texture atlas, seam reduction
core/export/         OBJ + MTL + PNG, GLB, PLY
render/              OpenGL ES 3.x or Vulkan: grayscale camera background, color overlay, orbit viewer
tests/               Unit tests (gtest for C++, JUnit for Kotlin), synthetic scenes, device test scripts
docs/                Architecture notes, DECISIONS.md, benchmark results
```

### Module rules
- Every module exposes an interface header (C++) or interface (Kotlin). No module reaches into another module's internals.
- The Kotlin layer calls the native core only through one JNI bridge in `core/bridge/`.
- Threads: camera thread, sensor thread, tracking thread, MVS worker pool, render thread. Nothing heavy runs on the UI thread.

### Data conventions
- Coordinates: right-handed, units in meters, radians for angles.
- Poses: camera-to-world, 4x4, stored as float64.
- Units appear in names: `focal_px`, `baseline_m`, `angle_rad`, `time_ns`.
- Every captured frame carries: timestamp (ns, same clock for camera and sensors), IMU orientation snapshot, exposure time, ISO, focus distance, and the image path (relative only, never absolute).

## 4. Preview and UI rules
- **Live preview is monochrome.** A fragment shader converts the camera feed to luminance only. No color is shown in the camera background.
- **Reconstruction is in color.** The point cloud or mesh is rendered over the monochrome preview with its real colors, depth-tested against the live pose, with adjustable opacity and an on/off toggle.
- **Pose from the IMU.** The current phone orientation drives the overlay alignment and is used as a prior for tracking. If no gyroscope exists, fall back to accelerometer and magnetometer, and show a reduced-accuracy hint.
- **Clean interface:** one primary action per screen. States: Idle, Capturing, Reconstructing, Review, Export.
- **Guidance:** a coverage ring around the object shows which angles are captured. Warn about motion blur (gyro rate too high), too little overlap, and poor lighting.
- **Accessibility:** touch targets at least 48dp, sufficient contrast, TalkBack labels on all controls.
- **Dark theme** is the default.

## 5. Working rules for agents
1. Read `roadmap.md` first. Work on the lowest-numbered unchecked task in the current phase.
2. One task per commit. Commit message format: `[phase.task] short summary`, for example `[2.3] add gyro timestamp sync`.
3. A task is done only when its acceptance test passes. Record the result in `docs/benchmarks/` or the test log.
4. Update the checkboxes in `roadmap.md` and add any decision to `DECISIONS.md` in the same commit.
5. Ask before adding a dependency, changing a public interface, or changing the minimum Android version.
6. Never present fake or placeholder reconstruction output as real. If a stage is stubbed, mark it `// STUB` and list it in the roadmap.
7. Keep functions short. Comments explain why, not what.
8. Do not use the deprecated `android.hardware.Camera` API.
9. Do not store absolute file paths in project data or saved sessions.

## 6. Quality gates
- **Synthetic tests:** render a known mesh from simulated cameras, run the pipeline, and measure the Chamfer distance to ground truth and the reprojection error. These run in CI.
- **Real-device tests:** a fixed set of test objects on a fixed turntable, run on at least two phones.
- **Performance budgets** (target device, to be refined in Phase 0):
  - Preview: at least 30 fps.
  - Per-frame tracking: 33 ms or less.
  - Dense reconstruction of 100 images at reduced resolution: a few minutes.
- **Memory:** peak below 2 GB for 150 keyframes, using images downscaled to 1600 px on the long edge for MVS.
- **Thermal:** the app detects throttling and lowers the processing load instead of crashing.

## 7. Open decisions (record in DECISIONS.md)
- Feature detector: SIFT-like (more accurate, slower) or ORB (faster, less accurate). Default: start with ORB, evaluate SIFT-like in Phase 3.
- Meshing: Poisson (needs oriented normals, watertight) or TSDF (robust, needs depth maps). Default: TSDF.
- Object detection model: which pretrained segmentation model, and its license. Must be redistributable.
- Minimum device tier and fallback when the phone has no usable gyroscope.

## 8. Don'ts
- Don't block the UI thread with native work.
- Don't run reconstruction without a cancel path.
- Don't delete user sessions without confirmation.
- Don't commit model weights larger than 50 MB to git. Use Git LFS or download them at build time from a fixed, checksummed source.
