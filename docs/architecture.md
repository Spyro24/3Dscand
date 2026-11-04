# Architecture

How the app is put together, which module owns what, and the conventions every module obeys.
`DECISIONS.md` holds the reasoning behind individual choices; this file describes the shape.

## Layers

```
app/                      Compose UI, navigation, ViewModels, permission and lifecycle handling
app/.../capture/          Camera2 session, exposure/focus locks, frame ring buffer, session store
app/.../render/           OpenGL ES monochrome preview renderer
app/.../benchmark/        Frame-time, memory and thermal recorder
core/bridge/              The single JNI bridge between Kotlin and C++
core/<module>/            C++ reconstruction modules, added phase by phase
tests/synthetic/          Known mesh + simulated cameras, ground truth for tests
tests/native/             gtest suites run on the host
app/src/test/             JUnit suites run on the host JVM
docs/                     Architecture, decisions, benchmark results
```

Kotlin owns everything that is an Android platform API (Camera2, SensorManager, PowerManager,
EGL). C++ owns algorithms: features, geometry, SfM, MVS, meshing, texturing, export. The boundary
between them is `core/bridge/`, and nothing else. `DECISIONS.md` D013 records why capture and
sensors stay on the Kotlin side.

## Module rules

- Every module exposes an interface: a C++ header in its own directory, or a Kotlin
  `interface`/single entry class. No module reaches into another module's internals.
- Kotlin talks to native code only through `com.scand3d.core.bridge.NativeCore`, which mirrors
  `core/bridge/jni_bridge.cpp` one `external fun` at a time.
- Test-only utilities live under `tests/` and are never compiled into the APK.
- A stage that is not implemented is marked `// STUB` and listed in `roadmap.md`; nothing may
  present placeholder geometry as a real reconstruction.

## Threads

| Thread | Owns | Rule |
|---|---|---|
| Main | Compose UI, state collection | No camera, file or GL work |
| Camera callback (`HandlerThread`) | Camera2 device, session, capture callbacks | Never touches UI state directly; reports through the ViewModel |
| JPEG reader (`HandlerThread`) | `ImageReader` JPEG planes, file writes | Hands metadata to the session store |
| GL thread (`GLSurfaceView`) | Preview texture, shader, viewport | Only GL calls |
| Benchmark writer | JSONL appends | Single writer, daemon thread |
| Sensor (`HandlerThread`) | `SensorManager` listeners, fusion feed | Only sensor callbacks; never blocks the camera or the UI |

State crosses threads through `StateFlow` in the ViewModel; Compose collects with
`collectAsState` and never blocks on a camera operation.

## Data conventions

- Right-handed coordinates, meters, radians. Units are part of the name: `focal_px`,
  `baseline_m`, `angle_rad`, `time_ns`, `exposure_ns`, `focus_diopter`.
- Poses are camera-to-world, 4x4, `float64`. Camera axes follow OpenCV (+X right, +Y down,
  +Z forward); see `DECISIONS.md` D012.
- A captured frame carries: `timestamp_ns` on the camera/sensor clock, exposure, ISO, focus
  distance and a **relative** path. Absolute paths are never stored (agents.md section 5).
- Session layout on device: `files/sessions/<session-id>/session.json` plus
  `files/sessions/<session-id>/frames/frame_<index>.jpg`.

## Capture pipeline (Phase 1)

```
Camera2 device
  ├─ preview: SurfaceTexture (OES) → GL shader → luminance only on screen
  └─ still:   ImageReader(JPEG) → frames/frame_XXXX.jpg
CaptureResult ──► frame ring buffer (timestamp, exposure, ISO, focus)
             └─► session manifest (JSON, relative paths)
```

After warm-up the session locks auto-exposure, auto-white-balance and auto-focus to the last
measured values, so consecutive frames share one photometric setting. The lock values are
written into the manifest, because reconstruction depends on them staying constant.

## Sensor pipeline (Phase 2)

```
SensorManager (gyro 5 ms, accel 10 ms, mag + rotation vector 20 ms)
  └─ imu-sensors thread → ImuSample(timestamp_ns, gyro, accel, mag)
       ├─ AngularRateMonitor → blur warning in CaptureUiState (no gyro → Unavailable)
       └─ ImuFusion (JNI) → Mahony filter → ImuRingBuffer
                              └─ ImuFrameAnnotator: orientation, clock residual, keyframe flag
                                   └─ FrameRecord → session.json (schema 2)
```

Samples are driven by the gyroscope and by the accelerometer when there is no gyro. The fusion
session lives in C++ (`core/sensors/`) behind `core/bridge/`; the rotation vector is read for
diagnostics but not fused, because the OS value cannot be tied to our own filter state.

Without a gyroscope the same session falls back to gravity plus magnetic north: pitch and roll stay
observable, yaw exists only with a magnetometer, and the capture screen says so. That is the
reduced-accuracy path agents.md section 4 requires.

Clock handling: both streams are read on the boot-relative clock, and the residual between a frame
timestamp and its nearest sensor sample is measured per session (median and MAD). The median is
stored per device in `files/devices/<model>.profile` and shifts the next session's pairing lookup;
the manifest records what this session measured. It is a measurement with an uncertainty, not a
calibration (D020).

Keyframes: the first oriented frame anchors the scan, a frame rotated inside the 0.10-0.70 rad
window is marked `keyframe_candidate`, and an overshoot re-anchors so one fast turn does not end
the selection. Phase 4 owns the final decision; the IMU only knows rotation, never translation.

## Feature pipeline (Phase 3)

```
GrayImage (level 0)
  └─ buildPyramid: blur sigma 1.2, then half-resolution levels (D022)
       └─ detectFastCorners (FAST-9 per level, non-max suppression)
            └─ extractFeatures: map back to level-0 pixels, dedup by 2 px, keep the strongest
                 └─ describeBrief (BRIEF-256, ±15 px patch, unsteered)
                      └─ FeatureSet { keypoints, descriptors }  -> saveFeatures (S3DFEAT1)

FeatureSet A + FeatureSet B
  └─ matchBruteForce / matchApproximate (Hamming, ratio 0.8)
       └─ PointCorrespondence[] (pixels)
            └─ estimateFundamentalRansac / estimateEssentialRansac (8-point, Sampson, refit)
                 └─ { matrix, inlier indices, median Sampson px }
```

`core/feature/` owns detection, description, matching and the feature store; `core/geometry/` owns
the linear algebra and the two-view estimators. Neither depends on Android. `tests/synthetic/`
renders the textured box used as ground truth, `tests/native/geometry_test.cpp` builds its own
non-coplanar lattice because a coplanar scene makes the eight-point problem ambiguous, and
`tests/tools/` holds the PGM/PPM helpers plus the `match_visualizer` desktop binary, which is built
only in `nativeTest` and never packaged into the APK.

Storage: one feature record is five `float32` values plus 32 descriptor bytes, behind the versioned
`S3DFEAT1` header (D023). Keypoints are stored in level-0 pixel coordinates with their scale, so a
feature file does not depend on the pyramid that produced it. The pipeline is native-only for now;
the bridge still exposes only the Phase 2 handle API, and the path from captured frames to matches
is not wired into the app yet.

## Sparse reconstruction (Phase 4)

```
pixels + camera model per view, matches between neighbouring views
  └─ reconstruct (core/sfm/pipeline.cpp)
       ├─ TrackBuilder: union-find over (view, feature) -> one track per 3D point
       ├─ seed: initializeTwoView on a matched pair (essential matrix, four pose candidates,
       │        cheirality with a reprojection tie-break), then a PnP polish on its inliers
       ├─ grow: PnP/RANSAC per unregistered view against the map, keyframe gate
       │        (rotation window, baseline, overlap), re-triangulate, BA, outlier removal
       └─ finalize: bundle adjustment, outlier removal, duplicate merge, report
            └─ { camera poses, sparse points, ReconstructionReport }
```

`core/sfm/` owns the reconstruction and depends only on `core/geometry/` and `core/sfm/types.hpp`.
It takes pixels and camera models rather than a `FeatureSet`, so the tests need no rendering and
nothing in the module touches Android. Two decisions shape it: the result is defined only up to a
similarity (the first camera is fixed and the global scale is free), and a seed pair is trusted only
after a third view registers against it, because a mirrored two-view decomposition fits its own pair
exactly (D029). Bundle adjustment is Levenberg-Marquardt with the Schur complement on the point
blocks, so only the 6N camera system is ever factorised (D028).

`tests/synthetic/sfm_scan.cpp` builds the turntable scan used by both the tests and
`tests/tools/sfm_report.cpp`, the desktop binary that prints the acceptance numbers. Like the match
visualizer it is built only in `nativeTest` and never packaged into the APK.

## Live overlay (Phase 5)

```
reconstruction points + sampled colour (Phase 4 output)
  └─ NativeCore.overlaySetPoints) -> overlay::OverlayScene        (core/overlay/)
       └─ overlayProject(camera: intrinsics in displayed preview pixels, camera-to-world pose)
            └─ { x_ndc, y_ndc, r, g, b } sorted far to near
                 └─ OverlayFeed -> MonochromePreviewRenderer point-sprite pass
                      └─ drawn into the same letterboxed rectangle as the camera image
```

`core/overlay/` owns the projection and depends only on `core/geometry/`; the camera contract
(pixels of the displayed preview image, camera-to-world pose, NDC out) is D031. The renderer
keeps its background pass untouched and adds one blended `GL_POINTS` pass afterwards, so the
overlay cannot change how the monochrome preview looks. Vertices are handed over through
`OverlayFeed`, which keeps the last frame and replays it into a renderer that was rebuilt with the
view.

The overlay camera (5.2) is built from what the device can actually report: focal length from
`LENS_INFO_AVAILABLE_FOCAL_LENGTHS` scaled by `SENSOR_INFO_PHYSICAL_SIZE`, the principal point at
the image centre, and the orientation from the fused quaternion relative to the pose the overlay
was anchored at, composed with a display-rotation extrinsic in `OverlayGeometry`. Projection runs
every 33 ms on the sensor thread and is published through `OverlayFeed`. There is no tracking pose
yet, so the camera is rotation-only and its centre is the origin (D032).

Colours (5.3) come from the app, because the core only ever sees gray images: `KeyframeColorSampler`
decodes the newest saved keyframe, and `overlay::samplePointColors` takes the per-channel median of
a 3x3 patch around each point's projection (D033). The opacity slider and on/off switch (5.4) go
through `OverlayFeed.setStyle` straight to the renderer's blend uniform, so toggling never
re-projects.

The drift check (5.5) is instrumented but not measured: while the overlay is on, the view model
logs once a second the anchored-to-live pose angle and the cloud's projected centre. Because the
app has no capture-to-reconstruction path yet (D030), the scene is still a debug-only canned cloud,
marked `// STUB`, so no frame of reference exists to compare against a real object.

## Dense reconstruction (Phase 6)

```
keyframes as gray images plus poses (Phase 4 output), optional sparse points
  └─ reconstructDense (core/mvs/pipeline.cpp)
       ├─ resample every view to the preset's working width (Draft 160 / Normal 320 / High 480)
       ├─ per view, against its nearest neighbour: estimateDepth (core/mvs/patchmatch.cpp)
       │    ├─ plane per pixel: depth + normal, 5x5 ZNCC of the warped patch
       │    ├─ log-spaced plane sweep for the first plane, then propagation + random search
       │    └─ a plane whose best cost is above max_cost keeps no depth (ZNCC below 0.5)
       ├─ filterByConsistency: forward-backward depth and pixel check (core/mvs/consistency.cpp)
       └─ fuseDepths: 4 mm voxels, mean position and gray, sorted output (core/mvs/fusion.cpp)
            └─ { points, per-view depth maps, stats }
```

`core/mvs/` depends only on `core/feature/`, `core/geometry/` and `core/sfm/types.hpp`, so it takes
gray images and camera models and nothing Android. Depth is a plane per pixel rather than a single
depth, which is what lets a slanted surface match across a baseline (D045); the plane sweep is the
initial guess and propagation plus random search does the work, with a fixed seed so one capture
gives one depth map twice. Sparse map points, when the caller has them, narrow the search to the
range the reconstruction occupies instead of the full 0.1-10 m range.

The views are independent - each owns its depth map and its own random sequence - so the per-view
loop runs on a small worker pool sized from `hardware_concurrency` and capped at eight, the thread
count agents.md lists for MVS (D047). Fusion then runs once over the surviving maps, so a cloud is
only published when the run was not cancelled.

Nothing in the app calls this yet: the bridge has no dense entry point and the overlay still draws
the sparse map. The desktop binary `tests/tools/mvs_report.cpp` is what prints the acceptance
numbers, and it is built only in `nativeTest`.

## Capture to reconstruction (app wiring)

```
session keyframe JPEGs
  └─ ReconstructionRunner: decode to 480 px gray, scale the overlay intrinsics to that size
       └─ NativeCore.reconstruction* -> bridge -> sfm::Session
            ├─ features, essential-gated neighbour matches, richest seed pair
            ├─ incremental registration: PnP against the map, keyframe gate, new points triangulated
            └─ copyPoints / copyCamera / TrackPose -> overlay scene + KeyframeColorSampler
```

`sfm::Session` composes `core/feature`, `core/geometry` and `core/sfm` rather than duplicating them:
neighbouring views are matched, each pair is gated by an essential matrix so a false match cannot
taint a point, the richest seed pair starts the map, and further views register one at a time by
PnP against it. Building the map from global tracks welded inconsistent matches together, which is
why the session grows view by view (D035). The bridge and JNI mirror the same contract, and the
runner keeps the core gray by decoding and colour-sampling in the app (D033, D036).

The measured part of this path is three of twenty-four views at 0.375 px mean reprojection: the seed
pair is chosen because a neighbour can use its points, and further views chain the pair's own
relative pose where the map is too thin for PnP (D037). Growth stops at the ring's weakest
neighbouring pair (~16 geometric matches), so the overlay still draws the debug cloud and this stays
`[~]` (D035, D037). On a second fixture with a continuous value-noise texture, a neighbour pair keeps
296 of 310 matches and the map reaches three views and 114 points before PnP refuses: the map is a
box face, and the six-point DLT cannot solve a coplanar point set (D038).

The app feeds the same session while the capture runs: each keyframe the camera writes is decoded to
480 px gray and added to one live session, a worker re-runs the map at most every 1.5 s, and the
resulting points replace the overlay scene (D039). The cloud arrives in the reconstruction's first
camera frame, so it is rotated by the anchor camera pose and rescaled before projection; without
that rotation every point has negative depth and nothing is drawn.

## Current phase status

- Phase 0 complete: project, native bridge, local test harnesses, synthetic scenes, benchmarks,
  this document.
- Phase 1 complete: Camera2 capture with locked settings, monochrome preview, session storage,
  and camera-loss/permission-denied handling, all exercised on the test device. A lost camera is
  torn down on the camera thread, the UI shows the `CameraLost` screen, and Retry builds a fresh
  controller and re-locks; a preview surface arriving while the UI is in a terminal state is
  ignored (D017).
- Phase 2 implemented and smoke-tested on the SM-G930F, reference accuracy outstanding: sensors,
  Mahony fusion, the tilt fallback, the blur monitor, per-frame clock pairing and the keyframe
  prior are in place, covered by host tests, and now confirmed on device for sensor rates, schema-2
  manifests, the measured clock offset, the persisted device profile and 27.1 fps preview. The
  2 degree orientation bound against a reference and the no-gyroscope fallback are still unmeasured
  (D021, D027, `docs/benchmarks/phase2-imu-verification.md`).
- Phase 3 implemented, real-image acceptance outstanding: FAST-9 on a pyramid, unsteered
  BRIEF-256 descriptors, binary feature storage, exhaustive and approximate matching, RANSAC for
  the fundamental and essential matrix, and the match visualizer are in place and covered by host
  tests. The synthetic acceptance (95.7% verified inlier precision) is measured; the real-object
  requirement of 100 verified inliers has not been run (D026,
  `docs/benchmarks/phase3-feature-verification.md`).
- Phase 4 implemented, real-scan acceptance outstanding: two-view initialisation, triangulation,
  PnP with RANSAC, the keyframe policy, sparse bundle adjustment with the Schur complement, the map
  store with track management and outlier removal, and the reconstruction report are in place and
  covered by host tests. The synthetic acceptance is measured (16 of 16 views, 0.588 px mean
  reprojection, 0.261 degree worst pose error); the real-object requirement of 90 percent
  registered keyframes has not been run, and the app-side path from captured frames to matches is
  still missing (D030, `docs/benchmarks/phase4-sfm-verification.md`).
- Phase 5 in progress: the overlay renders, follows the fused IMU pose with real camera
  intrinsics, samples point colours from keyframes and has its toggle and opacity control, all
  covered by host tests and (for 5.1) confirmed drawing on the SM-G930F. The app can now reconstruct
a saved session through `sfm::Session` and load it into the overlay, but only the seed pair
  reconstructs reliably on synthetic data, so the overlay's default content is still the debug cloud. 5.5 is open and the whole
  phase stays unaccepted until a real cloud exists (D031-D036).
- Phase 6 implemented, synthetic acceptance met: PatchMatch stereo depth per view, the
  forward-backward consistency filter, voxel fusion, the Draft/Normal/High working widths and
  cancellation with progress are in place and covered by nine host tests. On the synthetic turntable
  the fused cloud is 4.7 mm from the ground truth at Normal quality and 100 images at Draft take
  76 s on the host, inside the "few minutes" budget. Not measured on the SM-G930F and not wired into
  the app: there is no dense entry point in the bridge, so the overlay still shows the sparse map
  (D045-D048, `docs/benchmarks/phase6-mvs-verification.md`).
- Phases 7 and later are unimplemented; `roadmap.md` is the source of truth.

The preview is drawn in whatever orientation the camera's texture transform delivers: on the
SM-G930F that transform transposes the buffer, so the preview takes no extra rotation while the
overlay points take one quarter turn (D042). Rotating on the sensor angle as well was what made the
preview appear sideways.
