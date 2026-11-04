# DECISIONS.md

Every third-party dependency and every choice that constrains the architecture is recorded
here with its version, license and reason (agents.md sections 2 and 7).

## Toolchain

### D001 - Gradle 8.9, AGP 8.5.2, Kotlin 2.0.20, Compose BOM 2024.09.00
- **License:** Gradle (Apache-2.0), Android Gradle Plugin (Apache-2.0), Kotlin (Apache-2.0),
  Compose (Apache-2.0).
- **Reason:** AGP 8.5.2 requires Gradle 8.7 or newer, and these versions were already present in
  the local Gradle cache, so the first build needs no toolchain download. The Kotlin Compose
  compiler ships with Kotlin 2.0, so the separate `composeOptions` block is not needed.
- **Consequence:** Upgrading AGP is required before raising `compileSdk` past 34.

### D002 - compileSdk and targetSdk 34, minSdk 29
- **Reason:** `minSdk` 29 is a hard constraint (agents.md section 2). AGP 8.5.2 officially supports
  `compileSdk` 34; Android 10 (API 29) is the oldest supported runtime.
- **Consequence:** Raising `targetSdk` needs a matching AGP bump (D001).

### D003 - Project-local Android SDK in `.androidsdk/`
- **Reason:** The machine's `/usr/lib/android-sdk` is root-owned and contains only `build-tools`
  and `platform-tools`, with no `platforms`, `cmdline-tools` or NDK. Installing into the project
  needs no elevated privileges and keeps the toolchain reproducible.
- **Contents:** `cmdline-tools` 11076708, `platform-tools`, `platforms;android-34`,
  `build-tools;34.0.0`, added via `sdkmanager --sdk_root`.
- **Consequence:** The directory is gitignored. `local.properties` points at it and is also
  gitignored; a fresh clone must run the SDK install and write its own `local.properties`.

### D004 - Build with JDK 21, not the system default JDK 25
- **Reason:** Gradle 8.9 cannot run on JDK 25; JDK 21 is installed at
  `/usr/lib/jvm/java-21-openjdk-amd64` and is supported by both Gradle 8.9 and AGP 8.5.2.
- **Consequence:** `JAVA_HOME` must be set for `./gradlew`. Java and Kotlin target 17 for the
  app bytecode. The path is deliberately not committed to `gradle.properties` so the build stays
  portable to other machines.

### D005 - Kotlin and Compose UI only, single `:app` module
- **Reason:** Roadmap 0.1 asks for Kotlin, Compose and min SDK 29. Native modules arrive with
  tasks 0.2 and 0.3 under the module layout in agents.md section 3.
- **Consequence:** `abiFilters` is already restricted to `arm64-v8a` so packaging stays correct
  when the native library is added.

### D006 - Compose dark theme, platform window theme only
- **Reason:** agents.md section 4 requires a dark default and one primary action per screen. No
  Material Components (View) dependency is needed for a pure Compose app, so the manifest theme
  is the platform `android:Theme.Material.NoActionBar` with a black window background.
- **Consequence:** Splash and window background are black; all UI styling is Material3 in Compose.

### D007 - Application ID `com.scand3d`
- **Reason:** Stable, unique package for the app and its future JNI bridge; not tied to any
  publishing account yet.
- **Consequence:** Changing it later breaks saved session paths and installed-update continuity.

### D010 - Verification policy: one device, no API 29 floor test, CI only at public release
- **Decision:** Verification runs on the connected physical phone (currently a Samsung SM-G930F,
  API 34). A second device is not required, and behaviour at `minSdk` 29 is not exercised.
  Continuous integration is wired only for the public release (roadmap 12.1); until then builds
  and tests are run locally.
- **Reason:** Set by the project owner to keep the loop short; the minimum SDK stays 29 as a
  support claim, not as a tested configuration.
- **Consequence:** `agents.md` section 2 and the Phase 0 acceptance line in `roadmap.md` were
  updated to match. Runtime risk is concentrated on the one test device.

### D011 - googletest v1.15.2 for C++ unit tests
- **License:** BSD-3-Clause.
- **Reason:** Roadmap 0.4 prescribes gtest. It is fetched with CMake `FetchContent` (tag v1.15.2,
  shallow clone) into the host test build only, because the app's `libscand3d_core.so` is
  arm64-v8a and cannot be loaded by the x86_64 host JVM. The core sources are compiled for the
  host instead, so no test framework ships in the APK.
- **Consequence:** The first `nativeTest` run needs network access; afterwards CMake reuses the
  fetched sources in `build/native-tests`.

## Open items
- CI pipeline (roadmap 12.1) - not set up yet; builds and tests are verified locally (D010).
- Phase 2 device acceptance (roadmap 2.1-2.6) - code and host tests are complete, the hardware
  measurements are outstanding because no device was available (D021).
- Feature detector, meshing strategy and segmentation model (agents.md section 7) - still open,
  decided in Phases 3, 7 and 8.

### D008 - NDK 26.1.10909125 (r26b) and CMake 3.22.1
- **License:** NDK (Apache-2.0 with LLVM exceptions), CMake (BSD-3-Clause).
- **Reason:** 26.1.10909125 is the NDK version AGP 8.5.2 defaults to, and 3.22.1 is the CMake
  version packaged with it, so the native build needs no extra configuration. C++17 per
  agents.md section 2, `-DANDROID_STL=c++_static` so the APK carries one shared library.
- **Consequence:** Both live in the project-local SDK (D003). Native sources are compiled by
  CMake into a single `libscand3d_core.so`; the app loads it from `Scand3dApplication`.

### D009 - Single JNI bridge, one `external fun` per call
- **Reason:** agents.md section 3 allows exactly one JNI bridge. The Kotlin side is a `NativeCore`
  object whose `loadError` is captured instead of thrown, so a missing `.so` degrades to a UI
  message instead of a launch crash, and native functions are declared `external` one-to-one with
  `extern "C"` entry points in `core/bridge/jni_bridge.cpp`.
- **Consequence:** Every future native call is added there and reaches C++ through the interface
  header `core/bridge/bridge_api.hpp`, so nothing else in Kotlin talks to JNI directly.

### D012 - Synthetic scenes live in `tests/synthetic/`, camera axes follow OpenCV
- **Reason:** Roadmap 0.5 asks for a known mesh with simulated cameras as a test utility, and
  Phases 3, 4, 6 and 8 all need ground truth to measure against. The utility is test-only, so it
  stays out of `core/` and never enters the APK, but it is compiled into the shared host test
  library (`scand3d_test_util`) so any later test can use it.
- **Camera axes:** +X right, +Y down, +Z forward (OpenCV), `+Y` therefore increases down the
  image. `agents.md` fixes the pose convention (camera-to-world, float64, meters) but not the
  axis orientation, so it is pinned here before any geometry code depends on it.
- **Consequence:** The turntable object is two stacked, offset boxes with two albedo values - no
  symmetry, so a pose estimate cannot be ambiguous. Rendering is z-buffered with back-face
  rejection and Lambert shading, and is fully deterministic for pixel-exact assertions.

### D013 - Capture, preview and session storage live in Kotlin
- **Reason:** agents.md splits Kotlin (UI and app shell) from C++ (reconstruction core, image
  processing, rendering) but Camera2, SensorManager, EGL and file storage are Android platform
  APIs. Calling them through JNI would add a bridge for no benefit, so capture stays Kotlin under
  `app/.../capture/` and `app/.../render/`, and C++ keeps the algorithms.
- **Consequence:** `core/bridge/` stays as small as possible; nothing in Kotlin touches camera
  state from the UI thread.

### D014 - Monochrome preview through a GL external-texture shader
- **Reason:** Roadmap 1.4 asks for a luminance-only preview. Camera2 feeds a `SurfaceTexture`
  (external OES) and the fragment shader collapses RGB to luminance, so no colour reaches the
  screen; saved stills stay colour JPEGs because reconstruction needs them.
- **Consequence:** Preview is letterboxed to the camera aspect ratio; draw calls happen only when
  the camera delivers a frame (`RENDERMODE_WHEN_DIRTY`).

### D015 - EXIF via `android.media.ExifInterface`, focus distance in the manifest only
- **Reason:** The platform `ExifInterface` writes exposure time, ISO and capture time without a
  new dependency, keeping a frame readable on its own. EXIF has no field for lens focus distance,
  so the full photometric set is always written to `session.json`.
- **Consequence:** `DateTime` is the wall clock at capture, not the sensor timestamp, because
  `SENSOR_TIMESTAMP` is boot-relative.

### D017 - Camera-loss tests fire the real device callback; terminal states ignore late surfaces
- **Reason:** This phone's HAL cannot be made to drop a capture session from `adb`, and a second
  device is out of scope (D010). The debug-only `loss` command therefore posts the controller's
  stored `CameraDevice.StateCallback.onDisconnected(device)` onto the camera thread, the same
  entry point the OS uses, so the production teardown, `CameraLost` state and Retry action are
  what a device test exercises. The receiver and the bridge exist only in debug builds.
- **Consequence:** `CaptureViewModel.onPreviewTextureReady` ignores surfaces while the state is
  `CameraLost` or `Failed`. Without that guard a surface callback arriving after teardown
  restarted the camera and silently replaced the loss screen with a live preview, so the user
  never saw the error they should retry from.

### D016 - Debug-only broadcast trigger for device tests
- **Reason:** Driving capture through screen taps was unreliable on the test device. A receiver in
  the `debug` source set (`com.scand3d.debug.CaptureTestReceiver`, explicit component broadcast)
  starts and stops capture deterministically. It is not part of a release build.
- **Consequence:** Verification commands target the receiver by component; release builds expose
  no extra entry point.

### D018 - Mahony filter, fixed sensor rates and a gravity/magnetometer fallback
- **License:** none; `core/sensors/` is our own C++17 code, so no third-party dependency is added.
- **Reason:** agents.md section 7 allows Madgwick or Mahony. Mahony was chosen because its
  correction step is short enough to audit line by line and is unit-testable against synthetic
  gravity and magnetic vectors; Madgwick's gradient adds a Jacobian whose sign errors show up as a
  slow drift that a short test would miss. Rates: gyroscope 5 ms (~200 Hz), accelerometer 10 ms,
  magnetometer and rotation vector 20 ms. The fusion integrates on the gyro timestamps and clamps
  `dt` to 0.5-100 ms, so a delayed or duplicated sample cannot blow the state up.
- **Fallback (2.5):** without a gyroscope the same session computes orientation from gravity
  (`tiltOrientation`) and smooths it with a normalized lerp. Pitch and roll are observable; yaw
  exists only if a magnetometer does, which is exactly what the reduced-accuracy hint tells the
  user.
- **Consequence:** fixed gains (proportional 0.6, magnetic weight 0.5) and no gyro bias estimate.
  A capture burst lasts seconds and Phase 4 bundle adjustment refines poses, so short-term drift is
  acceptable; if the device run shows otherwise, an integral term is the next step.

### D019 - Session manifest schema 2 carries the IMU snapshot per frame
- **Reason:** Roadmap 2.3 needs the camera/sensor offset per frame, and agents.md section 3 says
  every captured frame carries an IMU orientation snapshot. Schema 1 had no place for either.
- **Shape:** each frame entry gains `orientation_wxyz` (w, x, y, z), `imu_timestamp_ns`,
  `imu_time_offset_ns` and `keyframe_candidate`. The manifest gains `clock`
  (`camera_source`, `sensor_source`, `offset_ns`, `offset_mad_ns`, `samples`) and `imu` (`mode`,
  `reduced_accuracy`, `keyframe_candidates`).
- **Consequence:** `schema` is now 2. Every added field is optional, so a schema 1 session still
  reads; new readers must key on `schema` instead of assuming the fields exist.

### D020 - Per-device clock profile measures the residual, it does not assume zero
- **What is measured:** the median and median absolute deviation of
  `sample_timestamp_ns - frame_timestamp_ns` over the session's pairings. Both Android streams are
  boot-relative (`elapsedRealtimeNanos` on API 29+), so what remains is pipeline residual plus any
  device that reports `SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN`. The camera's own source is recorded
  in the manifest rather than assumed.
- **Storage:** `files/devices/<model>.profile`, a `key=value` text file. Deliberately not JSON:
  the store is covered by host unit tests, where the platform `org.json` classes are stubs, and a
  file this small does not justify a test-only dependency (agents.md section 5, rule 5).
- **Use:** the next session on the same model shifts its pairing lookup by the stored offset; the
  in-session measurement is what the manifest records.
- **Consequence:** this is a measurement with an uncertainty, not a calibration certificate.
  Phases 4 and 5 must carry `offset_mad_ns` along and not treat the offset as exact.

### D021 - Phase 2 hardware acceptance deferred, not waived
- **Decision:** roadmap 2.1-2.6 are implemented and host-tested, and stay `[~]` until the device
  run recorded in `docs/benchmarks/phase2-imu-verification.md` is done. No phone was connected for
  this pass, so none of the hardware acceptance was run.
- **Verified instead:** 18 new gtest cases (quaternion math, Mahony convergence on synthetic
  gravity and magnetic vectors, tilt fallback, keyframe window, handle lifetime and malformed
  input) and 36 new JUnit cases (availability mapping, nearest-sample pairing, blur hysteresis,
  residual statistics, keyframe policy, profile persistence, pipeline glue); `assembleDebug`
  packages the new native symbols.
- **Not verified:** real sensor rates, orientation error against a reference rig, the measured
  offset on the SM-G930F, the no-gyroscope path on hardware, and preview frame rate with the
  sensor thread running.
- **Consequence:** the M1 milestone (Phases 0-2) is not claimable yet, and the Phase 2 acceptance
  numbers must not be reported as measured. Phase 3 work can proceed because feature code does not
  depend on those numbers, but the device run has to happen before the M1 claim.

### D022 - FAST-9 on a three-level pyramid, unsteered BRIEF-256 descriptors
- **License:** none; `core/feature/` is our own C++17 code. No OpenCV, no Eigen.
- **Reason:** agents.md section 7 allows SIFT-like or ORB, and ORB's FAST corner is short enough to
  audit and cheap enough for a phone. Detector: FAST-9 with threshold 20 on three levels (level 0
  blurred with sigma 1.2, then half-resolution levels blurred with sigma 1.0), 3x3 non-maximum
  suppression by response, 2 px cross-scale dedup keeping the stronger keypoint, at most 1200
  features. Descriptor: 256 fixed comparison pairs inside a 31x31 patch, bilinear sampling, pattern
  generated once from a fixed seed so builds stay comparable.
- **Steering removed after measurement:** the intensity-centroid angle is still computed and stored
  in `Keypoint.angle_rad`, but `describeBrief` is called with zero rotation. On the textured-box
  pair, steering the pattern by that angle reduced verified inlier precision from 95.7% (22 of 23)
  to 78% (18 of 23): the centroid of a low-texture quad corner moves between views, so the pattern
  was rotated by a different angle in each image.
- **Consequence:** matches survive the 30 degree turntable step but not an in-plane camera rotation
  yet. Rotation-aware matching (for example trying the stored angle, or several orientations) is a
  later task, and `angle_rad` is metadata until then. A half-resolution view matches only the
  pyramid level with the same physical footprint: 79.5% of those bridged matches were consistent
  within 3 px, which is a measured limitation, not scale invariance.

### D023 - Feature storage is a versioned little-endian binary, not JSON
- **Format:** magic `S3DFEAT1`, `uint32` version 1, `uint32` count, then one record per feature:
  five `float32` values (`x_px`, `y_px`, `scale`, `angle_rad`, `response`) followed by the 32
  descriptor bytes. 52 bytes per feature, so 1200 features are about 61 KB.
- **Reason:** a JSON manifest for the same data is several times larger, loses exact float bits and
  needs a JSON dependency on the host tests (agents.md section 5, rule 5). Binary keeps the store
  testable with plain file IO and keeps descriptor bits byte-exact, which matching requires.
- **Validation:** the loader rejects a wrong magic, an unknown version, a count above 1e6 and any
  file whose size does not match `12 + count * 52`. Truncation cannot be read as a shorter valid
  set.
- **Consequence:** the layout is fixed little-endian; adding a field means bumping the version and
  keeping the version-1 reader, because stored sessions outlive the build.

### D024 - Exhaustive Hamming matching plus a multi-index approximate matcher, ratio 0.8
- **Reason:** 256-bit descriptors are compared by Hamming distance, so the exhaustive matcher is a
  straightforward reference. The approximate matcher splits the descriptor into eight 32-bit
  blocks, indexes each block separately and compares only descriptors that share at least one
  block with the query; a bucket with fewer than 6 or more than 400 entries falls back to the
  exhaustive search, so the fast path never silently loses candidates.
- **Ratio test:** the best match must be better than the second best by `ratio_percent` 80. `bestTwo`
  distinguishes "no second candidate" from "second distance is zero": an earlier version used 0
  for both and rejected a descriptor that matched itself exactly.
- **Cross-check:** offered by the exhaustive matcher only. The approximate matcher would need a
  reverse index over the query set, and callers that want it can use the exhaustive path.
- **Consequence:** measured recall of the approximate matcher against exhaustive passes 0.85; the
  approximate path is for Phase 4's larger maps, where exhaustive search stops being affordable.

### D025 - Hartley-normalized eight-point inside RANSAC, with iterative refit
- **Reason:** the eight-point algorithm is the smallest estimator the roadmap allows, and Hartley
  normalization keeps the 9x9 normal equations well conditioned at pixel scale. The required
  linear algebra (Jacobi eigen solver for the 9x9 system, 3x3 SVD for rank enforcement) is written
  in `core/geometry/linalg.cpp`; Eigen is not used anywhere (agents.md section 2).
- **Constraints:** the fundamental matrix has its smallest singular value forced to zero; the
  essential matrix has its two largest singular values set equal, which is the calibrated
  constraint. Inliers are measured by Sampson distance in pixels, and the essential path converts
  the pixel threshold into normalized coordinates by the geometric mean focal length.
- **Refit:** after RANSAC picks the best minimal-sample model, the estimator alternates refitting on
  all inliers and reclassifying, at most four rounds, until the inlier set settles. One refit left
  the model visibly biased (median Sampson 0.91 px on the synthetic pair); the loop brings it below
  0.5 px and makes the reported inliers match the matrix that is actually returned.
- **Adaptive stopping:** the required iteration count is computed and compared as a `double`, and it
  may only shorten the budget. The first version cast the estimate to `int`; for a weak inlier ratio
  the value is astronomically large, the cast overflowed, and RANSAC silently stopped after one
  iteration with an invalid result.

### D026 - Phase 3 acceptance: synthetic half measured, real-object half deferred
- **Decision:** roadmap 3.1-3.5 are implemented and host-tested, and stay `[~]` until a real image
  pair is run. The synthetic half of the acceptance line is met: 95.7% of the RANSAC inliers (22 of
  23) map to the same known 3D point, and the geometry suite recovers F and E from 35% outliers at
  0.35 px noise.
- **Not run:** the real-object requirement of at least 100 verified inliers. There is no phone and
  no real image pair in this environment; a real pair has no ground truth beyond the visual overlay.
  No number for real data is reported anywhere in this repository.
- **Commit shape:** Phase 3 landed in two commits, code and tests first, then documentation, because
  the tasks share the same files and a per-task split would have produced commits that do not
  build on their own. This is the same deviation recorded for Phase 2.
- **Consequence:** the pipeline is not yet reachable from the app; `core/bridge/` still exposes only
  the Phase 2 handle API, and Phase 4 adds the calls that consume matches.

### D027 - Phase 2 device run on the SM-G930F: measured, fixed, still open
- **Measured:** the K6DS3TR accelerometer and gyroscope deliver events 10.0 ms and 5.03 ms apart
  (~100 Hz and ~199 Hz), matching the rates D018 chose; the app reports `imu mode=NineAxis` with
  `reduced_accuracy=false`. A schema-2 session carries 17 of 17 frames with orientation and offset,
  and the `clock`/`imu` blocks. Preview holds 27.1 fps median (33.3-38.3 ms intervals) while stills
  save, with thermal status `NONE`.
- **Clock:** measured offsets are -373.5 us, -699.5 us and -431.2 us across three sessions with MAD
  0.93-1.20 ms. The per-device profile stores the at-stop snapshot (-431.2 us, 16 samples), which
  can differ from the manifest's final value (17 samples) when a late frame lands after the stop
  flush. The profile is a starting point for the next session, not a record of this one (D020).
- **Fixed:** the late-frame manifest rewrite passed no session metadata, so the last session of a
  run lost its `clock`/`imu` blocks (session `20261104-151236`, 13 frames, no clock block). It now
  routes through `flushManifest` and the next run kept both (`1521eaf`).
- **Still open:** the roadmap's orientation error bound of 2 degrees is **not measured**. The
  substitute procedure (a 90 degree table turn with the phone aligned to a table edge) was declined
  during this pass, so there is no reference number, and the 4.63 degree change seen across a
  static 10 s hold is motion or drift, not accuracy. The no-gyroscope fallback also remains
  unexercised on hardware, because this device has a gyroscope and D010 rules out a second one.
- **Consequence:** roadmap 2.1-2.6 stay `[~]` and M1 stays unclaimable; Phase 2 must not be
  described as accepted, only as device-smoke-tested. Closing it needs a reference rig (or an
  agreed substitute) and either a gyro-less device or a debug sensor-injection path.

### D028 - Phase 4 core: DLT plus Gauss-Newton, and a bundle adjustment that only factorises cameras
- **Reason:** agents.md rules out COLMAP, OpenCV and Eigen, so triangulation, PnP and bundle
  adjustment are written here. Triangulation and PnP both start from a linear DLT solve (smallest
  eigenvector of the 4x4 or 12x12 normal equations) and finish with a few Gauss-Newton steps, which
  removes the algebraic bias without a second estimator. Bundle adjustment is Levenberg-Marquardt
  with the Schur complement on the point blocks, so only the 6N camera system is factorised; with
  64 points and 40 views that is the difference between a 240x240 and a 2106x2106 solve.
- **Gauge:** the first registered camera is held fixed. That leaves the global scale free, which is
  not a shortcut but the nature of monocular SfM: relative rotation and the direction of each
  baseline are observable, absolute distance is not. The acceptance numbers are therefore reported
  gauge-free (relative rotation, baseline direction), and a two-view seed has a unit baseline by
  construction.
- **Conventions:** poses are camera-to-world `(R, C)`; a step is `R <- R * Exp(delta)` and
  `C <- C + delta`, so `dXc/dtheta = [Xc]x` and `dXc/dC = -R^T`. The normal equations are
  `H d = J^T r`, so `d` is the ascent direction and every update subtracts it. Huber loss at 2 px
  keeps a stray observation from dominating a camera.
- **Consequence:** on the 16-view synthetic scan the mean reprojection error is 0.588 px and the
  worst pose error 0.261 degrees, both well inside the roadmap bounds.

### D029 - A seed pair is trusted only after a third view registers against it
- **Reason:** both decompositions of an essential matrix reproduce the same two images, so a
  mirrored pair fits its own pair exactly. Neither cheirality nor reprojection error can choose
  between them: on the same synthetic scan the mirror scored *better* (0.08 px median) than the
  correct pair (0.2-0.7 px). Only a third view contradicts a mirror.
- **Rule:** `reconstruct` seeds from the pair with the most matches, triangulates, tries to register
  any other view, and moves to the next pair (at most eight attempts) when nothing grows. Within the
  seed, cheirality decides between the four candidates and the reprojection error only breaks ties.
  Failed views are re-evaluated on every pass instead of being skipped, and a pass that registers
  nothing ends the growth loop.
- **Two-view bundle adjustment is refused:** with two cameras BA can trade the whole reconstruction
  for the mirrored one at the same cost, so `finalize` only runs it from three views up. Growth is
  also swept over all unregistered views per pass rather than extending from one end, so a scan
  seeded in the middle grows in both directions.
- **Consequence:** before this rule the same scan reconstructed correctly at 16 views and locked
  onto a mirror at 12 and 24; every view count from 12 to 40 now reconstructs fully. The cost is
  that seeding may evaluate up to eight pairs, which dominates the runtime at 40 views (30 s).

### D030 - Phase 4 acceptance: synthetic half measured, real-scan half deferred
- **Decision:** roadmap 4.1-4.6 are implemented and host-tested, and stay `[~]` until a real scan is
  run. The synthetic half is met with margin: 16 of 16 views registered, mean reprojection 0.588 px
  and worst pose error 0.261 degrees on the 16-view turntable scan, with the same behaviour from 12
  to 40 views and at twice the nominal pixel noise.
- **Not run:** the real-object requirement of at least 90 percent registered keyframes. There is no
  real capture in this repository and no app-side path from captured frames to matches, so there is
  nothing to point at a real object; a real scan also has no ground truth beyond a visual check.
  No real-data number is reported anywhere.
- **Commit shape:** Phase 4 lands in two commits, code and tests first, then documentation, for the
  same reason as D026: the tasks share the same files and a per-task split would produce commits
  that do not build on their own.
- **Consequence:** Phase 5 (live overlay) can consume this module, but the bridge still exposes only
  the Phase 2 handle API. Wiring capture -> features -> matches -> reconstruction is the next piece
  of work, and the 90 percent real-scan measurement depends on it.

### D031 - Overlay camera contract: displayed-preview pixels, one rectangle, no depth buffer
- **Decision:** `core/overlay` projects reconstructed points through a pinhole camera whose
  intrinsics are expressed in *displayed preview image pixels* (origin top-left, +x right, +y
  down) and whose pose is camera-to-world, matching `core/sfm` (D012, D028). It returns NDC, and
  the renderer draws the points into the same letterboxed rectangle it draws the camera image
  into, so image pixels and NDC line up one to one at any surface shape.
- **Reason:** the preview is letterboxed by aspect fit. Computing that rectangle twice, once for
  the background and once for the overlay, is exactly how a point cloud ends up a few pixels off
  when the view size changes. One rectangle, one mapping, and the projection stays a pure function
  that host tests can check.
- **Ordering:** vertices are sorted far to near in C++ and drawn with blending and no depth
  buffer. A sparse overlay does not justify a depth attachment on a surface that already carries
  the camera texture, and painter order is exact for point sprites.
- **Gauge:** points and poses come from Phase 4, where the first camera fixes the frame and the
  global scale is free, so the overlay is defined up to that similarity until the capture pipeline
  pins a metric scale. The overlay therefore cannot claim a physical size yet.
- **Consequence:** the app has no reconstruction output yet (D030), so roadmap 5.1 is verified with
  a debug-only canned cloud behind the debug receiver, marked `// STUB` in the code and in
  `roadmap.md`. It stays `[~]` until it draws real points.

### D032 - The overlay camera is rotation-only until tracking exists
- **Decision:** the overlay camera takes its intrinsics from the camera itself (focal length from
  `LENS_INFO_AVAILABLE_FOCAL_LENGTHS` scaled by `SENSOR_INFO_PHYSICAL_SIZE`, principal point at
  the image centre) and its orientation from the fused IMU quaternion relative to the pose the
  overlay was anchored at, composed with a body-to-camera extrinsic that follows the display
  rotation (`OverlayGeometry`). Its centre is the origin: translation is not estimated.
- **Reason:** there is no tracking pose in the app yet (D030), and inventing one would put a
  plausible-looking overlay on screen that is wrong in a way nobody can measure. A rotation-only
  overlay is honest about what the sensors know: rotating the phone moves the cloud as if it were
  fixed in the room, moving the phone does not.
- **Consequence:** roadmap 5.2 is done for the IMU half and open for the tracking half, and 5.5
  (alignment within 10 px after 10 s of handheld motion) cannot be measured, because handheld
  motion includes translation. The instrument that will measure it is in place: the view model
  logs, once a second, the anchored-to-live pose angle and the projected cloud centre in NDC.
- **Not verified on device:** the sign of the display-rotation step is asserted as
  `ROTATION_90`-one-quarter-turn in `OverlayGeometryTest`; whether that rolls the cloud the same
  way the preview is rolled has to be checked by rotating the phone while the overlay is on.

### D033 - Keyframe colours are sampled through the app, not in the core
- **Decision:** the reconstruction core stays gray (D022), so overlay colours come from the app:
  `KeyframeColorSampler` decodes the newest saved keyframe JPEG, packs it to 3-byte RGB, and hands
  it to `overlay::samplePointColors`, which takes the per-channel median of a 3x3 patch around each
  point's projection. The median keeps one specular or anti-aliased pixel from tinting a point.
- **Reason:** decoding JPEG is an Android codec job and the core is forbidden a decoder anyway
  (agents.md section 2 allows libjpeg, but the feature pipeline is defined on gray images).
  Keeping sampling in `core/overlay` means it is unit-testable without Android.
- **Consequence:** one keyframe and one camera are sampled per call; a point seen by several
  keyframes takes the colour of the keyframe it was sampled from, not a cross-view average. That is
  enough for a sparse overlay and is recorded here rather than hidden.

### D034 - Phase 5 status: 5.1-5.4 implemented, 5.5 open
- **Decision:** 5.1 to 5.4 are marked `[~]` and 5.5 stays `[ ]`. `[~]` means the code, the host
  tests and the wiring are in place, and what is missing is real reconstruction input, not more
  work on the task itself.
- **Reason:** the same convention as Phases 2, 3 and 4, and the same cause: D030 left the app
  without a capture-to-reconstruction path, so nothing in Phase 5 can be measured against a real
  object.
- **Consequence:** Phase 5 cannot be called accepted until a real cloud exists and 5.5 has a
  number. The next useful piece of work is that wiring, not more overlay code.

### D035 - The app reconstructs through an incremental session, not the global-track pipeline
- **Decision:** `sfm::Session` (`core/sfm/session.{hpp,cpp}`) owns capture-to-map: it matches
  neighbouring views only (gap at most two), gates every pair through `estimateEssentialRansac`,
  seeds from the richest pair `initializeTwoView` accepts, then registers one view at a time by PnP
  against the map and triangulates only the matches the new pose explains. It does not call
  `sfm::reconstruct`.
- **Reason:** on rendered feature matches the global-track path welded correspondences from
  different pairs into single tracks. Twenty-four rendered views of the textured box reconstructed
  2 views and 1 point across every scene setting tried, while the same seed pair triangulated to
  0.5 px mean reprojection. The Phase 4 pipeline was validated on exact lattice tracks, so it is
  left unchanged.
- **Consequence:** the session registers three of twenty-four views at 0.375 px mean reprojection
  on the 24-view turntable (D037). Growth stops there because the ring's weakest neighbouring pair
  yields about 16 geometrically consistent matches, too few to place another view by PnP or to
  anchor a chained relative pose. The 90 percent real-scan number stays open, so 5.2's tracking half
  remains `[~]`. The seed map covers a mostly planar box face, so a tracked pose is fixed only up to
  the planar flip; the tests assert that tracking returns a pose and refuses a featureless frame,
  not a rotation bound.

### D036 - The app reconstructs from saved keyframes decoded at 480 px
- **Decision:** `ReconstructionRunner` decodes up to 12 evenly spaced keyframes of the newest
  session, converts them to 8-bit gray with the 0.299/0.587/0.114 weights the preview shader uses,
  and passes the overlay camera's intrinsics scaled to the decoded size
  (`OverlayCameraParams.scaledTo`). The resulting points replace the demo cloud in the overlay scene
  and keep the keyframe colour sampling. It runs on its own thread from
  `CaptureViewModel.reconstructLatestSession`, requested with `adb shell am broadcast -a
  com.scand3d.debug.CAPTURE --es cmd reconstruct`.
- **Reason:** even spacing keeps the baseline spread of a short capture, decoding below preview
  resolution keeps the native side to a few hundred milliseconds per frame, and scaling the
  intrinsics is the only correct way to reuse one calibration at another resolution.
- **Consequence:** reconstruction is a debug-triggered action, not a review screen: it handles the
  newest session only and reports nothing when fewer than two frames are usable. The overlay still
  publishes on the IMU pose (D032) until a tracked pose is wired into the display pose.

### D037 - Seeds are scored by third-view support and thin maps bootstrap by chaining
- **Decision:** every candidate seed pair builds its own map; each map is scored by how many of its
  points the best neighbouring pair can actually match, and only the best few are grown. A view is
  registered by PnP when the map supports it, and otherwise by chaining that pair's own relative pose
  onto the registered reference, scaled by the median ratio of map depth to relative depth over the
  correspondences the map already knows. Bundle adjustment runs only after a third view registers.
- **Reason:** with exact correspondences the seeder is sound (162 of 162 points at 0.23 px), but on
  real matches the same pair can land on the planar mirror branch about 20 degrees off and keep only
  a handful of points. Scoring by support picks the branch that can grow instead of the one with the
  most matches, which is what the previous count-order retry did. A two-view map cannot place a third
  view by PnP at all, and D029 records that bundle adjustment on two views can trade the
  reconstruction for a mirrored one.
- **Consequence:** on the 24-view turntable the session now registers three views at 0.375 px mean
  reprojection and 51 points, deterministically for that scene seed, where the previous commit
  registered only the seed pair. Further growth is blocked by the fixture rather than the loop: the
  weakest neighbouring pair has about 16 matches, so nothing anchors the next pose. The same loop
  needs a capture with denser matches, which only real frames can show.

### D038 - A value-noise fixture, an essential-matrix fallback, and the planar PnP limit
- **Decision:** `tests/synthetic` gains `makeNoiseTexturedBoxScene`, which shades the box from a
  smoothstepped value-noise field over surface coordinates instead of the flat per-cell albedo of the
  Phase 3 box. The field is opt-in through `Scene::value_noise_texture`, so every existing scene
  renders byte-identically. Two estimator changes came out of using it: essential-matrix RANSAC now
  samples minimal sets that span the image (a sample from one face is planar and fits that face
  only), and when the calibrated fit keeps fewer than half the matches it falls back to estimating F
  in pixels and calibrating it with E = K2^T F K1, which is exact rather than approximate.
- **Reason:** the flat-cell mosaic gave neighbouring views as few as 24-34 geometrically consistent
  matches while the same geometry elsewhere gave hundreds, and the equal-singular-value fit collapsed
  on a mostly planar surface (34 essential inliers against 296 fundamental inliers on the same 310
  matches). Both are artefacts of the estimator and the fixture, not of the reconstruction loop.
- **Consequence:** the noise fixture raises a neighbour pair to 296 inliers, and the session grows to
  three views and 114 points at 0.375 px mean reprojection. It then stops at a different, genuine
  limit: the map lives on a box face, and `solvePoseDlt` is degenerate for coplanar points, so PnP
  reports no valid pose even with 96 mapped correspondences and the next view is rejected. Planar
  PnP (homography decomposition, or a plane-aware DLT) is the next piece; until it exists the loop
  cannot be measured on a whole scan, and the overlay keeps drawing the debug cloud.

### D039 - Reconstruct during the capture, and display the cloud in the anchored frame
- **Decision:** `LiveReconstruction` owns one native session for the whole capture. Each IMU
  keyframe is queued from the frame-writer thread, decoded to 480 px gray on a single worker, added
  to that session, and the map is re-run at most every 1.5 s. The newest points replace the overlay
  scene. Before projection the cloud is rotated by the anchor camera rotation and rescaled so its
  median distance is 0.55 m. The debug `reconstruct` command stays as the saved-session path.
- **Reason:** reconstruction only ran on a finished session through a debug broadcast, so the scan
  showed nothing while it was being recorded. On device that loop also exposed two display bugs: the
  cloud's median extent was 17.6 m (the reconstruction scale is free, D028) and every point had
  negative depth, because reconstruction points are in the first camera's +Z-forward frame while the
  overlay scene is anchored in the body frame where that axis is -Z (D031). The demo ring had the
  same sign error, so the render path had never actually drawn a point.
- **Consequence:** a 22 s capture on the SM-G930F grows to eight views and eight points at 0.6-3 s
  per update, and every point projects into the preview (`8 of 8`). The view cap is eight because
  each update re-matches every pair (12 views cost 14 s), so the loop is not yet incremental; a pair
  cache in the session is the next step. Map growth on a real scene still stops where D038 says,
  coplanar PnP, which is why the point counts are small.

### D040 - Turn the preview upright, and make the overlay readable on it
- **Decision:** the preview renderer takes the camera's sensor orientation and rotates the sensor
  image by whole quarter turns, letterboxing by the *rotated* aspect; the overlay point shader
  applies the same quarter turn to its NDC, so the projection computed in the sensor image still
  lands on the right pixels. Points are drawn at 10 px and lifted 35% towards white. The capture
  controller rebuilds its handler thread when the previous one has been quit, and clears the field
  when the camera is lost.
- **Reason:** the app is portrait-locked and the sensor is landscape, so a 1280x720 image was drawn
  as a 16:9 band inside a 9:16 surface: the preview looked squashed and the overlay rectangle did
  not match the visible image. Points were also 6 px and painted the scene's own colour, so a
  correct cloud was invisible in practice. Separately, re-entering the app after a camera loss
  crashed the GL thread, because `start()` built a `Handler` from a thread whose looper had been
  quit.
- **Consequence:** on the SM-G930F the preview fills the screen upright and the live cloud reads
  clearly during a capture (124 points at eight views, all projecting). Verified with device
  screenshots rather than by inference; the frames are in `app/build/shot*.png`.

### D041 - The capture screen follows the reference camera layout
- **Decision:** the capture screen is laid out like the reference camera app: the preview fills the
  top edge to edge, a control strip under it carries the focus-lock indicator, the camera's fixed
  1x value and the colour-overlay toggle, then a divider, a shutter row with the newest keyframe as
  the gallery tile, a round shutter and a flip glyph, and mode tabs with SCAN active. The preview
  renderer crops the upright image to the preview box's aspect instead of letterboxing it, and the
  overlay points are scaled by the same crop so projection stays aligned. The opacity slider is a
  slim white strip rather than a themed Material slider.
- **Reason:** the screen was a status panel, a full-width text button and a Material slider stacked
  over the preview, which reads nothing like a camera app. Cropping is also what keeps the preview
  free of black bars inside the boxed area: the sensor image is 9:16 and the box is 0.83.
- **Consequence:** verified on device (`app/build/ui1_small.png` idle, `app/build/ui2_small.png`
  mid-capture):
  during a capture 123 of 123 points project over the live image. PHOTO and VIDEO tabs and the flip
  glyph are drawn dimmed and are not interactive, because the app scans only and has one camera; the
  focus square is a lock indicator, not a control, so it never invites a tap it cannot honour.

### D042 - The preview follows the camera's own texture transform, not the sensor angle
- **Decision:** the preview renderer reads the SurfaceTexture transform and treats a transposing
  matrix, `(u,v) -> (1-v, 1-u)`, as the orientation the camera already delivers: the preview gets no
  extra rotation, the displayed aspect uses the transposed dimensions, and the overlay points take
  exactly that one quarter turn, because they are projected in the untransposed image frame. The
  preview stream is 4:3 (1280x960, else the closest 4:3), and a debug broadcast
  `orientation <deg>` forces a quarter turn so the orientation can be calibrated on a device.
- **Reason:** D041 rotated the preview by the sensor orientation, which on the SM-G930F turned an
  already-upright image 90 degrees sideways: its preview buffer arrives transposed, and the
  transform carries that. Forcing each of the four rotations on the device and comparing the screen
  with a captured still showed 0 degrees is upright and the other three are sideways or inverted.
  The 16:9 preview stream also forced a 35% vertical crop, which read as a stretched preview.
- **Consequence:** the preview is upright and only about 14% cropped (crop 1.0x0.86 logged on
  device), and the cloud lands on the same features it is drawn over. Devices whose transform does
  not transpose still follow the sensor angle and remain untested; the debug override exists so the
  next device can be calibrated in one broadcast rather than a rebuild.

### D043 - Anchor every reconstruction run to the first keyframe
- **Decision:** `Session::run` re-expresses its result in view 0's camera frame after growth and
  bundle adjustment: points and camera poses are rotated and translated so the first view sits at the
  origin with an identity rotation. The app anchors the overlay to the IMU pose of the keyframe that
  became view 0, and pre-rotates the cloud by the camera's own body extrinsic. A native test
  (`Session.KeepsEveryRunAnchoredToTheFirstView`) asserts camera 0 is the identity in two successive
  runs and that the map keeps its scale.
- **Reason:** each run re-seeds from whichever pair matches best (D037), and the seed pair's first
  camera becomes the frame origin, so on a growing view set the gauge changed between live updates.
  The cloud therefore changed pose and size while the phone and the scene stood still. The overlay
  also anchored itself to the pose at the moment the first cloud arrived rather than at the keyframe
  the map is built from, which added a constant rotation error.
- **Consequence:** successive live updates share one gauge, so a cloud drawn on a feature stays on
  it as the map grows and as the phone turns. Scale remains free (D028) and is still normalised for
  display, and the anchor is only taken once per capture.

### D044 - The live cloud leaves the phone as a PLY file that Blender 4.x polls over adb
- **Decision:** every live update writes the sparse map to `files/sessions/<stamp>/cloud.ply` in the
  app's own files directory: a binary little-endian PLY with `x`, `y`, `z` per point in the
  reconstruction's frame (first keyframe camera, free scale), written to a temporary file and
  renamed over the target. `tools/blender/scand3d_stream.py` is a Blender 4.0+ add-on that polls
  that file - through `adb exec-out run-as com.scand3d cat`, newest session by default, or from a
  local PLY - turns it into one mesh object with a geometry-nodes modifier that instances a small
  ico sphere per point, converts the axes to Blender's Z-up frame, and normalises the median point
  distance to a target size.
- **Reason:** agents.md forbids network calls during capture, so the transport is the existing USB
  adb connection rather than a socket; a file that is rewritten at each update keeps the app and
  the viewer independent, and needs no server, no new dependency and no bridge change. The viewer
  is an add-on script, not app code, so the sparse map is reachable from a desktop without waiting
  for the Phase 9 exporters. Polling (not push) keeps the phone side to a few lines and works with
  a scan that is already running.
- **Verified:** `blender --background --factory-startup --python tools/blender/selftest.py` passes
  19 checks on Blender 4.0.2 with no phone attached: the PLY reader (binary, ascii with reordered
  properties, truncated and non-PLY bodies), axis conversion and normalisation, the mesh object,
  instances, material and view camera, session selection and fetching through a stand-in `adb`
  executable, the streaming worker, and a Cycles frame with the cloud as the only object. Three JVM
  cases cover the writer's header, byte layout, empty cloud and rewrite.
- **Not verified:** the device leg never ran, because no phone was connected in this pass, so the
  `run-as` read on real hardware, the session listing and the live growth in Blender are untested
  against a scan. The stream carries no colour either: the overlay's sampled colours live in the
  native scene and reading them back needs a bridge call, which is a public-interface change and
  was not made (agents.md rule 5). Points are therefore drawn in one configurable colour.
- **Consequence:** the add-on shows geometry only, and its PLY reader accepts any `x`/`y`/`z`
  point PLY, so a hand-pulled or Phase 9 cloud opens the same way. A future colour or tracking-pose
  stream extends the file format, not the transport.
