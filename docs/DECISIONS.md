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
