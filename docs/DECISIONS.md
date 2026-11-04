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
- `docs/architecture.md` (roadmap 0.7) - not written yet.
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

### D016 - Debug-only broadcast trigger for device tests
- **Reason:** Driving capture through screen taps was unreliable on the test device. A receiver in
  the `debug` source set (`com.scand3d.debug.CaptureTestReceiver`, explicit component broadcast)
  starts and stops capture deterministically. It is not part of a release build.
- **Consequence:** Verification commands target the receiver by component; release builds expose
  no extra entry point.
