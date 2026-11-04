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

## Open items
- CI pipeline (roadmap 0.4) - not set up yet; builds are verified locally with `assembleDebug`.
- `docs/architecture.md` (roadmap 0.7) - not written yet.
- Feature detector, meshing strategy and segmentation model (agents.md section 7) - still open,
  decided in Phases 3, 7 and 8.
