# Phase 0 device smoke test - native load and JNI bridge

- **Date:** 2026-11-04
- **Build:** debug APK, versionName 0.1.0, package `com.scand3d`
- **Commit:** fe491c9 ([0.3] add JNI bridge skeleton in core/bridge)
- **Install:** `adb install -r app-debug.apk` -> Success

## Device under test

| Field | Value |
|---|---|
| Model | Samsung SM-G930F (heroltexx) |
| Android release / API level | 14 / 34 |
| Primary ABI | arm64-v8a |

## Result

| Check | Result | Evidence |
|---|---|---|
| App installs and launches | PASS | `am start` reports `cmp=com.scand3d/.MainActivity`, process pid 15448 alive |
| Native library loads at app start | PASS | `I scand3d-native: libscand3d_core loaded` (JNI_OnLoad), `I Scand3dApp: native core loaded` |
| JNI bridge returns a value | PASS | UI node text `native core 0.1.0` |
| No crash on launch | PASS | no `AndroidRuntime` fatal entries |

logcat excerpt:

```
I scand3d-native: libscand3d_core loaded
I Scand3dApp: native core loaded
```

UI text from `uiautomator dump`: `3D Scan`, `Idle`, `native core 0.1.0`.

## Not covered

- Only one of the two devices required by the Phase 0 acceptance has run this build.
- The device reports API 34, so behaviour at minSdk 29 is untested.
- CI has not run this build (roadmap 0.4); the APK was built locally with
  `JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64 ./gradlew :app:assembleDebug`.
