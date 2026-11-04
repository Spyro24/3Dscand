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

## Current phase status

- Phase 0 complete: project, native bridge, local test harnesses, synthetic scenes, benchmarks,
  this document.
- Phase 1 complete: Camera2 capture with locked settings, monochrome preview, session storage,
  and camera-loss/permission-denied handling, all exercised on the test device. A lost camera is
  torn down on the camera thread, the UI shows the `CameraLost` screen, and Retry builds a fresh
  controller and re-locks; a preview surface arriving while the UI is in a terminal state is
  ignored (D017).
- Phase 2 implemented, hardware acceptance outstanding: sensors, Mahony fusion, the tilt
  fallback, the blur monitor, per-frame clock pairing and the keyframe prior are in place and
  covered by host tests, but no device was available to run the acceptance measurements (D021,
  `docs/benchmarks/phase2-imu-verification.md`).
- Everything after Phase 2 is unimplemented; `roadmap.md` is the source of truth.
