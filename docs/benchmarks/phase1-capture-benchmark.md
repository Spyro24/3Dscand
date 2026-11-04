# Phase 1 capture benchmark

Measured on the connected test device with a debug build. Raw recorder output is JSON Lines in
the app's private storage (`files/benchmarks/bench-<stamp>.jsonl`) and was pulled with
`adb exec-out run-as com.scand3d cat ...`.

- **Device:** Samsung SM-G930F (heroltexx), Android 14 / API 34, arm64-v8a
- **Build:** debug, versionName 0.1.0
- **Preview:** 1280x720 SurfaceTexture, AE target range 30-30 fps
- **Still:** 4032x3024 JPEG, quality 95

## Results

| Metric | Value | How measured |
|---|---|---|
| Preview frame rate (idle) | **30 fps** | `fps` state sampled five times, 4 s apart |
| Preview frame interval during a still burst | 36.9 ms median (27.1 fps) | JSONL frame entries derived from the camera clock |
| Still capture rate | 1.6 frames/s | 100 frames in 62 s |
| Frames saved in one session | 100 | `session.json` `saved_frames` and 100 files on disk |
| Distinct exposure / ISO / focus over 100 frames | 1 / 1 / 1 | 30006000 ns, ISO 1600, 5.811 diopter |
| Peak process memory (PSS) | 272.8 MB | JSONL `memory` entries |
| Thermal state | NONE throughout | `PowerManager.currentThermalStatus` |

## Preview colour check

A `screencap` of the preview band (middle 30-60 % of the screen, 1,105,920 pixels) contained
**0 pixels where R, G and B differed**, maximum channel spread 0: the shader output is strictly
grayscale.

## Frame metadata

`frame_0100.jpg` opens as 4032x3024 and carries EXIF `Make=samsung`, `Model=SM-G930F`,
`Orientation=6`, `ExposureTime=0.03`, `ISOSpeedRatings=1600` and `DateTimeOriginal` set at
capture time. Focus distance is not an EXIF field, so it lives only in the session manifest.

Manifest paths are relative (`sessions/<stamp>/frames/frame_0100.jpg`); no absolute path is
stored.

## Notes and limits

- The 27.1 fps during a burst is the preview rate while the JPEG pipeline is busy, measured from
  the camera clock; idle preview is 30 fps. Moving JPEG writes off the camera thread (they used
  to run inline) is what keeps the burst from collapsing the preview to ~4 fps.
- Camera-loss handling is implemented (`CameraDevice.StateCallback.onDisconnected`/`onError` ->
  `CameraLost` state with a retry action) but was not reproduced on device; the permission-denied
  path was exercised by revoking `CAMERA`, setting `user-fixed` and relaunching, and shows the
  "Camera access denied" screen with an "Open settings" action.
- Single-device verification is the project policy (`docs/DECISIONS.md` D010).
