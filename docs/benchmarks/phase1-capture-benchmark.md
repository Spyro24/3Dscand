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

## Camera loss and retry

This device's HAL cannot be made to drop a session from `adb`, so the loss path is driven through
`CaptureTestReceiver`'s debug-only `loss` command: it posts the controller's stored
`CameraDevice.StateCallback.onDisconnected(device)` onto the camera thread, which is the callback
the OS itself invokes. Teardown, state and UI are production code; only the trigger is debug
(`docs/DECISIONS.md` D017).

Sequence observed on the SM-G930F with a debug build:

| Step | Observation |
|---|---|
| `loss` command | `CameraCapture: camera lost: camera disconnected` on the camera thread, then `CaptureViewModel: camera lost: camera disconnected` |
| CameraLost screen | `Camera unavailable`, `camera disconnected`, `Retry`; held for 15 s of polling with no input |
| Tap on Retry | `retry requested from phase=CameraLost caller=...CaptureScreen.kt`, new controller opened the device and re-locked about 2 s later |
| Recovered preview | `Exposure and focus locked`, 30 fps, 0 coloured pixels in the 1440x768 preview band |
| Capture after recovery | 12 frames saved; `saved_frames: 12` with 12 files on disk |
| Metadata over the burst | one exposure (19990000 ns), one ISO (250), one focus (4.07 diopter) |
| Frame EXIF | `ExposureTime=0.0199`, `ISOSpeedRatings=250` |
| Camera clients after retry | exactly one `Client PID` in `dumpsys media.camera`, so the lost session is not leaked |

The retry run sits under different light than the 100-frame run above, which is why its locked
exposure and ISO differ; what matters is that each burst shares one photometric setting.

Two defects surfaced while proving this and are fixed:

- A preview-surface callback could arrive while the CameraLost screen was up and restart the
  camera behind it, silently undoing the loss screen. `CaptureViewModel.onPreviewTextureReady`
  now ignores a surface while the state is `CameraLost` or `Failed`.
- A still that was already in flight when capture stopped reached disk after the stop flush, so
  the first observed session had 12 files with `saved_frames: 11`. The manifest now derives
  `saved_frames` from the entries it lists and is rewritten when a trailing frame saves after
  stop; the re-run above shows 12/12.

## Idle behaviour

With a live preview and no input for 120 s, `files/sessions` was never created, the UI stayed at
`0 frames`, and neither `CaptureTest` nor `CaptureViewModel` logged a capture start: no
spontaneous capture. A capture then started by the debug command saved frames and stopped
cleanly, so capture still runs only on demand.

## Notes and limits

- The 27.1 fps during a burst is the preview rate while the JPEG pipeline is busy, measured from
  the camera clock; idle preview is 30 fps. Moving JPEG writes off the camera thread (they used
  to run inline) is what keeps the burst from collapsing the preview to ~4 fps.
- The loss path is proven through the stored device callback, not by a real HAL disconnect: the
  OS-triggered variant (`onDisconnected` from the camera service) was not reproducible on this
  phone without unplugging the camera from the framework. Everything downstream of the callback
  is the production path.
- The permission-denied path was exercised by revoking `CAMERA`, setting `user-fixed` and
  relaunching; it shows the "Camera access denied" screen with an "Open settings" action.
- Single-device verification is the project policy (`docs/DECISIONS.md` D010).
