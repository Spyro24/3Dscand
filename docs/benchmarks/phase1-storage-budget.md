# Phase 1 storage budget: keyframe-only sessions (D050)

Measured on the SM-G930F (Android 8.0), debug build, 4 November 2026.

## Why this exists

Every still was written to disk at full resolution, one still every 120 ms at about 2.4-3.0 MB.
The on-device sessions from short test captures were 218 MB for 88 frames, so a 15-20 minute capture
wrote several GB and the phone warned about storage (7.2 GB free at the time of the report).

No later stage reads those pixels: the live map, `ReconstructionRunner`, the dense pass and
`KeyframeColorSampler` all open keyframe JPEGs, and `GrayDecoder` decodes them at 480 px or less.

## Change

- `SessionStoragePolicy`: keyframes only (a still is kept anyway after 40 without one, so a missing
  gyro cannot stop a scan), 1600 px on the long edge, JPEG quality 90, at most 150 frames per scan.
- `CameraCaptureController.persistStill` applies the policy on the frame writer thread; every still
  still goes through the keyframe annotator, so skipped frames leave the rotation reference alone.

## Measurement

Method: `adb shell am broadcast -n com.scand3d/.debug.CaptureTestReceiver -a com.scand3d.debug.CAPTURE
--es cmd start`, wait 60 s, `--es cmd stop`, then `run-as com.scand3d du -sm files/sessions/<stamp>`
and the JPEG SOF header of one stored frame.

| Metric | Before | After |
| --- | --- | --- |
| Frames stored per 60 s capture | 88 | 25 |
| Session size | 218 MB | 4 MB |
| Size per frame | 2.4-3.0 MB at 4032x3024 | 130-135 KB at 1600x1200 |
| Size per minute of capture | ~218 MB | ~4 MB |
| 20 minute capture (extrapolated) | ~4.3 GB | ~80 MB, hard cap 150 frames = ~20 MB |

The 150-frame cap is the same view count the memory budget in `agents.md` section 6 is written for,
and 1600 px is the long edge that section names for the MVS input.

## Test

`SessionStoragePolicyTest` (6 tests) covers the keep rule, the fallback cadence, the frame cap and
the downscale arithmetic.

## Caveats

- Nothing is deleted: earlier sessions stay on the device (371 MB on the test phone) until the user
  removes them, as `agents.md` section 8 requires.
- At the cap the capture loop stops requesting stills; the preview keeps running until the user
  stops the scan.
- Stored frames no longer carry the camera's EXIF orientation tag, because they are re-encoded from
  decoded pixels. The pixels keep the camera's own orientation (the pipeline never read the tag),
  so the reconstruction is unaffected, but a desktop viewer may show them sensor-oriented.
