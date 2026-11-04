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

## Low-light input (D052)

Measured on the same phone, same room, 4 November 2026.

Method: start a 20 s capture, then pull the stored keyframes and take the mean of the luma channel
with PIL; the exposure lock is the one the app logged.

| Metric | Locked preview exposure | Stills with D052 |
| --- | --- | --- |
| Exposure | 30 ms | 100 ms (cap) |
| ISO | 1600 | 1600 |
| Mean luma | 13-21/255 | 33-40/255 |
| FAST corners at 480 px | 0 | 160-250 per frame |

The still is still about five times too dark for a comfortable map, and the capture screen now says
"Too dark: add light" instead of silently storing frames no stage can use.

## Why the overlay still does not line up

`tracking_report` replays a saved session through the map and then through `sfm::Tracker` on the
host. Replaying the brightest real session (`20261104-205309`, 26 keyframes) with a growing number
of map views:

| Map views | Registered | Points | Live frames tracked |
| --- | --- | --- | --- |
| 4 | 3 | 23 | 0 of 22 |
| 8 | 3 | 101 | 2 of 18 |
| 12 | 4 | 101 | 0 of 14 |
| 24 | 4 | 101 | 0 of 2 |

Feeding more views does not grow the map: registration stops after four views, so raising
`MAX_RECONSTRUCTION_FRAMES` would only cost time. A dark session registers 0 views and has no map
at all, which is what an empty overlay looks like from the outside.

## Caveats

- Nothing is deleted: earlier sessions stay on the device (371 MB on the test phone) until the user
  removes them, as `agents.md` section 8 requires.
- At the cap the capture loop stops requesting stills; the preview keeps running until the user
  stops the scan.
- Stored frames no longer carry the camera's EXIF orientation tag, because they are re-encoded from
  decoded pixels. The pixels keep the camera's own orientation (the pipeline never read the tag),
  so the reconstruction is unaffected, but a desktop viewer may show them sensor-oriented.
