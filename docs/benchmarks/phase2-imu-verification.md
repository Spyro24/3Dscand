# Phase 2 IMU verification

**Status: host verification plus one device smoke run on the SM-G930F. Two acceptance items are
still unmeasured** — orientation error against a reference rig and the no-gyroscope fallback path.
The physical 90 degree table procedure that would have produced a reference number was declined
during this pass, so no accuracy figure exists for Phase 2.

## What ran

| Check | Command | Result |
|---|---|---|
| C++ core, host | `./gradlew nativeTest` | 48 tests pass (29 at the end of Phase 2) |
| Kotlin, host JVM | `./gradlew :app:testDebugUnitTest` | 37 tests pass, 36 of them new for Phase 2 |
| APK builds | `./gradlew :app:assembleDebug` | BUILD SUCCESSFUL |
| Device smoke run | `adb install -r app-debug.apk`, debug capture receiver | SM-G930F, API 34, arm64-v8a |

`JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64` was set for all builds (D004).

## Device run (SM-G930F, 2026-11-04)

| Check | Measurement | Verdict |
|---|---|---|
| Sensors present | `dumpsys sensorservice`: K6DS3TR accelerometer and gyroscope, Samsung rotation vector; app logs `imu mode=NineAxis reduced_accuracy=false` | All four sources available |
| Sensor rates | HAL event timestamps 10.0 ms apart (accelerometer) and 5.03 ms apart (gyroscope) | ~100 Hz and ~199 Hz, matching D018 |
| Schema 2 manifest | Session `20261104-151510`: `schema` 2, 17 of 17 frames carry `orientation_wxyz`, `imu_timestamp_ns` and `imu_time_offset_ns`, `clock` and `imu` blocks present | Pass after the fix below |
| Clock offset | `clock.offset_ns` = -373.5 us, -699.5 us, -431.2 us over three sessions; MAD 0.93-1.20 ms | Offset measured per session, spread below its own noise floor |
| Per-device profile | `files/devices/SM-G930F.profile` holds `offset_ns=-431156`, `offset_mad_ns=1182888`, `samples=16`, `imu_mode=NineAxis` | Persisted across sessions |
| Preview frame rate | Benchmark JSONL: median interval 36.9 ms, range 33.3-38.3 ms | 27.1 fps during capture, same as the Phase 1 baseline |
| Still capture rate | 17 frames in about 11 s | ~1.6 fps; full-resolution JPEG plus serialisation, not a preview problem |
| Thermal | All entries `NONE` during the run | No throttling observed in a 30 s session |

### Defect found by the device run

The last session of a capture run was losing its `clock` and `imu` blocks. A still that was
in-flight when capture stopped lands after the stop flush, and that code path rewrote the manifest
with `writeManifest(lockedSettings, model)` — no session metadata. Session `20261104-151236` ended
with 13 frames and no clock/imu block; after routing the rewrite through `flushManifest`, session
`20261104-151510` keeps both with 17 frames. Fixed in `1521eaf`.

### What the device run cannot say

- **Orientation error against a reference rig is not measured.** The planned stand-in was a 90
  degree table turn with the phone edge aligned to a table edge; that procedure was declined. A
  10 s static hold in one session changed by 4.63 degrees start to end, but that is motion or
  drift, not a reference comparison, and it is not reported as accuracy.
- **The offset is a measurement with an uncertainty.** The profile stores the value written at
  stop, so it can differ slightly from the manifest's final value when a late frame lands after
  the stop flush (the session above shows 16 samples at stop, 17 in the manifest).
- **The no-gyroscope fallback was not exercised on hardware.** The SM-G930F has a gyroscope and
  D010 rules out a second device; the fallback is covered only by host tests and the bridge test.
- **Sensor rates were read from the HAL's event log, not from the app's own sample counter.** The
  app does not log its delivery rate, so a dropped sample inside the app would not show up here.

## Host evidence (unchanged from the earlier pass)

| Task | Evidence | Limit |
|---|---|---|
| 2.1 sensors | `ImuAvailabilityTest` maps hardware combinations to fusion modes; `AndroidImuSource` compiles against `SensorManager` | Rates now measured at the HAL (above) |
| 2.2 filter | `MahonyFilter.IntegratesNinetyDegreesOfYaw...` within 0.01 rad of a synthetic 90 degree turn; `StaysAtRestWhenLevel`; `ConvergesToTheMagnetometerHeading` reaches cosine > 0.995 after 20 s at 200 Hz | Synthetic vectors, no sensor noise or bias model |
| 2.3 clock sync | `TimestampSyncTest` covers median/MAD against outliers, even windows, eviction and tolerance; `ImuFrameAnnotatorTest` proves the offset shifts the lookup and the residual is recorded per frame | Offset now measured on device |
| 2.4 blur monitor | `AngularRateMonitorTest` covers the exposure-scaled smear calculation and the 1.5/1.0 degree hysteresis | Thresholds still unvalidated against real motion |
| 2.5 fallback | `TiltOrientation` recovers gravity, ignores a magnetic vector parallel to gravity, uses north when available and fails in free fall; `BridgeImu.RunsTheFallbackWithoutAGyroscope` | Never exercised on hardware without a gyroscope |
| 2.6 keyframe prior | `KeyframePrior` accepts only the 0.10-0.70 rad window; annotator tests cover acceptance, rejection below the window and re-anchoring after overshoot | Device sessions show 1 candidate while still and 14-17 while the phone was being handled, which is consistent but not a tuned measurement |

`ImuFrameAnnotatorTest` supplies its own rotation math because the host JVM cannot load the
arm64-v8a library (D011). The bridge math is covered from the C++ side instead.

## Still outstanding

1. **Orientation accuracy.** Needs either a reference rig or an accepted substitute (for example
   holding known angles with a digital level as the reference). Until then the 2 degree line is
   unmeasured and must not be quoted.
2. **Fallback on hardware.** Needs a device without a gyroscope, or a debug-only sensor-injection
   path in the D016/D017 style that removes the gyroscope from the fusion input.
3. **Optional:** log the app-side sensor delivery rate so HAL rates and delivered rates can be
   compared, and store both the at-stop and final clock sample counts.

## Consequences

Roadmap 2.1-2.6 stay `[~]`; they are no longer "code only" — the sensor set, rates, schema-2
manifests, clock offset, device profile and preview rate are measured on the SM-G930F — but the
reference accuracy measurement and the fallback path are still open (D021, D027). The M1 milestone
is not claimable.
