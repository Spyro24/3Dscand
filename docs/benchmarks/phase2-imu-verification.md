# Phase 2 IMU verification

**Status: host verification only. The hardware acceptance has not been run** — no device was
available for this pass, which is recorded as D021. Nothing below should be read as a measurement
on a phone.

## What ran

| Check | Command | Result |
|---|---|---|
| C++ core, host | `./gradlew nativeTest` | 29 tests passed, 18 of them new for Phase 2 |
| Kotlin, host JVM | `./gradlew :app:testDebugUnitTest` | 37 tests passed, 36 of them new for Phase 2 |
| APK builds with the new native code | `./gradlew :app:assembleDebug` | BUILD SUCCESSFUL |

`JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64` was set for all three (D004).

## What the host tests actually establish

| Task | Evidence | Limit |
|---|---|---|
| 2.1 sensors | `ImuAvailabilityTest` maps hardware combinations to fusion modes; `AndroidImuSource` compiles against `SensorManager` on the device build | No real rate, timestamp or accuracy was observed |
| 2.2 filter | `MahonyFilter.IntegratesNinetyDegreesOfYaw...` within 0.01 rad (~0.57 deg) of a synthetic 90 deg turn; `StaysAtRestWhenLevel`; `ConvergesToTheMagnetometerHeading` reaches cosine > 0.995 against the measured field after 20 s of simulated 200 Hz samples | Synthetic vectors, no sensor noise, bias or magnetic disturbance |
| 2.3 clock sync | `TimestampSyncTest` covers median/MAD against outliers, even windows, eviction and tolerance; `ImuFrameAnnotatorTest` proves the offset shifts the lookup, the residual is recorded per frame and `metadata()` reports it | No camera and no sensor ever ran, so no real offset exists |
| 2.4 blur monitor | `AngularRateMonitorTest` covers the exposure-scaled smear calculation, the 1.5/1.0 deg hysteresis and the no-gyro state | Thresholds are unvalidated against real motion |
| 2.5 fallback | `TiltOrientation` recovers gravity to 1e-9, ignores a magnetic vector parallel to gravity, uses north when available and fails in free fall; `BridgeImu.RunsTheFallbackWithoutAGyroscope` runs the path through the handle API | Never exercised on a phone without a gyroscope |
| 2.6 keyframe prior | `KeyframePrior` accepts only the 0.10-0.70 rad window; `ImuFrameAnnotatorTest` proves acceptance, rejection below the window, and re-anchoring after an overshoot | Window is a starting guess for Phase 4, not a tuned value |

`ImuFrameAnnotatorTest` supplies its own rotation math because the host JVM cannot load the
arm64-v8a library (D011); the same test therefore does not cover the bridge call it wraps. The
bridge math is covered from the C++ side instead.

## Still outstanding (device run)

1. Install the debug APK and confirm `AndroidImuSource` logs all four sensors present, with the
   gyro delivering roughly 200 Hz at `SENSOR_DELAY` 5 ms.
2. Capture a 20-frame session and pull `session.json`: `schema` must be 2, every frame must carry
   `orientation_wxyz`, `imu_timestamp_ns`, `imu_time_offset_ns`, and `clock`/`imu` must be present.
3. Compare the recorded `clock.offset_ns` and `clock.offset_mad_ns` across two sessions; the
   profile at `files/devices/<model>.profile` must hold the first session's median.
4. Mount the phone on the reference rig, step it by known angles and compare integrated
   orientation: the roadmap requires under 2 degrees of error.
5. Exercise the fallback path. The SM-G930F has a gyroscope and D010 rules out a second device, so
   this needs either an explicit sensor-injection path in the debug build (the D016/D017
   precedent) or a recorded limitation instead of a claim.
6. Watch preview frame rate with the sensor thread running; Phase 1 held 30 fps idle and 27.1 fps
   during a burst.

## Consequences

Roadmap 2.1-2.6 stay `[~]`. The M1 milestone (Phases 0-2) is not claimable until items 1-6 are
recorded here.
