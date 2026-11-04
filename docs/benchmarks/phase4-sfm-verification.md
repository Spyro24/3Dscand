# Phase 4 reconstruction verification

**Status: synthetic acceptance met, real-scan acceptance not run.** Every number below comes from
the synthetic turntable scan on the host. No real object capture has been fed through the pipeline,
so the roadmap's "a real object scan registers at least 90 percent of keyframes" is unmeasured
(D030). Nothing here is a measurement on a phone.

## What ran

| Check | Command | Result |
|---|---|---|
| C++ core, host | `./gradlew nativeTest` | 57 tests passed, 9 of them new for Phase 4 |
| Kotlin, host JVM | `./gradlew :app:testDebugUnitTest` | 37 tests passed (unchanged) |
| APK build | `./gradlew :app:assembleDebug` | green |
| Reconstruction report | `./build/native-tests/sfm_report 16 0.5` | exit 0, prints the table below |

`JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64` was set for all of them (D004).

The scan is `tests/synthetic/sfm_scan.cpp`: a 4 x 4 x 4 lattice of 64 known points spanning
0.6 x 0.48 x 0.48 m at 0.36-0.84 m depth, seen by a turntable ring of radius 1.6 m at height 0.5 m,
640 x 480, `focal_px` 500. Every point is visible in every view, pixels carry Gaussian noise, and
matches connect neighbouring views. The report tool and the gtest pipeline test build the scan from
the same code, so a number in this file is reproducible with one command.

## Measured numbers

16 views, 22.5 degrees apart, sigma 0.5 px:

| Quantity | Measured | Roadmap limit |
|---|---|---|
| Registered views | 16 of 16 | - |
| Points / observations | 64 / 1024 | - |
| Mean reprojection error | **0.588 px** | below 1 px |
| Median / max reprojection | 0.539 px / 1.799 px | - |
| Worst pose rotation error | **0.261 deg** | below 1 deg |
| Worst camera-centre direction error | **0.582 deg** | - |
| Runtime | 1368 ms | - |

Both halves of the synthetic acceptance line are met with margin: the mean reprojection error is
59 percent of the 1 px bound and the worst pose error is 26 percent of the 1 degree bound. Pose
errors are gauge-free: a monocular reconstruction fixes the cameras only up to a similarity, so each
view is compared with the seed view by relative rotation and by the *direction* of its baseline.

### View-count sweep (sigma 0.5 px)

| Views | Degrees per view | Registered | Mean / median / max px | Worst rotation | Worst centre direction | Runtime |
|---|---|---|---|---|---|---|
| 8 | 45.0 | 2 of 8 | 1.121 / 1.040 / 2.529 | 1.828 deg | 2.365 deg | 0.5 s |
| 12 | 30.0 | 12 of 12 | 0.569 / 0.542 / 1.591 | 0.213 deg | 0.302 deg | 1.3 s |
| 16 | 22.5 | 16 of 16 | 0.588 / 0.539 / 1.799 | 0.261 deg | 0.582 deg | 1.4 s |
| 24 | 15.0 | 24 of 24 | 0.599 / 0.556 / 1.662 | 0.165 deg | 0.399 deg | 5.9 s |
| 30 | 12.0 | 30 of 30 | 0.609 / 0.575 / 1.905 | 0.222 deg | 0.563 deg | 4.3 s |
| 40 | 9.0 | 40 of 40 | 0.611 / 0.579 / 1.891 | 0.212 deg | 0.497 deg | 30.1 s |

The 8-view row is the keyframe policy working, not a reconstruction failure: 45 degrees between
views is outside the 0.10-0.70 rad window, so the ten candidate views are all rejected with the
rotation reason and only the two seed views remain.

### Noise stress

16 views at sigma 1.0 px, twice the nominal detector localisation error: 16 of 16 registered, mean
1.171 px, median 1.063 px, max 3.485 px, worst rotation 0.391 deg, worst centre direction 0.634 deg,
18.8 s. The reconstruction stays correct; only the reprojection error crosses the 1 px line, as it
must when the input noise is above it.

## What the host tests establish

| Task | Evidence |
|---|---|
| 4.1 two-view initialisation | `TwoViewInitialization.RecoversTheCalibratedRelativePose`: 64 matches at sigma 0.4 px recover the relative rotation within 1 degree, the unit-baseline direction within 1 degree, median reprojection below 1 px |
| 4.1 triangulation | `Triangulation.RecoversAKnownPoint` (exact to 1e-6 m, parallax above 0.1 rad), `Triangulation.RejectsAZeroBaselinePair` |
| 4.2 PnP with RANSAC | `PnP.RecoversAPoseWithOutliers`: 20 percent forced outliers, rotation within 0.5 deg, centre direction within 1 deg, median below 1 px |
| 4.3 keyframe selection | `KeyframeSelection.AppliesTheRotationAndOverlapWindow`: too-small and too-large rotation, too-small baseline and too-low overlap are each rejected; a valid candidate is accepted; `rotationAngleRad` checked against a 90 degree turn |
| 4.4 bundle adjustment | `BundleAdjustment.ReducesTheCostOfANoisyReconstruction`: noisy triangulations, cost strictly drops, mean reprojection ends below 1 px |
| 4.5 map store | `MapStore.DropsOutlierObservationsAndMergesDuplicates`: a 60 px observation is dropped from a five-observation track, the track keeps its other four and stays indexed; two points 5 mm apart that share a view merge into one |
| 4.6 report + acceptance | `Pipeline.ReconstructsTheSyntheticTurntableScan` (>= 15 of 16 registered, mean below 1 px, worst rotation and centre direction below 1 degree), `Report.SummarizesWhatWasEstimated` (observations and registered count agree with the reconstruction) |

## Findings worth carrying forward

- **Tracks carried no pixels.** `TrackBuilder` groups `(view, feature)` keys, so its observations
  were built with a zero pixel and every map triangulation ran on `(0, 0)`. Only the pixel table
  handed to `MapStore::setFeaturePixels` makes a track triangulable; one point survived by accident.
- **The parallax check compared two different frames.** Rays were taken in each camera's own
  coordinates, which cancels exactly the rotation that produces the parallax, so a 20 degree pair
  measured 0.02 rad and every badly conditioned point passed.
- **PnP's scale solution is sign ambiguous, and `det < 0` must be fixed by negating the whole
  matrix.** Negating one column instead - the obvious "make it a rotation" reflex - turns the camera
  180 degrees about that axis. All projections then fell behind the camera and PnP returned 1e9 px
  residuals on its own inliers.
- **Gauss-Newton steps had the wrong sign in PnP and in bundle adjustment.** The system is
  `H d = J^T r`, so `d` is the ascent direction; both applied it as a descent step while the
  triangulation refinement subtracted it. BA reduced nothing until the sign was fixed, and the pose
  Jacobian also had `-[Xc]x` where the right-multiplied update gives `+[Xc]x`.
- **A mirrored seed fits its own pair perfectly.** Both decompositions of the essential matrix
  reproduce the two input images, so neither cheirality nor reprojection error can choose between
  them. The pipeline now seeds a pair, tries to register a third view, and moves to the next pair
  (up to eight) when nothing grows. Before that, the same scan reconstructed correctly at 16 views
  and locked onto a mirror at 12 and 24.
- **Skipping a failed view permanently cost a keyframe.** At 30 views one view failed PnP while the
  map was still two views wide and was never retried. Views are now re-evaluated every pass, and a
  pass that registers nothing ends the loop.
- **Bundle adjustment on two cameras is degenerate.** It can trade the whole reconstruction for a
  mirror at the same reprojection error, which is why `finalize` only runs it from three views up.

## Still outstanding (real object scan)

1. Capture a real object with the app, run the frames through feature detection, matching and this
   pipeline, and record the registered fraction. The roadmap requires **at least 90 percent**; this
   pass has no real capture at all, and the app-side path from captured frames to matches does not
   exist yet, so there is nothing to point at a real object.
2. Record the same numbers for a low-texture object and for a hand-held (non-turntable) arc, where
   the keyframe policy and PnP have to cope with motion blur and rolling shutter.
3. Measure runtime on the device. 30 s for 40 views is a host number with an unoptimised O(N) seed
   search and a full BA after every registration; neither has been profiled.
