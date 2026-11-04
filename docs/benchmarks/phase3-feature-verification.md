# Phase 3 feature and matching verification

**Status: host verification only. The real-object acceptance has not been run** — no device and no
real image pair were available for this pass, which is recorded as D026. Nothing below is a
measurement on a phone or on a real photograph.

## What ran

| Check | Command | Result |
|---|---|---|
| C++ core, host | `./gradlew nativeTest` | 48 tests passed, 19 of them new for Phase 3 |
| Kotlin, host JVM | `./gradlew :app:testDebugUnitTest` | 37 tests passed (unchanged) |
| Desktop tool binary | `./build/native-tests/match_visualizer a.pgm b.pgm out.ppm` | exit 0, writes a `P6` overlay |

`JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64` was set for all three (D004).

The synthetic pair throughout is `makeTexturedBoxScene(12, 1.5, 0.45, 640, 480, 500.0, 12, seed)`:
a 0.7 x 0.5 x 0.5 m box with a 12 x 12 speckled grid per face, observed from two cameras of the
turntable ring (30 degrees apart), 640 x 480, `focal_px` 500. Ground truth comes from
`visibleSurfaceVertices`, which projects every visible mesh vertex and gives it a position-derived
id that is identical in both views.

## Measured numbers

| Check | Measurement | Limit |
|---|---|---|
| 3.1 detector | 101+ keypoints on the synthetic pair; all inside the image bounds, scale in {1, 2, 4} | Flat albedo per cell, so the image is easier than a real object |
| 3.2 descriptor + storage | Roundtrip of every keypoint and all 32 descriptor bytes; a truncated file and a wrong magic are both rejected | Format checked on this build only |
| 3.3 matching | Ratio test (0.8) yields 67 matches on the 30-degree pair; 26 of them land within 3 px of a known corner and 20 of those (76.9%) are the same 3D point | Raw descriptor precision; RANSAC is the filter below |
| 3.3 approximate matcher | Recall against the exhaustive matcher passes the 0.85 bound; identical output on the crafted ambiguous-descriptor cases | Multi-index hashing falls back to exhaustive for tiny or huge buckets |
| 3.4 fund. matrix | 35% forced outliers, sigma 0.35 px: all true inliers found (>= 95% recall), at most 2 false positives, median Sampson < 0.5 px, matrix within 0.15 of ground truth up to scale | Same synthetic noise model, not real detector noise |
| 3.4 essential matrix | Recovers E with equal singular values (1e-6) and the same inlier quality | No cheirality/triangulation yet (Phase 4) |
| 3.5 end to end | 67 descriptor matches -> 55 RANSAC inliers at a 3 px threshold -> 23 mappable, **22 correct = 95.7%** | Meets the synthetic acceptance line |
| resolution change | Small view matched against the full-resolution pyramid: 58 of 73 bridged matches consistent within 3 px (79.5%) | Not scale invariance; see D022 |

The roadmap asks for "at least 95 percent of inlier matches are correct" on synthetic pairs with
known ground truth. The end-to-end row above is that measurement, and the gtest
`FeaturePipeline.MatchedInliersAgreeWithKnown3DPoints` asserts it.

## What the host tests actually establish

| Task | Evidence | Limit |
|---|---|---|
| 3.1 detector | `FeatureDetector.FindsManyCornersInsideTheImage`, `FeaturePyramid.BuildsLevelsThatHalveTheResolution`, `FeatureDetector.MatchingSurvivesAChangeOfResolution` | FAST threshold 20 and the 2 px dedup radius are untested against real texture |
| 3.2 descriptors | `FeatureStore.RoundTripsKeypointsAndDescriptors`, `FeatureStore.RejectsATruncatedFile`; the descriptor itself is exercised through matching | No descriptor-level unit test for rotation, because the descriptor is deliberately unsteered (D022) |
| 3.3 matching | `FeatureMatching.RatioTestMatchesAgreeWithKnown3DPoints`, `FeatureMatching.ApproximateMatchesAgreeWithExhaustive`, `FeatureMatching.RatioTestRejectsAmbiguousDescriptors` | Only 67 matches on the pair, so the percentages are coarse |
| 3.4 geometry | `TwoViewGeometry.*`: three-view-free two-camera lattice, 64 points, noise and outliers, ground-truth F/E built from the camera poses; `LinearAlgebra.*` checks the in-house SVD and inverse | The noise model is Gaussian in pixels, not detector localisation error |
| 3.5 tool | `MatchOverlay.RendersBothImagesSideBySide`, `MatchOverlay.WritesAPpm`, `PgmIo.RoundTripsAGrayscaleImage`; the CLI was run on an 8 x 6 PGM pair and produced a 16 x 6 `P6` overlay | No real image pair was ever rendered |

## Findings worth carrying forward

- Steering the descriptor with the intensity-centroid angle made matching **worse** on this object:
  verified inlier precision fell to 78% (18 of 23). The angle is still computed and stored as
  keypoint metadata, but the descriptor is sampled unsteered (D022). Rotation invariance is
  therefore not claimed.
- The first RANSAC implementation cut itself down to a single iteration: the adaptive-stop estimate
  is astronomically large for a weak inlier ratio, and casting it to `int` overflowed. The estimate
  is now compared as a `double` and only ever shortens the budget (D025).
- One refit after RANSAC was not enough. Alternating refit and reclassification until the inlier set
  settles moved the median Sampson distance from 0.91 px to below 0.5 px.
- Ground-truth ids in `visibleSurfaceVertices` are position-derived, not discovery-ordered. An
  earlier version numbered points per view, so the same corner carried different ids in the two
  images and every match looked wrong.

## Still outstanding (real images)

1. Capture a textured object twice from roughly 20-30 degrees apart with the app (or any two real
   JPEGs of the same object) and convert them to PGM.
2. Run `match_visualizer` on the pair and inspect the overlay: matches must follow the object, not
   the background.
3. Feed the pair through the matcher and RANSAC and record the verified inlier count. The roadmap
   requires **at least 100**. This pass has no way to verify them: the ground truth for a real pair
   is the visual overlay plus a manual check.
4. Record the same numbers for a pair with a larger baseline and for a low-texture object, so the
   detector thresholds can be tuned against data instead of guesses.
5. Once Phase 4 exists, re-run the synthetic pair end to end through the app and confirm the same
   inlier precision on device.
