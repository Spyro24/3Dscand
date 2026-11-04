# Phase 6 dense reconstruction verification

**Status: synthetic acceptance met on the host; no phone measurement and no app wiring.** Every
number below comes from the synthetic turntable scan on a 16-core desktop. Nothing here is a
measurement on the SM-G930F, no real object capture has been through the dense stage, and the cloud
is not yet reachable from the app (the bridge was not extended, so no public interface changed).

## What ran

| Check | Command | Result |
|---|---|---|
| C++ core, host | `./gradlew nativeTest` | 91 tests passed, 9 of them new for Phase 6 |
| Kotlin, host JVM | `./gradlew :app:testDebugUnitTest` | 49 tests passed (unchanged) |
| APK build | `./gradlew :app:assembleDebug` | green |
| Dense report | `./build/native-tests/mvs_report 16 320 1.5` | exit 0, prints the tables below |

`JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64` was set for all of them (D004). The host test build
now defaults to `RelWithDebInfo` (D047): the earlier unoptimised build made the timing numbers here
meaningless.

The scene is `synthetic::makeNoiseTexturedBoxScene`: a 0.70 x 0.50 x 0.50 m box shaded with
continuous value noise, seen by a turntable ring of radius 1.5 m at height 0.60 m, rendered at
480 x 360 with `focal_px` 360. The depth search runs over 0.6-2.0 m, sparse hints come from the
mesh vertices as they would from a Phase 4 map, and fusion uses 4 mm voxels. The report tool and the
gtest pipeline test build the scene from the same code, so a number in this file is reproducible
with one command.

## Measured numbers

16 views, 22.5 degrees apart:

| Preset | Working width | Depth pixels | After consistency | Fused points | Chamfer accuracy | Chamfer completeness | Runtime |
|---|---|---|---|---|---|---|---|
| Draft | 160 px | 51 087 | 36 922 | 35 027 | **6.0 mm** | **9.2 mm** | 10.4 s |
| Normal | 320 px | 192 902 | 128 783 | 91 158 | **4.7 mm** | **6.7 mm** | 42.4 s |
| High | 480 px | 424 296 | 269 000 | 115 143 | **5.9 mm** | **5.8 mm** | 105.8 s |

Completeness is scored over the 1.200 m2 of box surface a ring at the object centre can see - the
four sides. The top and the underside are never observed by any camera in this capture, so counting
them would measure the capture geometry instead of the reconstruction (D048). Accuracy is measured
over every fused point.

The acceptance line is "Chamfer distance to synthetic ground truth below the threshold in
`docs/benchmarks/`". That threshold is **10 mm accuracy and 15 mm completeness at Draft**
(16 views), and all three presets stay under it: the worst accuracy number is 6.0 mm and the worst
completeness number is 9.2 mm. Normal quality is the best trade: it halves the error of Draft for
four times the runtime, while High adds points without improving either distance.

### 100 images at reduced resolution

The performance budget in agents.md is "a few minutes" for 100 images at reduced resolution:

| Views | Preset | Depth pixels | After consistency | Fused points | Chamfer accuracy | Chamfer completeness | Runtime |
|---|---|---|---|---|---|---|---|
| 100 | Draft (160 px) | 338 707 | 259 382 | 202 674 | 8.8 mm | 3.9 mm | **75.9 s** |

100 images finish in 76 s, inside the budget with room to spare. Accuracy is worse than the 16-view
Draft run (8.8 mm against 6.0 mm) because a denser ring gives shorter baselines: each view is still
matched only against its nearest neighbour, and with 3.6 degrees between neighbours the parallax per
pair is small. Completeness improves to 3.9 mm because 100 views cover the surface far more
thickly.

### View-count sweep (Normal, 320 px)

| Views | Degrees per view | Depth pixels | Fused points | Accuracy | Completeness | Runtime |
|---|---|---|---|---|---|---|
| 8 | 45.0 | 86 801 | 42 812 | 4.4 mm | 31.2 mm | 25.6 s |
| 16 | 22.5 | 192 902 | 91 158 | 4.7 mm | 6.7 mm | 42.4 s |
| 100 | 3.6 | (Draft, see above) | - | - | - | - |

Eight views are enough to place points on the ground truth but not to cover it: completeness is
31.2 mm because 45 degrees between neighbours leaves the surfaces that are only seen at grazing
angles unmatched. Sixteen views is the first count where coverage is no longer the limiting error,
and that is what the tests use.

## What the tests cover

Nine gtest cases in `tests/native/mvs_test.cpp`:

- `DepthMapTest.ASizedMapHasNoValidPixel` - the depth map contract (sizes, no valid pixel).
- `PatchMatchTest.RecoversTheSurfaceOfTheTurntableBox` - one stereo pair on the box: at least 5
  percent of pixels get a depth and the mean distance to the mesh stays under 20 mm.
- `PatchMatchTest.AFlatImageYieldsNoDepth` - a textureless image yields zero valid pixels.
- `ConsistencyTest.KeepsThePlaneBothViewsAgreeOnAndDropsTheRest` - a corrupted half of a depth map
  is rejected by the forward-backward check.
- `FusionTest.AveragesTwoDepthMapsIntoOnePlane` - two observations of one plane fuse to one layer
  with the mean gray, and `min_observations` filtering works.
- `MvsPipelineTest.FusesTheTurntableIntoAPointCloud` - 16 views, Draft: accuracy under 20 mm and
  completeness under 35 mm over the four sides.
- `MvsPipelineTest.CancellingStopsBeforeFusionAndReportsProgress` - a cancelled run reports
  progress, stops early and publishes no partial cloud.
- `MvsPipelineTest.HigherQualityEstimatesMorePixels` - the presets produce the requested working
  widths (160 and 240 px here) and a higher preset finds more than twice the depth pixels.
- `MvsPipelineTest.SparseHintsNarrowTheDepthRange` - a 0.05-20 m search narrowed by sparse hints
  still fuses a cloud on the object.

`mvs_test` runs in 63 s; the whole `nativeTest` target takes 236 s, most of it the older
synthetic scenes.

## Not measured or not implemented

- **Phone timing.** The numbers above are from a desktop with an 8-worker pool (`std::thread` sizes
  the pool from `hardware_concurrency`, capped at 8). The SM-G930F has four big and four little
  cores, so the wall clock there will be several times longer. The budget comparison above is
  therefore a host number, not a device guarantee.
- **A real capture.** Only the synthetic box has been through the dense stage. Real JPEG keyframes
  are decoded and posed by Phase 4, but the dense stage has not been pointed at them.
- **App wiring.** `mvs::reconstructDense` is core-only: no JNI entry point and no UI path exists
  yet, so the app still draws the sparse map. Wiring it is a public-interface change and was not
  made (agents.md rule 5).
- **Colour.** Fusion stores one gray value per point; the core only ever sees gray images. Colour
  would come from the app-side keyframe sampler that already colours the overlay (D033), not from
  the core.
- **Occlusion and background.** Every pixel with a good correlation keeps a depth, including the
  background plane behind the object. Masks (Phase 7) are what remove it; the box scene has no
  background geometry behind the object that would hide this.
