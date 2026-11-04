package com.scand3d.sensors

import com.scand3d.capture.FrameRecord
import com.scand3d.core.bridge.OrientationMath
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ImuFrameAnnotatorTest {

    private class FakeMath(angles_rad: List<Double> = emptyList()) : OrientationMath {
        private val pending = ArrayDeque(angles_rad)
        var calls = 0

        override fun angularDisplacementRad(a: List<Double>, b: List<Double>): Double {
            calls += 1
            return if (pending.isEmpty()) 0.0 else pending.removeFirst()
        }
    }

    private fun frame(timestamp_ns: Long) = FrameRecord(
        timestamp_ns = timestamp_ns,
        exposure_ns = 30_000_000,
        iso = 100,
        focus_diopter = 5f,
        relative_path = "sessions/test/frames/frame_0001.jpg",
    )

    private val quaternion = listOf(1.0, 0.0, 0.0, 0.0)

    @Test
    fun `a frame without a sensor sample is stored unchanged`() {
        val annotator = ImuFrameAnnotator(snapshots = { null }, math = FakeMath())

        val annotated = annotator.annotate(frame(1_000_000_000))

        assertNull(annotated.orientation_wxyz)
        assertFalse(annotated.keyframe_candidate)
        assertEquals(0, annotator.keyframeCandidates())
    }

    @Test
    fun `the applied offset shifts the lookup and the residual is measured`() {
        val requested = mutableListOf<Long>()
        val annotator = ImuFrameAnnotator(
            snapshots = { timestamp_ns ->
                requested += timestamp_ns
                ImuOrientationSample(timestamp_ns - 7, quaternion)
            },
            math = FakeMath(),
            applied_offset_ns = 5_000_000,
        )

        val annotated = annotator.annotate(frame(1_000_000_000))

        assertEquals(listOf(1_005_000_000L), requested)
        assertEquals(quaternion, annotated.orientation_wxyz)
        assertEquals(1_004_999_993L, annotated.imu_timestamp_ns)
        assertEquals(4_999_993L, annotated.imu_time_offset_ns)
    }

    @Test
    fun `keyframes follow the baseline window and re-anchor after a jump`() {
        val math = FakeMath(listOf(0.05, 0.25, 0.05, 0.90, 0.25))
        val annotator = ImuFrameAnnotator(
            snapshots = { ImuOrientationSample(it, quaternion) },
            math = math,
        )

        assertTrue(annotator.annotate(frame(1)).keyframe_candidate) // first frame anchors
        assertFalse(annotator.annotate(frame(2)).keyframe_candidate) // below the window
        assertTrue(annotator.annotate(frame(3)).keyframe_candidate) // 0.25 rad accepted
        assertFalse(annotator.annotate(frame(4)).keyframe_candidate) // 0.05 rad, too close
        assertFalse(annotator.annotate(frame(5)).keyframe_candidate) // 0.90 rad, overlap lost
        assertTrue(annotator.annotate(frame(6)).keyframe_candidate) // 0.25 rad from the new anchor

        assertEquals(5, math.calls) // the anchoring frame needs no rotation
        assertEquals(3, annotator.keyframeCandidates())
    }

    @Test
    fun `a non-finite rotation is rejected and keeps the reference`() {
        val math = FakeMath(listOf(Double.NaN, 0.25))
        val annotator = ImuFrameAnnotator(
            snapshots = { ImuOrientationSample(it, quaternion) },
            math = math,
        )
        annotator.annotate(frame(1))

        assertFalse(annotator.annotate(frame(2)).keyframe_candidate)
        // Still compared against the anchor from frame 1, so this lands inside the window.
        assertTrue(annotator.annotate(frame(3)).keyframe_candidate)
    }

    @Test
    fun `metadata reports the measured offset and the sensor mode`() {
        val availability = ImuAvailability(
            has_gyroscope = true,
            has_accelerometer = true,
            has_magnetometer = true,
        )
        val annotator = ImuFrameAnnotator(
            snapshots = { timestamp_ns -> ImuOrientationSample(timestamp_ns - 2_000_000, quaternion) },
            math = FakeMath(),
            availability = availability,
        )
        annotator.annotate(frame(1_000_000_000))
        annotator.annotate(frame(2_000_000_000))

        val metadata = annotator.metadata("realtime")

        assertEquals("realtime", metadata.camera_clock_source)
        assertEquals("elapsed_realtime_ns", metadata.sensor_clock)
        assertEquals(-2_000_000L, metadata.clock_offset_ns)
        assertEquals(0L, metadata.clock_offset_mad_ns)
        assertEquals(2, metadata.clock_samples)
        assertEquals("NineAxis", metadata.imu_mode)
        assertFalse(metadata.imu_reduced_accuracy)
        assertEquals(1, metadata.keyframe_candidates) // the second frame is 0 rad away
    }
}
