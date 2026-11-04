package com.scand3d.sensors

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class FusedImuTest {

    private class FakeSource(override val availability: ImuAvailability) : SensorSource {
        var listener: ((ImuSample) -> Unit)? = null

        override fun start(listener: (ImuSample) -> Unit) {
            this.listener = listener
        }

        override fun stop() {
            listener = null
        }

        fun emit(sample: ImuSample) {
            listener?.invoke(sample)
        }
    }

    private class FakeFilter(private val quaternion: List<Double>? = listOf(1.0, 0.0, 0.0, 0.0)) :
        OrientationFilter {
        var pushed = 0
        var reset = false
        var closed = false

        override val isAvailable: Boolean = true

        override fun push(sample: ImuSample): List<Double>? {
            pushed += 1
            return quaternion
        }

        override fun reset() {
            reset = true
        }

        override fun close() {
            closed = true
        }
    }

    private fun sample(timestamp_ns: Long) = ImuSample(
        timestamp_ns = timestamp_ns,
        gyro_rad_s = Vec3(0.1, 0.0, 0.0),
        accel_m_s2 = Vec3(0.0, 0.0, 9.81),
        has_gyro = true,
        has_accel = true,
    )

    @Test
    fun `fused orientations are stored and paired by timestamp`() {
        val source = FakeSource(ImuAvailability(has_gyroscope = true, has_accelerometer = true))
        val filter = FakeFilter()
        val seen = mutableListOf<Long>()
        val imu = FusedImu(source, filter, ImuRingBuffer(), onSample = { seen += it.timestamp_ns })

        imu.start()
        source.emit(sample(1_000))
        source.emit(sample(2_000))

        assertEquals(listOf(1_000L, 2_000L), seen)
        assertEquals(2, filter.pushed)
        assertEquals(1_000L, imu.snapshotAt(1_100)?.timestamp_ns)
        assertEquals(2_000L, imu.snapshotAt(1_900)?.timestamp_ns)
    }

    @Test
    fun `a filter without an estimate adds nothing`() {
        val source = FakeSource(ImuAvailability(has_gyroscope = true, has_accelerometer = true))
        val imu = FusedImu(source, FakeFilter(quaternion = null))

        imu.start()
        source.emit(sample(1_000))

        assertNull(imu.latest())
        assertEquals(0, imu.snapshotAt(1_000).let { if (it == null) 0 else 1 })
    }

    @Test
    fun `close stops the source and releases the filter`() {
        val source = FakeSource(ImuAvailability())
        val filter = FakeFilter()
        val imu = FusedImu(source, filter)

        imu.start()
        imu.close()

        assertNull(source.listener)
        assertTrue(filter.closed)
    }

    @Test
    fun `reset clears history and the filter state`() {
        val source = FakeSource(ImuAvailability(has_gyroscope = true, has_accelerometer = true))
        val filter = FakeFilter()
        val imu = FusedImu(source, filter)

        imu.start()
        source.emit(sample(1_000))
        imu.reset()

        assertNull(imu.latest())
        assertTrue(filter.reset)
        assertFalse(filter.closed)
    }
}
