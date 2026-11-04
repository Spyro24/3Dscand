package com.scand3d.sensors

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class ImuRingBufferTest {

    private fun sample(timestamp_ns: Long) =
        ImuOrientationSample(timestamp_ns, listOf(1.0, 0.0, 0.0, 0.0))

    @Test
    fun `empty buffer has no nearest sample`() {
        assertNull(ImuRingBuffer().nearestTo(1_000))
    }

    @Test
    fun `returns the sample closest to the requested timestamp`() {
        val buffer = ImuRingBuffer()
        buffer.add(sample(1_000))
        buffer.add(sample(2_000))
        buffer.add(sample(3_000))

        assertEquals(2_000L, buffer.nearestTo(1_900)?.timestamp_ns)
        assertEquals(3_000L, buffer.nearestTo(2_900)?.timestamp_ns)
        assertEquals(1_000L, buffer.nearestTo(0)?.timestamp_ns)
    }

    @Test
    fun `keeps only the newest capacity entries`() {
        val buffer = ImuRingBuffer(capacity = 2)
        buffer.add(sample(1_000))
        buffer.add(sample(2_000))
        buffer.add(sample(3_000))

        assertEquals(2, buffer.size())
        assertNull(buffer.nearestTo(1_000)?.takeIf { it.timestamp_ns == 1_000L })
        assertEquals(3_000L, buffer.latest()?.timestamp_ns)
    }

    @Test
    fun `clear drops every sample`() {
        val buffer = ImuRingBuffer()
        buffer.add(sample(1_000))
        buffer.clear()

        assertEquals(0, buffer.size())
        assertNull(buffer.latest())
    }
}
