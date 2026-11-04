package com.scand3d.sensors

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class TimestampSyncTest {

    @Test
    fun `an empty window has no estimate`() {
        val sync = TimestampSync()

        assertNull(sync.medianResidualNs())
        assertNull(sync.medianAbsoluteDeviationNs())
        assertFalse(sync.isAligned())
    }

    @Test
    fun `median ignores a single bad pairing`() {
        val sync = TimestampSync(window = 5, tolerance_ns = 25_000_000)
        sync.observe(frame_timestamp_ns = 1_000, sample_timestamp_ns = 1_000 + 10_000)
        sync.observe(frame_timestamp_ns = 1_000, sample_timestamp_ns = 1_000 + 20_000)
        // One sample from a stalled sensor must not move the median.
        sync.observe(frame_timestamp_ns = 1_000, sample_timestamp_ns = 1_000 + 900_000)
        sync.observe(frame_timestamp_ns = 1_000, sample_timestamp_ns = 1_000 + 20_000)
        sync.observe(frame_timestamp_ns = 1_000, sample_timestamp_ns = 1_000 + 20_000)

        assertEquals(20_000L, sync.medianResidualNs())
        assertTrue(sync.isAligned())
    }

    @Test
    fun `an even window averages the two middle values`() {
        val sync = TimestampSync()
        sync.observe(0, 10)
        sync.observe(0, 20)

        assertEquals(15L, sync.medianResidualNs())
        assertEquals(5L, sync.medianAbsoluteDeviationNs())
    }

    @Test
    fun `the window drops old residuals`() {
        val sync = TimestampSync(window = 2)
        sync.observe(0, 10)
        sync.observe(0, 20)
        sync.observe(0, 30)

        assertEquals(2, sync.samples())
        assertEquals(25L, sync.medianResidualNs())
    }

    @Test
    fun `a persistent offset is reported as not aligned`() {
        val sync = TimestampSync(tolerance_ns = 5_000_000)
        repeat(4) { sync.observe(frame_timestamp_ns = 0, sample_timestamp_ns = 50_000_000) }

        assertEquals(50_000_000L, sync.medianResidualNs())
        assertFalse(sync.isAligned())
    }

    @Test
    fun `clear drops the evidence`() {
        val sync = TimestampSync()
        sync.observe(0, 10)
        sync.clear()

        assertEquals(0, sync.samples())
        assertNull(sync.medianResidualNs())
    }
}
