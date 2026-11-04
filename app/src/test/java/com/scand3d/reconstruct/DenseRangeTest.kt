package com.scand3d.reconstruct

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The dense search needs a depth window in the reconstruction's own units for views the sparse map
 * does not cover (D028). These cases pin the window to the camera radius, not to metres.
 */
class DenseRangeTest {

    @Test
    fun `window is centred on the median camera radius`() {
        // Cloud centred at the origin, cameras two units away: 0.25 * 2 .. 3 * 2.
        val points = doubleArrayOf(1.0, 0.0, 0.0, -1.0, 0.0, 0.0)
        val centres = listOf(doubleArrayOf(0.0, 2.0, 0.0), doubleArrayOf(0.0, -2.0, 0.0))

        val range = DenseRange.from(points, centres)

        assertEquals(0.5, range.start, 1e-9)
        assertEquals(6.0, range.endInclusive, 1e-9)
    }

    @Test
    fun `empty input falls back to the default window`() {
        val empty = DenseRange.from(DoubleArray(0), emptyList())
        assertEquals(DenseRange.DEFAULT.start, empty.start, 1e-9)
        assertEquals(DenseRange.DEFAULT.endInclusive, empty.endInclusive, 1e-9)

        val no_centres = DenseRange.from(doubleArrayOf(1.0, 0.0, 0.0), emptyList())
        assertEquals(DenseRange.DEFAULT.start, no_centres.start, 1e-9)
    }

    @Test
    fun `a camera sitting on the cloud centroid keeps a usable window`() {
        val range = DenseRange.from(
            doubleArrayOf(1.0, 0.0, 0.0),
            listOf(doubleArrayOf(1.0, 0.0, 0.0)),
        )

        assertTrue(range.start > 0.0)
        assertTrue(range.endInclusive > range.start)
    }
}
