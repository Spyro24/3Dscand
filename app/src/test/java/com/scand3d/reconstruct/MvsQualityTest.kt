package com.scand3d.reconstruct

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Test

/** The presets are the measured Phase 6 working widths; changing one changes the timing claim. */
class MvsQualityTest {

    @Test
    fun `presets map to the documented working widths`() {
        assertEquals(160, MvsQuality.Draft.working_width_px)
        assertEquals(320, MvsQuality.Normal.working_width_px)
        assertEquals(480, MvsQuality.High.working_width_px)
    }

    @Test
    fun `next cycles draft normal high`() {
        assertEquals(MvsQuality.Normal, MvsQuality.Draft.next())
        assertEquals(MvsQuality.High, MvsQuality.Normal.next())
        assertEquals(MvsQuality.Draft, MvsQuality.High.next())
    }

    @Test
    fun `each preset has its own label`() {
        val labels = MvsQuality.values().map { it.label_res }
        for (first in labels.indices) {
            for (second in first + 1 until labels.size) {
                assertNotEquals(labels[first], labels[second])
            }
        }
    }
}
