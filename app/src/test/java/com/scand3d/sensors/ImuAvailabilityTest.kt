package com.scand3d.sensors

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ImuAvailabilityTest {

    @Test
    fun `gyroscope and magnetometer give nine-axis fusion`() {
        val availability = ImuAvailability(
            has_gyroscope = true,
            has_accelerometer = true,
            has_magnetometer = true,
        )

        assertEquals(ImuMode.NineAxis, availability.mode)
        assertFalse(availability.reduced_accuracy)
    }

    @Test
    fun `gyroscope without magnetometer is six-axis and still full accuracy`() {
        val availability = ImuAvailability(has_gyroscope = true, has_accelerometer = true)

        assertEquals(ImuMode.SixAxis, availability.mode)
        assertFalse(availability.reduced_accuracy)
    }

    @Test
    fun `no gyroscope falls back to gravity and flags reduced accuracy`() {
        val availability = ImuAvailability(
            has_gyroscope = false,
            has_accelerometer = true,
            has_magnetometer = true,
        )

        assertEquals(ImuMode.TiltOnly, availability.mode)
        assertTrue(availability.reduced_accuracy)
    }

    @Test
    fun `a gyroscope without gravity cannot produce an orientation`() {
        // Integrating the gyro alone drifts without bound, so there is no usable mode.
        val availability = ImuAvailability(has_gyroscope = true, has_accelerometer = false)

        assertEquals(ImuMode.Unavailable, availability.mode)
        assertTrue(availability.reduced_accuracy)
    }

    @Test
    fun `no sensors at all is unavailable`() {
        val availability = ImuAvailability()

        assertEquals(ImuMode.Unavailable, availability.mode)
        assertTrue(availability.reduced_accuracy)
    }

    @Test
    fun `the rotation vector alone does not make orientation available`() {
        // It is read for diagnostics only; the native filter does not consume it.
        val availability = ImuAvailability(has_rotation_vector = true)

        assertEquals(ImuMode.Unavailable, availability.mode)
    }
}
