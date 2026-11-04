package com.scand3d.sensors

import org.junit.Assert.assertEquals
import org.junit.Test

class AngularRateMonitorTest {

    private val one_rad_per_s = Vec3(1.0, 0.0, 0.0)

    @Test
    fun `without a gyroscope the monitor reports unavailable`() {
        val monitor = AngularRateMonitor()

        assertEquals(
            MotionWarning.Unavailable,
            monitor.update(Vec3.ZERO, has_gyro = false, exposure_ns = 30_000_000),
        )
    }

    @Test
    fun `a long exposure at rest stays steady`() {
        val monitor = AngularRateMonitor()

        // 1 rad/s over 10 ms is 0.57 degrees of smear.
        assertEquals(
            MotionWarning.Steady,
            monitor.update(one_rad_per_s, has_gyro = true, exposure_ns = 10_000_000),
        )
    }

    @Test
    fun `a slow shutter while turning warns about blur`() {
        val monitor = AngularRateMonitor()

        // 1 rad/s over 30 ms is 1.72 degrees of smear.
        assertEquals(
            MotionWarning.BlurRisk,
            monitor.update(one_rad_per_s, has_gyro = true, exposure_ns = 30_000_000),
        )
    }

    @Test
    fun `the warning has hysteresis so it does not flicker`() {
        val monitor = AngularRateMonitor(enter_blur_deg = 1.5, exit_blur_deg = 1.0)

        assertEquals(
            MotionWarning.BlurRisk,
            monitor.update(one_rad_per_s, has_gyro = true, exposure_ns = 30_000_000),
        )
        // 1.20 degrees: below the entry threshold but above the exit one, so it must hold.
        assertEquals(
            MotionWarning.BlurRisk,
            monitor.update(one_rad_per_s, has_gyro = true, exposure_ns = 21_000_000),
        )
        // 0.57 degrees: now it clears.
        assertEquals(
            MotionWarning.Steady,
            monitor.update(one_rad_per_s, has_gyro = true, exposure_ns = 10_000_000),
        )
    }

    @Test
    fun `reset returns the monitor to steady`() {
        val monitor = AngularRateMonitor()
        monitor.update(one_rad_per_s, has_gyro = true, exposure_ns = 30_000_000)

        monitor.reset()

        assertEquals(MotionWarning.Steady, monitor.current)
    }
}
