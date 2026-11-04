package com.scand3d.sensors

import kotlin.math.PI

enum class MotionWarning { Steady, BlurRisk, Unavailable }

/**
 * Roadmap 2.4. Turns the angular rate and the locked exposure time into a blur warning: the
 * smear during one exposure is the rate times the exposure, and a fraction of a degree is already
 * visible at this focal length. The two thresholds give it hysteresis so the warning does not
 * flicker while the phone hovers at the boundary.
 */
class AngularRateMonitor(
    private val enter_blur_deg: Double = 1.5,
    private val exit_blur_deg: Double = 1.0,
    private val fallback_exposure_s: Double = 1.0 / 30.0,
) {
    var current: MotionWarning = MotionWarning.Steady
        private set

    fun update(gyro_rad_s: Vec3, has_gyro: Boolean, exposure_ns: Long): MotionWarning {
        if (!has_gyro) {
            current = MotionWarning.Unavailable
            return current
        }

        val exposure_s = if (exposure_ns > 0L) exposure_ns / 1_000_000_000.0 else fallback_exposure_s
        val blur_deg = gyro_rad_s.length * exposure_s * 180.0 / PI

        current = when {
            current == MotionWarning.BlurRisk && blur_deg > exit_blur_deg -> MotionWarning.BlurRisk
            blur_deg > enter_blur_deg -> MotionWarning.BlurRisk
            else -> MotionWarning.Steady
        }
        return current
    }

    fun reset() {
        current = MotionWarning.Steady
    }
}
