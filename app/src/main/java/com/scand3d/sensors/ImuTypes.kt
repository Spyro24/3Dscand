package com.scand3d.sensors

import kotlin.math.sqrt

/** Meters per second squared, radians per second, microtesla. Right-handed, float64. */
data class Vec3(val x: Double, val y: Double, val z: Double) {
    val length: Double
        get() = sqrt(x * x + y * y + z * z)

    companion object {
        val ZERO = Vec3(0.0, 0.0, 0.0)
    }
}

/**
 * One reading from the motion sensors. Values are optional because the hardware is not uniform:
 * a phone without a magnetometer still produces usable samples, so presence travels with the
 * data instead of being guessed downstream.
 */
data class ImuSample(
    val timestamp_ns: Long,
    val gyro_rad_s: Vec3 = Vec3.ZERO,
    val accel_m_s2: Vec3 = Vec3.ZERO,
    val mag_u_t: Vec3? = null,
    val has_gyro: Boolean = false,
    val has_accel: Boolean = false,
    /** The OS-fused rotation vector, kept as a reference for device diagnostics only. */
    val rotation_vector_wxyz: List<Double>? = null,
) {
    val has_mag: Boolean
        get() = mag_u_t != null

    /** Layout the native bridge expects; see `bridge_api.hpp`. */
    fun toNativeValues(): DoubleArray {
        val mag = mag_u_t ?: Vec3.ZERO
        var flags = 0
        if (has_gyro) flags = flags or FLAG_GYRO
        if (has_accel) flags = flags or FLAG_ACCEL
        if (mag_u_t != null) flags = flags or FLAG_MAG
        return doubleArrayOf(
            timestamp_ns.toDouble(),
            gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z,
            accel_m_s2.x, accel_m_s2.y, accel_m_s2.z,
            mag.x, mag.y, mag.z,
            flags.toDouble(),
        )
    }

    companion object {
        const val FLAG_GYRO = 1
        const val FLAG_ACCEL = 2
        const val FLAG_MAG = 4
    }
}

/** Fused orientation at one sensor timestamp, quaternion {w, x, y, z}, body to world. */
data class ImuOrientationSample(
    val timestamp_ns: Long,
    val quaternion_wxyz: List<Double>,
)

/**
 * How orientation is being produced. Anything except [NineAxis] and [SixAxis] is reduced
 * accuracy, which agents.md section 4 requires the UI to say out loud.
 */
enum class ImuMode { NineAxis, SixAxis, TiltOnly, Unavailable }

data class ImuAvailability(
    val has_gyroscope: Boolean = false,
    val has_accelerometer: Boolean = false,
    val has_magnetometer: Boolean = false,
    val has_rotation_vector: Boolean = false,
) {
    val mode: ImuMode
        get() = when {
            // Gravity is the reference every mode needs; without it only a gyro remains, and a
            // gyro alone drifts without bound.
            has_gyroscope && has_magnetometer -> ImuMode.NineAxis
            has_gyroscope && has_accelerometer -> ImuMode.SixAxis
            has_accelerometer && !has_gyroscope -> ImuMode.TiltOnly
            else -> ImuMode.Unavailable
        }

    /** True when yaw is missing or unobservable, so the overlay cannot be trusted for heading. */
    val reduced_accuracy: Boolean
        get() = mode == ImuMode.TiltOnly || mode == ImuMode.Unavailable
}
