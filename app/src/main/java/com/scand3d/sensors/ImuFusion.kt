package com.scand3d.sensors

import com.scand3d.core.bridge.NativeCore

/** What [FusedImu] needs from an orientation filter, so tests can run without the native core. */
interface OrientationFilter {
    val isAvailable: Boolean

    /** Returns the orientation after this sample, or null while none is available yet. */
    fun push(sample: ImuSample): List<Double>?

    fun reset()

    fun close()
}

/**
 * Session handle for the native Mahony filter (roadmap 2.2). The bridge is optional: when the
 * library is missing the handle stays 0 and every call degrades to null instead of throwing,
 * matching the NativeCore policy in D009.
 */
class ImuFusion(private val sample_rate_hz: Double = 200.0) : OrientationFilter {

    private var handle: Long = NativeCore.imuCreate(sample_rate_hz)

    override val isAvailable: Boolean
        get() = handle != 0L

    override fun push(sample: ImuSample): List<Double>? {
        if (handle == 0L) {
            return null
        }
        if (!NativeCore.imuUpdate(handle, sample.toNativeValues())) {
            return null
        }
        val out = DoubleArray(QUATERNION_SIZE)
        return if (NativeCore.imuOrientation(handle, out)) out.toList() else null
    }

    override fun reset() {
        if (handle != 0L) {
            NativeCore.imuReset(handle)
        }
    }

    override fun close() {
        val current = handle
        handle = 0L
        if (current != 0L) {
            NativeCore.imuDestroy(current)
        }
    }

    private companion object {
        const val QUATERNION_SIZE = 4
    }
}
