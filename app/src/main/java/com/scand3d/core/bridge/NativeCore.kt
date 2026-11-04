package com.scand3d.core.bridge

/**
 * The only place Kotlin reaches the native core (agents.md section 3). Loading happens once,
 * when this object is first touched, and the failure is kept instead of thrown so the UI can
 * report a broken native build rather than crashing on launch.
 *
 * Every call below degrades to a safe value when the library is absent, so host unit tests and a
 * damaged APK take the same path.
 */
object NativeCore {

    private const val LIBRARY_NAME = "scand3d_core"
    private const val QUATERNION_SIZE = 4

    /** Null when the shared library loaded successfully. */
    val loadError: Throwable? = try {
        System.loadLibrary(LIBRARY_NAME)
        null
    } catch (t: UnsatisfiedLinkError) {
        t
    }

    val isLoaded: Boolean
        get() = loadError == null

    /**
     * Version reported by the native core, or null when the library did not load. Calling this
     * is what proves the bridge works end to end.
     */
    fun coreVersionOrNull(): String? = if (isLoaded) nativeCoreVersion() else null

    /** Opens a fusion session; 0 when the native core is unavailable. */
    fun imuCreate(sample_rate_hz: Double): Long =
        if (isLoaded) nativeImuCreate(sample_rate_hz) else 0L

    fun imuUpdate(handle: Long, values: DoubleArray): Boolean =
        isLoaded && handle != 0L && nativeImuUpdate(handle, values)

    fun imuOrientation(handle: Long, out: DoubleArray): Boolean =
        isLoaded && handle != 0L && out.size >= QUATERNION_SIZE && nativeImuOrientation(handle, out)

    fun imuReset(handle: Long) {
        if (isLoaded && handle != 0L) nativeImuReset(handle)
    }

    fun imuDestroy(handle: Long) {
        if (isLoaded && handle != 0L) nativeImuDestroy(handle)
    }

    fun angularDisplacementRad(a: List<Double>, b: List<Double>): Double =
        if (isLoaded && a.size == QUATERNION_SIZE && b.size == QUATERNION_SIZE) {
            nativeAngularDisplacementRad(a.toDoubleArray(), b.toDoubleArray())
        } else {
            Double.NaN
        }

    fun keyframeBaselineAccept(
        reference: List<Double>,
        candidate: List<Double>,
        min_baseline_rad: Double,
        max_baseline_rad: Double,
    ): Boolean =
        isLoaded &&
            reference.size == QUATERNION_SIZE &&
            candidate.size == QUATERNION_SIZE &&
            nativeKeyframeBaselineAccept(
                reference.toDoubleArray(),
                candidate.toDoubleArray(),
                min_baseline_rad,
                max_baseline_rad,
            )

    private external fun nativeCoreVersion(): String

    private external fun nativeImuCreate(sample_rate_hz: Double): Long

    private external fun nativeImuDestroy(handle: Long)

    private external fun nativeImuReset(handle: Long)

    private external fun nativeImuUpdate(handle: Long, values: DoubleArray): Boolean

    private external fun nativeImuOrientation(handle: Long, out: DoubleArray): Boolean

    private external fun nativeAngularDisplacementRad(a: DoubleArray, b: DoubleArray): Double

    private external fun nativeKeyframeBaselineAccept(
        reference: DoubleArray,
        candidate: DoubleArray,
        min_baseline_rad: Double,
        max_baseline_rad: Double,
    ): Boolean
}
