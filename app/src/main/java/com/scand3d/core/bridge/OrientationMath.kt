package com.scand3d.core.bridge

/**
 * Rotation math the annotator needs. An interface so host unit tests can supply exact values
 * without the arm64-only native library, which the JVM cannot load (D011).
 */
interface OrientationMath {
    /** Rotation angle between two {w, x, y, z} quaternions, in radians; NaN for bad input. */
    fun angularDisplacementRad(a: List<Double>, b: List<Double>): Double
}

object NativeOrientationMath : OrientationMath {
    override fun angularDisplacementRad(a: List<Double>, b: List<Double>): Double =
        NativeCore.angularDisplacementRad(a, b)
}
