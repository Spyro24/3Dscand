package com.scand3d.reconstruct

import kotlin.math.sqrt

/**
 * Depth window for the dense search, in the reconstruction's own free-scale units (D028).
 *
 * The sparse map points narrow the search per view (their depth range drives the sweep), but a view
 * the sparse map does not cover has to fall back to a window. The only scale available there is the
 * capture itself: how far the cameras sit from the cloud they produced. A turntable-looking scan
 * puts the object at roughly that distance, so the fallback is a wide window around the median
 * camera radius.
 */
object DenseRange {

    val DEFAULT = 0.2..3.0

    private const val NEAR_FACTOR = 0.25
    private const val FAR_FACTOR = 3.0

    /**
     * Window around the median distance from a camera centre to the cloud centroid. Empty or
     * degenerate input falls back to [DEFAULT].
     */
    fun from(
        positions_m: DoubleArray,
        centres_m: List<DoubleArray>,
    ): ClosedFloatingPointRange<Double> {
        val count = positions_m.size / 3
        if (count == 0 || centres_m.isEmpty()) {
            return DEFAULT
        }
        var x = 0.0
        var y = 0.0
        var z = 0.0
        for (index in 0 until count) {
            x += positions_m[index * 3]
            y += positions_m[index * 3 + 1]
            z += positions_m[index * 3 + 2]
        }
        x /= count
        y /= count
        z /= count

        val radii = DoubleArray(centres_m.size)
        for ((index, centre) in centres_m.withIndex()) {
            if (centre.size < 3) {
                return DEFAULT
            }
            val dx = centre[0] - x
            val dy = centre[1] - y
            val dz = centre[2] - z
            radii[index] = sqrt(dx * dx + dy * dy + dz * dz)
        }
        radii.sort()
        val median = radii[radii.size / 2]
        if (!median.isFinite() || median <= 0.0) {
            return DEFAULT
        }
        return (median * NEAR_FACTOR)..(median * FAR_FACTOR)
    }
}
