package com.scand3d.sensors

import kotlin.math.abs

/**
 * Roadmap 2.3. Camera frames and sensor events are both on the boot-relative clock
 * (`elapsedRealtimeNanos`), so what is left to measure is the residual between a frame timestamp
 * and the sensor sample nearest it. The median is the offset a later stage subtracts when it
 * compares the two streams; the median absolute deviation says how trustworthy that number is.
 *
 * Thread-safe: the frame writer observes while the manifest writer reads.
 */
class TimestampSync(
    private val window: Int = 120,
    private val tolerance_ns: Long = 20_000_000L,
) {
    private val residuals = ArrayDeque<Long>()

    @Synchronized
    fun observe(frame_timestamp_ns: Long, sample_timestamp_ns: Long): Long {
        val residual_ns = sample_timestamp_ns - frame_timestamp_ns
        if (residuals.size == window) {
            residuals.removeFirst()
        }
        residuals.addLast(residual_ns)
        return residual_ns
    }

    @Synchronized
    fun samples(): Int = residuals.size

    @Synchronized
    fun medianResidualNs(): Long? {
        val median = medianOf(residuals.toList()) ?: return null
        return median.numerator / median.denominator
    }

    @Synchronized
    fun medianAbsoluteDeviationNs(): Long? {
        val median = medianResidualNs() ?: return null
        val deviations = residuals.map { abs(it - median) }
        val mad = medianOf(deviations) ?: return null
        return mad.numerator / mad.denominator
    }

    /** True when the two streams line up within [tolerance_ns] of each other. */
    @Synchronized
    fun isAligned(): Boolean {
        val median = medianResidualNs() ?: return false
        return abs(median) <= tolerance_ns
    }

    @Synchronized
    fun clear() {
        residuals.clear()
    }

    private data class Ratio(val numerator: Long, val denominator: Long)

    /** Exact median of a small window, as a ratio so an even count does not lose precision. */
    private fun medianOf(values: List<Long>): Ratio? {
        if (values.isEmpty()) {
            return null
        }
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 1) {
            Ratio(sorted[middle], 1L)
        } else {
            Ratio(sorted[middle - 1] + sorted[middle], 2L)
        }
    }
}
