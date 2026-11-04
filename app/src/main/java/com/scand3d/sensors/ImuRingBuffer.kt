package com.scand3d.sensors

import kotlin.math.abs

/**
 * Bounded history of fused orientations, so a frame can be paired with the sample nearest its
 * own timestamp. The sensor thread writes, the frame writer reads.
 */
class ImuRingBuffer(private val capacity: Int = 512) {

    private val samples = ArrayDeque<ImuOrientationSample>(capacity)

    @Synchronized
    fun add(sample: ImuOrientationSample) {
        if (samples.size == capacity) {
            samples.removeFirst()
        }
        samples.addLast(sample)
    }

    @Synchronized
    fun latest(): ImuOrientationSample? = samples.lastOrNull()

    @Synchronized
    fun size(): Int = samples.size

    @Synchronized
    fun clear() {
        samples.clear()
    }

    /**
     * Nearest sample by timestamp. A linear scan over at most [capacity] entries is cheap at the
     * ~2 frames/s capture rate, and it tolerates sensors whose samples arrive out of order.
     */
    @Synchronized
    fun nearestTo(timestamp_ns: Long): ImuOrientationSample? {
        var best: ImuOrientationSample? = null
        var best_delta_ns = Long.MAX_VALUE
        for (sample in samples) {
            val delta_ns = abs(sample.timestamp_ns - timestamp_ns)
            if (delta_ns < best_delta_ns) {
                best_delta_ns = delta_ns
                best = sample
            }
        }
        return best
    }
}
