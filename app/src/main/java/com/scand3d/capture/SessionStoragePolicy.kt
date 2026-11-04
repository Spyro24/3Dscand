package com.scand3d.capture

import kotlin.math.max
import kotlin.math.roundToInt

/**
 * What a scan keeps on disk (D050). Every still used to be written at full resolution, about 3 MB
 * each, at one still every 120 ms: a 20 minute recording wrote several GB and filled the phone.
 * Only keyframes reach any later stage (the live map, the reconstruction, the dense pass, the
 * colour sampler), and the pipeline decodes them at 480 px or less, so the rest of the pixels and
 * the rest of the frames were paid for and never read.
 */
object SessionStoragePolicy {

    /** Frames a scan keeps; the memory budget in agents.md section 6 is written for 150 keyframes. */
    const val MAX_FRAMES = 150

    /** Long edge of a stored frame; the quality gate names 1600 px for the MVS input. */
    const val MAX_LONG_EDGE_PX = 1600

    /** Stills without a keyframe before one is kept anyway, so a dead gyro cannot stop the scan. */
    const val FALLBACK_STILLS = 40

    /** Stored frames are re-encoded once, so the camera's own quality does not decide their size. */
    const val JPEG_QUALITY = 90

    /**
     * True when this still is worth a file. With a keyframe prior only candidates are kept, because
     * a non-keyframe is never read again; without one every still is a candidate and the frame
     * count alone bounds the session.
     */
    fun shouldKeep(
        has_keyframe_prior: Boolean,
        keyframe_candidate: Boolean,
        stills_since_kept: Int,
        kept: Int,
    ): Boolean {
        if (kept >= MAX_FRAMES) {
            return false
        }
        if (!has_keyframe_prior) {
            return true
        }
        return keyframe_candidate || stills_since_kept >= FALLBACK_STILLS
    }

    /** Largest power-of-two decode factor that still leaves the long edge at or above 1600 px. */
    fun sampleSize(long_edge_px: Int): Int {
        var sample = 1
        while (long_edge_px / (sample * 2) >= MAX_LONG_EDGE_PX) {
            sample *= 2
        }
        return sample
    }

    /** Stored size for a decoded frame: the long edge is at most 1600 px, the aspect is kept. */
    fun scaledSize(width_px: Int, height_px: Int): Pair<Int, Int> {
        val long_edge = max(width_px, height_px)
        if (long_edge <= MAX_LONG_EDGE_PX || width_px <= 0 || height_px <= 0) {
            return width_px to height_px
        }
        val scale = MAX_LONG_EDGE_PX.toDouble() / long_edge
        return max(1, (width_px * scale).roundToInt()) to max(1, (height_px * scale).roundToInt())
    }
}
