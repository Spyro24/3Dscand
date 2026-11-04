package com.scand3d.reconstruct

import android.os.SystemClock
import android.util.Log
import com.scand3d.core.bridge.NativeCore
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Tracks the live preview against the map (roadmap 5.2, D051). The map arrives as a native
 * snapshot handle, so a pose estimate never waits behind the seconds a map run takes.
 *
 * One worker and at most one frame in flight: a frame that arrives while the worker is busy is
 * dropped instead of queued, because its pose would describe a camera that has already moved.
 */
class LiveTracking {

    /** A camera-to-world pose in the reconstruction's own (free-scale) frame, with its support. */
    class Pose(
        val rotation: DoubleArray,
        val centre_m: DoubleArray,
        val inliers: Int,
        val at_ms: Long,
    )

    @Volatile
    private var tracker = 0L

    @Volatile
    private var latest: Pose? = null

    private val worker = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "live-tracking").apply { isDaemon = true }
    }
    private val busy = AtomicBoolean(false)

    /** Replaces the map snapshot; the previous handle is destroyed here, on the calling thread. */
    fun replaceTracker(handle: Long) {
        val previous = tracker
        tracker = handle
        latest = null
        if (previous != 0L && previous != handle) {
            NativeCore.trackerDestroy(previous)
        }
    }

    /** Newest pose, or null when tracking has not produced one within [MAX_AGE_MS]. */
    fun pose(): Pose? {
        val current = latest ?: return null
        return if (SystemClock.elapsedRealtime() - current.at_ms > MAX_AGE_MS) null else current
    }

    /** Feeds one preview frame; frames at the analysis stream's own rate already bound the cost. */
    fun onFrame(gray: ByteArray, width_px: Int, height_px: Int, focal_px: Double) {
        val handle = tracker
        if (handle == 0L || !(focal_px > 0.0) || width_px <= 0 || height_px <= 0) {
            return
        }
        if (gray.size != width_px * height_px) {
            return
        }
        if (!busy.compareAndSet(false, true)) {
            return
        }
        worker.execute {
            try {
                val rotation = DoubleArray(9)
                val centre = DoubleArray(3)
                val inliers = NativeCore.trackerTrack(
                    handle,
                    gray,
                    width_px,
                    height_px,
                    focal_px,
                    width_px / 2.0,
                    height_px / 2.0,
                    rotation,
                    centre,
                )
                // A frame the map cannot place leaves the previous pose in place until it ages out,
                // which is what makes a momentary blur a dropped frame rather than a jump.
                if (inliers > 0) {
                    latest = Pose(rotation, centre, inliers, SystemClock.elapsedRealtime())
                }
            } catch (t: Throwable) {
                Log.w(LOG_TAG, "tracking frame failed", t)
            } finally {
                busy.set(false)
            }
        }
    }

    fun close() {
        worker.shutdownNow()
        replaceTracker(0L)
    }

    private companion object {
        const val LOG_TAG = "LiveTracking"

        /** Older than this the phone has moved too far for the pose to still describe the view. */
        const val MAX_AGE_MS = 600L
    }
}
