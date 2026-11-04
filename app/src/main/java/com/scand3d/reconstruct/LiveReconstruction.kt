package com.scand3d.reconstruct

import android.os.SystemClock
import android.util.Log
import com.scand3d.core.bridge.NativeCore
import com.scand3d.render.OverlayCameraParams
import java.io.File
import java.util.ArrayDeque

/**
 * Reconstructs while the capture is still running (roadmap 5.2): every keyframe the camera writes is
 * decoded and added to one long-lived native session, and a single worker thread re-runs the sparse
 * map at a bounded rate and hands the points to the overlay.
 *
 * The reconstruction runs from scratch on each update (`sfm::Session::run`), so the cost grows with
 * the square of the view count and the view count is capped. Captures longer than [MAX_VIEWS]
 * keyframes keep their first [MAX_VIEWS] views only: that is the honest limit of this loop, not a
 * property of a scan.
 */
class LiveReconstruction(
    private val on_cloud: (DoubleArray, FloatArray, DoubleArray?, Long) -> Unit,
    private val min_interval_ms: Long = MIN_INTERVAL_MS,
    private val max_views: Int = MAX_VIEWS,
) {

    private class Keyframe(
        val file: File,
        val camera: OverlayCameraParams,
        val orientation_wxyz: DoubleArray?,
    )

    private val lock = Object()
    private val pending = ArrayDeque<Keyframe>()
    private var thread: Thread? = null
    private var finishing = false
    private var closed = false
    private var views = 0
    private var views_since_run = 0
    private var last_run_ms = 0L
    private var reference_orientation: DoubleArray? = null

    /** Queues a keyframe that is already on disk; decoding and matching happen on the worker. */
    fun onKeyframe(file: File, camera: OverlayCameraParams, orientation_wxyz: List<Double>?) {
        synchronized(lock) {
            if (finishing || closed) {
                return
            }
            pending.addLast(Keyframe(file, camera, orientation_wxyz?.toDoubleArray()))
            lock.notifyAll()
        }
        startWorker()
    }

    /**
     * Stops taking keyframes and lets the worker finish the map it has. The final run still happens,
     * so the cloud on screen is the whole capture rather than wherever the last interval landed.
     */
    fun finish() {
        synchronized(lock) {
            if (finishing || closed) {
                return
            }
            finishing = true
            lock.notifyAll()
        }
        startWorker()
    }

    /** Joins the worker and releases the native session; safe to call from any state. */
    fun close() {
        val worker = synchronized(lock) {
            if (closed) {
                return
            }
            closed = true
            finishing = true
            pending.clear()
            lock.notifyAll()
            thread
        }
        worker?.let { runCatching { it.join(CLOSE_JOIN_MS) } }
        synchronized(lock) { thread = null }
    }

    private fun startWorker() {
        val created = synchronized(lock) {
            if (closed || thread != null) {
                return
            }
            Thread(::work, "live-reconstruction").apply {
                isDaemon = true
                thread = this
            }
        }
        created.start()
    }

    private fun work() {
        val handle = NativeCore.reconstructionCreate()
        if (handle == 0L) {
            Log.w(LOG_TAG, "native core unavailable, live reconstruction disabled")
            return
        }
        try {
            while (true) {
                val batch = ArrayDeque<Keyframe>()
                var done = false
                synchronized(lock) {
                    while (pending.isEmpty() && !finishing && !closed) {
                        lock.wait()
                    }
                    if (!finishing && !closed) {
                        // Hold the new views until the map has had time to settle: one run per
                        // keyframe would spend the whole capture inside matching.
                        val remaining =
                            min_interval_ms - (SystemClock.elapsedRealtime() - last_run_ms)
                        if (remaining > 0) {
                            lock.wait(remaining)
                        }
                        while (pending.isEmpty() && !finishing && !closed) {
                            lock.wait()
                        }
                    }
                    batch.addAll(pending)
                    pending.clear()
                    done = finishing || closed
                }

                var added = 0
                for (keyframe in batch) {
                    if (views >= max_views) {
                        if (views == max_views) {
                            Log.i(LOG_TAG, "view cap $max_views reached, later keyframes are not mapped")
                        }
                        // Counted once so the warning above is not repeated every run.
                        views += 1
                        break
                    }
                    val gray = GrayDecoder.decode(keyframe.file) ?: continue
                    val scaled = keyframe.camera.scaledTo(gray.width_px, gray.height_px)
                    val index = NativeCore.reconstructionAddView(
                        handle,
                        gray.pixels,
                        gray.width_px,
                        gray.height_px,
                        scaled.focal_px,
                        scaled.cx_px,
                        scaled.cy_px,
                    )
                    if (index < 0) {
                        Log.w(LOG_TAG, "view rejected: ${keyframe.file.name}")
                        continue
                    }
                    if (reference_orientation == null) {
                        // The session's first view is the frame the map is anchored to, so the
                        // overlay has to use the pose the phone held when that view was captured
                        // (D043). Anything later is a different camera pose.
                        reference_orientation = keyframe.orientation_wxyz
                    }
                    views += 1
                    views_since_run += 1
                    added += 1
                }

                if (added > 0 || views_since_run > 0) {
                    runOnce(handle)
                }
                if (done) {
                    return
                }
            }
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "live reconstruction stopped", t)
        } finally {
            NativeCore.reconstructionDestroy(handle)
        }
    }

    private fun runOnce(handle: Long) {
        val started = SystemClock.elapsedRealtime()
        synchronized(lock) { views_since_run = 0 }
        if (views < 2) {
            // A still phone produces one keyframe, because the prior only marks a frame once the
            // view has rotated away from the last one. Nothing to triangulate yet.
            Log.i(LOG_TAG, "live: $views view, waiting for a second keyframe")
            return
        }

        val registered = NativeCore.reconstructionRun(handle)
        val count = NativeCore.reconstructionPointCount(handle)
        val positions = if (count > 0) {
            val out = DoubleArray(count * 3)
            val written = NativeCore.reconstructionCopyPoints(handle, out)
            if (written > 0) out.copyOf(written * 3) else DoubleArray(0)
        } else {
            DoubleArray(0)
        }

        // One cloud per keyframe: each point carries the keyframe that first saw it (D059).
        val cloud_ids = IntArray(positions.size / 3)
        if (cloud_ids.isNotEmpty()) {
            NativeCore.reconstructionCopyKeyframeIds(handle, cloud_ids)
        }

        synchronized(lock) { last_run_ms = SystemClock.elapsedRealtime() }
        val elapsed = SystemClock.elapsedRealtime() - started
        Log.i(
            LOG_TAG,
            "live: $views views, $registered registered, ${positions.size / 3} points, " +
                "${elapsed} ms",
        )
        if (positions.isNotEmpty()) {
            // The snapshot is handed out with the cloud: the caller owns it from here and destroys
            // it when the next one replaces it (D051).
            on_cloud(
                positions,
                KeyframeClouds.colours(cloud_ids, positions.size / 3),
                reference_orientation,
                NativeCore.trackerCreate(handle),
            )
        }
    }

    private companion object {
        const val LOG_TAG = "LiveReconstruction"
        const val MIN_INTERVAL_MS = 1500L
        // Every update re-matches every pair, and the cost grows with the square of the view count:
        // 6 views took 6.3 s and 12 views 14.1 s on the SM-G930F. Eight keeps a run inside the
        // interval it replaces, so the cloud keeps up with the scan instead of lagging behind it.
        const val MAX_VIEWS = 8
        const val CLOSE_JOIN_MS = 4000L
    }
}
