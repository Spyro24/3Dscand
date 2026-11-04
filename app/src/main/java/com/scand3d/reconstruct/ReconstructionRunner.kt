package com.scand3d.reconstruct

import android.util.Log
import com.scand3d.core.bridge.NativeCore
import com.scand3d.render.OverlayCameraParams
import java.io.File

/** One dense pass request (roadmap 6.4): the width the depth maps are estimated at. */
data class DenseRequest(val working_width_px: Int)

/**
 * Runs the app-side path from saved keyframes to a point cloud: decode each frame to gray, add it to
 * a native reconstruction session, and hand back the points. This is the wiring D030 records as
 * missing; the heavy work stays in `core/sfm/session.cpp`.
 *
 * With a [DenseRequest] the same run then feeds the dense stage (roadmap 6): the sparse poses place
 * the views, the sparse points narrow the depth search, and the fused cloud comes back alongside the
 * sparse one.
 *
 * The result is in the reconstruction's own frame, where the first camera is the origin and the
 * scale is free, so callers must not read metric distances out of it (D028).
 */
class ReconstructionRunner {

    data class Outcome(
        val added_views: Int,
        val registered_views: Int,
        val positions_m: DoubleArray,
        val dense_positions_m: DoubleArray = DoubleArray(0),
        val dense_views: Int = 0,
        val cancelled: Boolean = false,
    ) {
        val points: Int get() = positions_m.size / 3
        val dense_points: Int get() = dense_positions_m.size / 3
    }

    @Volatile
    private var cancel_requested = false

    @Volatile
    private var dense_handle = 0L

    /** Stops the running pass; a cancelled dense stage publishes no cloud. */
    fun cancel() {
        cancel_requested = true
        val handle = dense_handle
        if (handle != 0L) {
            NativeCore.denseCancel(handle)
        }
    }

    fun run(
        frames: List<File>,
        camera: OverlayCameraParams,
        max_edge_px: Int = GrayDecoder.GRAY_MAX_EDGE_PX,
        dense: DenseRequest? = null,
        on_dense_progress: ((Double) -> Unit)? = null,
    ): Outcome {
        val handle = NativeCore.reconstructionCreate()
        if (handle == 0L) {
            Log.w(LOG_TAG, "native core unavailable, cannot reconstruct")
            return Outcome(0, 0, DoubleArray(0))
        }
        try {
            // The decoded frames are kept for the dense stage: decoding a second time would cost
            // more than holding a few hundred kilobytes each.
            val decoded = ArrayList<Pair<GrayImage, OverlayCameraParams>>()
            var added = 0
            for (file in frames) {
                if (cancel_requested) {
                    return Outcome(added, 0, DoubleArray(0), cancelled = true)
                }
                val gray = GrayDecoder.decode(file, max_edge_px) ?: continue
                // The keyframe is decoded smaller than the preview, so the intrinsics follow it.
                val scaled = camera.scaledTo(gray.width_px, gray.height_px)
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
                    continue
                }
                decoded.add(gray to scaled)
                added += 1
            }
            if (added < 2) {
                Log.w(LOG_TAG, "only $added usable views, nothing to reconstruct")
                return Outcome(added, 0, DoubleArray(0))
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
            Log.i(LOG_TAG, "$added views, $registered registered, ${positions.size / 3} points")
            if (dense == null) {
                return Outcome(added, registered, positions)
            }
            if (positions.isEmpty()) {
                // Without sparse points there is no frame for the dense cloud to live in and no
                // depth hint range, so a dense pass on top of it would be guesswork.
                Log.w(LOG_TAG, "dense skipped: the sparse stage produced no points")
                return Outcome(added, registered, positions)
            }

            val cloud = runDense(
                reconstruction_handle = handle,
                decoded = decoded,
                sparse_positions_m = positions,
                working_width_px = dense.working_width_px,
                on_progress = on_dense_progress,
            )
            return Outcome(
                added_views = added,
                registered_views = registered,
                positions_m = positions,
                dense_positions_m = cloud.points,
                dense_views = cloud.views,
                cancelled = cloud.cancelled,
            )
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "reconstruction failed", t)
            return Outcome(0, 0, DoubleArray(0))
        } finally {
            NativeCore.reconstructionDestroy(handle)
        }
    }

    private class DenseCloud(
        val points: DoubleArray,
        val views: Int,
        val cancelled: Boolean,
    )

    /**
     * Adds every view the sparse stage placed to a dense session and runs it. The views keep the
     * poses the reconstruction gave them, which is what makes the fused cloud land in the same
     * frame as the sparse map.
     */
    private fun runDense(
        reconstruction_handle: Long,
        decoded: List<Pair<GrayImage, OverlayCameraParams>>,
        sparse_positions_m: DoubleArray,
        working_width_px: Int,
        on_progress: ((Double) -> Unit)?,
    ): DenseCloud {
        val handle = NativeCore.denseCreate()
        if (handle == 0L) {
            Log.w(LOG_TAG, "dense session unavailable, native core not loaded")
            return DenseCloud(DoubleArray(0), 0, false)
        }
        dense_handle = handle
        try {
            val centres = ArrayList<DoubleArray>()
            for ((index, entry) in decoded.withIndex()) {
                if (cancel_requested) {
                    return DenseCloud(DoubleArray(0), centres.size, true)
                }
                val rotation = DoubleArray(9)
                val centre = DoubleArray(3)
                if (!NativeCore.reconstructionCopyCamera(
                        reconstruction_handle,
                        index,
                        rotation,
                        centre,
                    )
                ) {
                    // A view the sparse stage did not place has no pose to estimate depth from.
                    continue
                }
                val (gray, camera) = entry
                val added = NativeCore.denseAddView(
                    handle,
                    gray.pixels,
                    gray.width_px,
                    gray.height_px,
                    camera.focal_px,
                    camera.cx_px,
                    camera.cy_px,
                    rotation,
                    centre,
                )
                if (added < 0) {
                    continue
                }
                centres.add(centre)
            }
            if (centres.size < 2) {
                Log.w(LOG_TAG, "dense: only ${centres.size} placed views, nothing to fuse")
                return DenseCloud(DoubleArray(0), centres.size, false)
            }

            NativeCore.denseSetHints(handle, sparse_positions_m)
            val range = DenseRange.from(sparse_positions_m, centres)
            Log.i(
                LOG_TAG,
                "dense: ${centres.size} views at $working_width_px px, " +
                    "fallback depth ${format(range.start)}..${format(range.endInclusive)}",
            )
            val poller = startProgressPolling(handle, on_progress)
            val fused = try {
                NativeCore.denseRun(
                    handle,
                    working_width_px,
                    range.start,
                    range.endInclusive,
                )
            } finally {
                poller?.interrupt()
            }

            val count = NativeCore.densePointCount(handle)
            val points = if (count > 0) {
                val out = DoubleArray(count * 3)
                val written = NativeCore.denseCopyPoints(handle, out)
                if (written > 0) out.copyOf(written * 3) else DoubleArray(0)
            } else {
                DoubleArray(0)
            }
            Log.i(
                LOG_TAG,
                "dense: $fused fused points from ${centres.size} views, ${points.size / 3} copied",
            )
            return DenseCloud(points, centres.size, cancel_requested)
        } finally {
            NativeCore.denseDestroy(handle)
            dense_handle = 0L
        }
    }

    /** Reports the native progress while [NativeCore.denseRun] blocks this thread. */
    private fun startProgressPolling(
        handle: Long,
        on_progress: ((Double) -> Unit)?,
    ): Thread? {
        if (on_progress == null) {
            return null
        }
        val thread = Thread(
            {
                while (!Thread.currentThread().isInterrupted) {
                    on_progress(NativeCore.denseProgress(handle))
                    try {
                        Thread.sleep(PROGRESS_POLL_MS)
                    } catch (interrupted: InterruptedException) {
                        return@Thread
                    }
                }
            },
            "dense-progress",
        )
        thread.isDaemon = true
        thread.start()
        return thread
    }

    private fun format(value: Double): String =
        String.format(java.util.Locale.US, "%.3f", value)

    private companion object {
        const val LOG_TAG = "Reconstruction"
        const val PROGRESS_POLL_MS = 250L
    }
}
