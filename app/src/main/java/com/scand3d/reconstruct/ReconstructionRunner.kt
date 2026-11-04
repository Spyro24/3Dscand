package com.scand3d.reconstruct

import android.util.Log
import com.scand3d.core.bridge.NativeCore
import com.scand3d.render.OverlayCameraParams
import java.io.File

/**
 * Runs the app-side path from saved keyframes to a sparse map: decode each frame to gray, add it to
 * a native reconstruction session, and hand back the points. This is the wiring D030 records as
 * missing; the heavy work stays in `core/sfm/session.cpp`.
 *
 * The result is in the reconstruction's own frame, where the first camera is the origin and the
 * scale is free, so callers must not read metric distances out of it (D028).
 */
class ReconstructionRunner {

    data class Outcome(
        val added_views: Int,
        val registered_views: Int,
        val positions_m: DoubleArray,
    ) {
        val points: Int get() = positions_m.size / 3
    }

    fun run(
        frames: List<File>,
        camera: OverlayCameraParams,
        max_edge_px: Int = GrayDecoder.GRAY_MAX_EDGE_PX,
    ): Outcome {
        val handle = NativeCore.reconstructionCreate()
        if (handle == 0L) {
            Log.w(LOG_TAG, "native core unavailable, cannot reconstruct")
            return Outcome(0, 0, DoubleArray(0))
        }
        try {
            var added = 0
            for (file in frames) {
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
                if (index >= 0) {
                    added += 1
                }
            }
            if (added < 2) {
                Log.w(LOG_TAG, "only $added usable views, nothing to reconstruct")
                return Outcome(added, 0, DoubleArray(0))
            }

            val registered = NativeCore.reconstructionRun(handle)
            val count = NativeCore.reconstructionPointCount(handle)
            if (count <= 0) {
                Log.w(LOG_TAG, "$added views, $registered registered, no points")
                return Outcome(added, registered, DoubleArray(0))
            }
            val positions = DoubleArray(count * 3)
            val written = NativeCore.reconstructionCopyPoints(handle, positions)
            val result = if (written > 0) positions.copyOf(written * 3) else DoubleArray(0)
            Log.i(LOG_TAG, "$added views, $registered registered, ${result.size / 3} points")
            return Outcome(added, registered, result)
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "reconstruction failed", t)
            return Outcome(0, 0, DoubleArray(0))
        } finally {
            NativeCore.reconstructionDestroy(handle)
        }
    }

    private companion object {
        const val LOG_TAG = "Reconstruction"
    }
}
