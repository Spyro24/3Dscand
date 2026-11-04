package com.scand3d.capture

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Log
import com.scand3d.core.bridge.NativeCore
import com.scand3d.render.OverlayCameraParams
import java.io.File

/**
 * Samples overlay point colours from a saved keyframe JPEG (roadmap 5.3). The native scene holds
 * the points, so only the keyframe pixels cross the bridge: decode, pack to RGB and let
 * `overlay::samplePointColors` take a median around each projection.
 */
object KeyframeColorSampler {

    private const val LOG_TAG = "KeyframeColor"
    private const val MAX_EDGE_PX = 720

    /**
     * Re-samples every point of the scene at `handle` from `jpeg`, using `camera` scaled to the
     * decoded image. Returns the number of points that received a sample, 0 when nothing could be
     * read. Never throws: a broken keyframe simply leaves the colours alone.
     */
    fun sample(handle: Long, camera: OverlayCameraParams, jpeg: File): Int {
        if (handle == 0L || !jpeg.isFile) {
            return 0
        }
        val bitmap = runCatching { decode(jpeg) }.getOrNull() ?: return 0
        return try {
            val width = bitmap.width
            val height = bitmap.height
            val pixels = IntArray(width * height)
            bitmap.getPixels(pixels, 0, width, 0, 0, width, height)
            val rgb = ByteArray(width * height * 3)
            for (index in pixels.indices) {
                val colour = pixels[index]
                rgb[index * 3] = ((colour shr 16) and 0xFF).toByte()
                rgb[index * 3 + 1] = ((colour shr 8) and 0xFF).toByte()
                rgb[index * 3 + 2] = (colour and 0xFF).toByte()
            }
            val scaled = camera.scaledTo(width, height)
            val sampled = NativeCore.overlaySampleColors(
                handle,
                scaled.rotation,
                scaled.centre_m,
                scaled.focal_px,
                scaled.cx_px,
                scaled.cy_px,
                scaled.image_width_px,
                scaled.image_height_px,
                rgb,
                width,
                height,
            )
            Log.i(LOG_TAG, "sampled $sampled points from ${jpeg.name} ${width}x$height")
            sampled
        } catch (t: Throwable) {
            Log.w(LOG_TAG, "sampling failed", t)
            0
        } finally {
            bitmap.recycle()
        }
    }

    private fun decode(jpeg: File): Bitmap {
        val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
        BitmapFactory.decodeFile(jpeg.absolutePath, bounds)
        var sampleSize = 1
        val longEdge = maxOf(bounds.outWidth, bounds.outHeight)
        while (longEdge / (sampleSize * 2) >= MAX_EDGE_PX) {
            sampleSize *= 2
        }
        return BitmapFactory.decodeFile(
            jpeg.absolutePath,
            BitmapFactory.Options().apply { inSampleSize = sampleSize },
        ) ?: error("decode returned nothing")
    }
}
