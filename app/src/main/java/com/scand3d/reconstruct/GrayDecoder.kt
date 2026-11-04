package com.scand3d.reconstruct

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Log
import java.io.File

/** An 8-bit gray frame at the size the matcher works at. */
class GrayImage(val width_px: Int, val height_px: Int, val pixels: ByteArray)

/**
 * Decodes a saved keyframe to the gray image the native matcher expects. Downsampling happens in
 * the decoder, so a 12 MP still never lands in the Java heap at full size. The one-shot runner and
 * the live reconstruction share this so both feed the matcher identical pixels.
 */
object GrayDecoder {

    const val GRAY_MAX_EDGE_PX = 480

    private const val LOG_TAG = "Reconstruction"

    fun decode(file: File, max_edge_px: Int = GRAY_MAX_EDGE_PX): GrayImage? {
        return try {
            val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
            BitmapFactory.decodeFile(file.absolutePath, bounds)
            if (bounds.outWidth <= 0 || bounds.outHeight <= 0) {
                null
            } else {
                var sample = 1
                val long_edge = maxOf(bounds.outWidth, bounds.outHeight)
                while (long_edge / (sample * 2) >= max_edge_px) {
                    sample *= 2
                }
                val bitmap = BitmapFactory.decodeFile(
                    file.absolutePath,
                    BitmapFactory.Options().apply { inSampleSize = sample },
                )
                if (bitmap == null) {
                    null
                } else {
                    try {
                        toGray(bitmap)
                    } finally {
                        bitmap.recycle()
                    }
                }
            }
        } catch (t: Throwable) {
            Log.w(LOG_TAG, "could not decode ${file.name}", t)
            null
        }
    }

    private fun toGray(bitmap: Bitmap): GrayImage {
        val width = bitmap.width
        val height = bitmap.height
        val pixels = IntArray(width * height)
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height)
        val gray = ByteArray(width * height)
        for (index in pixels.indices) {
            val colour = pixels[index]
            val red = (colour shr 16) and 0xFF
            val green = (colour shr 8) and 0xFF
            val blue = colour and 0xFF
            // Integer luma in the same 0.299/0.587/0.114 proportions the preview shader uses.
            gray[index] = ((red * 77 + green * 150 + blue * 29) shr 8).toByte()
        }
        return GrayImage(width, height, gray)
    }
}
