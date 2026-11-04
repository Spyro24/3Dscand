package com.scand3d.reconstruct

import android.util.Log
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Writes the live sparse map as a binary little-endian PLY next to the session manifest, where the
 * Blender add-on in `tools/blender/` polls it over adb (D044).
 *
 * The points stay in the reconstruction's own frame: the first keyframe's camera frame (x right,
 * y down, z forward) with free scale (D028). The add-on converts the axes and normalises the size,
 * so nothing about one particular viewer leaks into the file.
 */
object CloudPlyWriter {

    const val FILE_NAME = "cloud.ply"

    private const val LOG_TAG = "CloudPly"

    /**
     * Replaces `file` with the cloud in `positions_m` (three doubles per point, count = size / 3).
     *
     * The points are written to a temporary file that is renamed over the target: the reader polls
     * at any moment, and a half-written body would parse as a valid PLY with a wrong point count.
     */
    fun write(file: File, positions_m: DoubleArray): Boolean {
        val count = positions_m.size / 3
        val directory = file.parentFile ?: return false
        val temporary = File(directory, file.name + ".tmp")
        return try {
            directory.mkdirs()
            FileOutputStream(temporary).use { out ->
                out.write(header(count).toByteArray(Charsets.US_ASCII))
                if (count > 0) {
                    val body = ByteBuffer.allocate(count * 3 * Float.SIZE_BYTES)
                        .order(ByteOrder.LITTLE_ENDIAN)
                    for (index in 0 until count * 3) {
                        body.putFloat(positions_m[index].toFloat())
                    }
                    out.write(body.array())
                }
            }
            if (temporary.renameTo(file)) {
                true
            } else {
                temporary.delete()
                false
            }
        } catch (t: Throwable) {
            Log.w(LOG_TAG, "write failed: ${file.name}", t)
            temporary.delete()
            false
        }
    }

    private fun header(count: Int): String = buildString {
        append("ply\n")
        append("format binary_little_endian 1.0\n")
        append("comment 3Dscand live sparse map, first keyframe camera frame, scale free\n")
        append("element vertex ").append(count).append('\n')
        append("property float x\n")
        append("property float y\n")
        append("property float z\n")
        append("end_header\n")
    }
}
