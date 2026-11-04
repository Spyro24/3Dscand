package com.scand3d.capture

import android.util.Log
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Owns one capture session on disk: the JPEG frames plus a JSON manifest. Only relative paths
 * are written, never absolute ones (agents.md section 5, rule 9).
 */
class SessionStore(rootDirectory: File) {

    val sessionDirectory: File
    val manifestFile: File

    /** Directory of this session relative to the app's files directory, e.g. `sessions/20261104-131500`. */
    val relativeRoot: String

    private val frames = JSONArray()

    init {
        rootDirectory.mkdirs()
        val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        sessionDirectory = File(rootDirectory, stamp)
        File(sessionDirectory, FRAMES_DIRECTORY).mkdirs()
        manifestFile = File(sessionDirectory, MANIFEST_NAME)
        relativeRoot = "$SESSIONS_DIRECTORY/$stamp"
    }

    fun frameFile(index: Int): File =
        File(File(sessionDirectory, FRAMES_DIRECTORY), "frame_%04d.jpg".format(index))

    fun frameRelativePath(index: Int): String = "$relativeRoot/$FRAMES_DIRECTORY/frame_%04d.jpg".format(index)

    @Synchronized
    fun appendFrame(frame: FrameRecord) {
        frames.put(
            JSONObject()
                .put("timestamp_ns", frame.timestamp_ns)
                .put("exposure_ns", frame.exposure_ns)
                .put("iso", frame.iso)
                .put("focus_diopter", frame.focus_diopter.toDouble())
                .put("path", frame.relative_path ?: JSONObject.NULL),
        )
    }

    @Synchronized
    fun writeManifest(
        settings: LockedCaptureSettings?,
        deviceModel: String,
        savedFrames: Int,
    ) {
        val manifest = JSONObject()
            .put("schema", SCHEMA_VERSION)
            .put("device_model", deviceModel)
            .put("saved_frames", savedFrames)
            .put("frames", frames)

        if (settings != null) {
            manifest.put(
                "locked",
                JSONObject()
                    .put("exposure_ns", settings.exposure_ns)
                    .put("iso", settings.iso)
                    .put("focus_diopter", settings.focus_diopter.toDouble())
                    .put("awb_red", settings.awb_red.toDouble())
                    .put("awb_green_even", settings.awb_green_even.toDouble())
                    .put("awb_green_odd", settings.awb_green_odd.toDouble())
                    .put("awb_blue", settings.awb_blue.toDouble()),
            )
        }

        runCatching { manifestFile.writeText(manifest.toString(2)) }
            .onSuccess { Log.i(LOG_TAG, "manifest ${manifestFile.name}: ${frames.length()} frames") }
            .onFailure { Log.e(LOG_TAG, "manifest write failed", it) }
    }

    private companion object {
        const val LOG_TAG = "SessionStore"
        const val SCHEMA_VERSION = 1
        const val SESSIONS_DIRECTORY = "sessions"
        const val FRAMES_DIRECTORY = "frames"
        const val MANIFEST_NAME = "session.json"
    }
}
