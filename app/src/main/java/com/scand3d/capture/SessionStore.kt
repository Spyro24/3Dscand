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
        val entry = JSONObject()
            .put("timestamp_ns", frame.timestamp_ns)
            .put("exposure_ns", frame.exposure_ns)
            .put("iso", frame.iso)
            .put("focus_diopter", frame.focus_diopter.toDouble())
            .put("path", frame.relative_path ?: JSONObject.NULL)
            .put("keyframe_candidate", frame.keyframe_candidate)

        // Only frames that actually met a sensor sample carry an orientation (roadmap 2.3).
        frame.orientation_wxyz?.let { quaternion ->
            entry.put("orientation_wxyz", JSONArray(quaternion))
            frame.imu_timestamp_ns?.let { entry.put("imu_timestamp_ns", it) }
            frame.imu_time_offset_ns?.let { entry.put("imu_time_offset_ns", it) }
        }

        frames.put(entry)
    }

    @Synchronized
    fun writeManifest(
        settings: LockedCaptureSettings?,
        deviceModel: String,
        metadata: SessionMetadata? = null,
    ) {
        val manifest = JSONObject()
            .put("schema", SCHEMA_VERSION)
            .put("device_model", deviceModel)
            // Derived from the entries themselves: a separate counter drifts when a still that was
            // already in flight lands after capture stopped, leaving a file the manifest omits.
            .put("saved_frames", frames.length())
            .put("frames", frames)

        // Sensor timing and IMU facts for this session; Phase 4 reads the offset when it pairs
        // poses, Phase 5 uses it for the live overlay.
        if (metadata != null) {
            manifest.put(
                "clock",
                JSONObject()
                    .put("camera_source", metadata.camera_clock_source)
                    .put("sensor_source", metadata.sensor_clock)
                    .put("offset_ns", metadata.clock_offset_ns ?: JSONObject.NULL)
                    .put("offset_mad_ns", metadata.clock_offset_mad_ns ?: JSONObject.NULL)
                    .put("samples", metadata.clock_samples),
            )
            manifest.put(
                "imu",
                JSONObject()
                    .put("mode", metadata.imu_mode ?: JSONObject.NULL)
                    .put("reduced_accuracy", metadata.imu_reduced_accuracy)
                    .put("keyframe_candidates", metadata.keyframe_candidates),
            )
        }

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
        // Schema 2 adds the per-frame IMU snapshot and the clock/imu blocks (DECISIONS.md D019).
        const val SCHEMA_VERSION = 2
        const val SESSIONS_DIRECTORY = "sessions"
        const val FRAMES_DIRECTORY = "frames"
        const val MANIFEST_NAME = "session.json"
    }
}
