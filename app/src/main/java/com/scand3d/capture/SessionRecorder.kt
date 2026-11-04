package com.scand3d.capture

/** Measured timing and IMU facts for one session, written into the manifest (schema 2). */
data class SessionMetadata(
    val camera_clock_source: String,
    val sensor_clock: String,
    val clock_offset_ns: Long?,
    val clock_offset_mad_ns: Long?,
    val clock_samples: Int,
    val imu_mode: String?,
    val imu_reduced_accuracy: Boolean,
    val keyframe_candidates: Int,
)

/**
 * The seam between capture and the sensor layer: the controller hands every saved frame through
 * here before it reaches the session store, and asks for the metadata when it writes the manifest.
 */
interface SessionRecorder {
    fun annotate(frame: FrameRecord): FrameRecord

    fun metadata(camera_clock_source: String): SessionMetadata
}
