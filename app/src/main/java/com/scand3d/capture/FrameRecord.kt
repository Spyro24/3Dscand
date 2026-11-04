package com.scand3d.capture

/** A frame plus the IMU and timing facts gathered when it was saved. */
data class FrameRecord(
    val timestamp_ns: Long,
    val exposure_ns: Long,
    val iso: Int,
    val focus_diopter: Float,
    val relative_path: String?,
    /** Fused orientation at the frame timestamp, {w, x, y, z}; null when no IMU was available. */
    val orientation_wxyz: List<Double>? = null,
    /** Sensor timestamp of the sample this orientation came from. */
    val imu_timestamp_ns: Long? = null,
    /** Measured sensor-minus-frame residual for this pairing. */
    val imu_time_offset_ns: Long? = null,
    /** IMU keyframe prior (roadmap 2.6); the final decision belongs to Phase 4. */
    val keyframe_candidate: Boolean = false,
)
