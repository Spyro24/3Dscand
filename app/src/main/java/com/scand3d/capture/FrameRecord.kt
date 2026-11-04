package com.scand3d.capture

/**
 * One captured frame: the camera clock reading plus the photometric settings it was taken with.
 * `relative_path` is null for preview-only frames that were never written to disk.
 */
data class FrameRecord(
    val timestamp_ns: Long,
    val exposure_ns: Long,
    val iso: Int,
    val focus_diopter: Float,
    val relative_path: String?,
)
