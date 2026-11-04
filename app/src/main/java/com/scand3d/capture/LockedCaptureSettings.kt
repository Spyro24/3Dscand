package com.scand3d.capture

/**
 * Photometric settings frozen after warm-up (roadmap 1.2). A reconstruction session needs one
 * exposure, gain, white balance and focus, so these values are held for the whole capture and
 * written into the session manifest.
 */
data class LockedCaptureSettings(
    val exposure_ns: Long,
    val iso: Int,
    val focus_diopter: Float,
    val awb_red: Float,
    val awb_green_even: Float,
    val awb_green_odd: Float,
    val awb_blue: Float,
)
