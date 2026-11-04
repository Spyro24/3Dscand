package com.scand3d.render

/**
 * One overlay camera: camera-to-world rotation (row-major 3x3), centre, and a pinhole model in
 * displayed-image pixels (D031). Reused by the projection and by keyframe colour sampling, which
 * scales it to the resolution of the keyframe it samples.
 */
class OverlayCameraParams(
    val rotation: DoubleArray,
    val centre_m: DoubleArray,
    val focal_px: Double,
    val cx_px: Double,
    val cy_px: Double,
    val image_width_px: Int,
    val image_height_px: Int,
) {

    /** Same camera expressed at another image size, for a keyframe of a different resolution. */
    fun scaledTo(width_px: Int, height_px: Int): OverlayCameraParams {
        if (width_px <= 0 || height_px <= 0 || image_width_px <= 0) {
            return this
        }
        val scale_x = width_px.toDouble() / image_width_px
        val scale_y = height_px.toDouble() / image_height_px
        return OverlayCameraParams(
            rotation = rotation,
            centre_m = centre_m,
            focal_px = focal_px * 0.5 * (scale_x + scale_y),
            cx_px = cx_px * scale_x,
            cy_px = cy_px * scale_y,
            image_width_px = width_px,
            image_height_px = height_px,
        )
    }
}
