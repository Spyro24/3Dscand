package com.scand3d.render

import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * The tracked pose is in the map's free-scale frame, while the cloud on screen has been rotated by
 * the anchor basis and rescaled into metres (D051). These cases pin that mapping: if it were wrong
 * the cloud would still draw, just somewhere other than on the object.
 */
class OverlayTrackingTest {

    /** The native projector's relation: p_cam = R^T (p_world - C), then a pinhole. */
    private fun project(
        rotation: DoubleArray,
        centre_m: DoubleArray,
        focal_px: Double,
        point_world_m: DoubleArray,
    ): Pair<Double, Double> {
        val delta = doubleArrayOf(
            point_world_m[0] - centre_m[0],
            point_world_m[1] - centre_m[1],
            point_world_m[2] - centre_m[2],
        )
        val camera = OverlayGeometry.applyToVector(OverlayGeometry.transpose(rotation), delta)
        return focal_px * camera[0] / camera[2] to focal_px * camera[1] / camera[2]
    }

    private fun scaled(vector: DoubleArray, scale: Double): DoubleArray =
        doubleArrayOf(vector[0] * scale, vector[1] * scale, vector[2] * scale)

    @Test
    fun `tracked pose projects a map point where the map camera does`() {
        val basis = OverlayGeometry.bodyFromCamera(frontFacing = false, displayRotationDeg = 90)
        val scale = 0.42
        val turn = Math.toRadians(30.0)
        val map_rotation = doubleArrayOf(
            Math.cos(turn), 0.0, Math.sin(turn),
            0.0, 1.0, 0.0,
            -Math.sin(turn), 0.0, Math.cos(turn),
        )
        val map_centre = doubleArrayOf(0.1, -0.2, 1.0)
        val point_map = doubleArrayOf(0.05, 0.02, 1.3)

        // The three steps the view model applies to a tracked pose.
        val overlay_rotation = OverlayGeometry.multiply(basis, map_rotation)
        val overlay_centre = scaled(OverlayGeometry.applyToVector(basis, map_centre), scale)
        val point_overlay = scaled(OverlayGeometry.applyToVector(basis, point_map), scale)

        val map_px = project(map_rotation, map_centre, 600.0, point_map)
        val overlay_px = project(overlay_rotation, overlay_centre, 600.0, point_overlay)

        assertEquals(map_px.first, overlay_px.first, 1e-9)
        assertEquals(map_px.second, overlay_px.second, 1e-9)
    }

    @Test
    fun `an identity basis and unit scale pass the pose through unchanged`() {
        val rotation = OverlayGeometry.rotationFromQuaternion(doubleArrayOf(1.0, 0.0, 0.0, 0.0))
        val centre = doubleArrayOf(0.3, 0.4, -1.2)
        val point = doubleArrayOf(0.0, 0.0, -2.0)

        val direct = project(rotation, centre, 500.0, point)
        val mapped = project(
            OverlayGeometry.multiply(OverlayGeometry.IDENTITY_ROTATION, rotation),
            scaled(OverlayGeometry.applyToVector(OverlayGeometry.IDENTITY_ROTATION, centre), 1.0),
            500.0,
            scaled(OverlayGeometry.applyToVector(OverlayGeometry.IDENTITY_ROTATION, point), 1.0),
        )

        assertEquals(direct.first, mapped.first, 1e-9)
        assertEquals(direct.second, mapped.second, 1e-9)
    }
}
