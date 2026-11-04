package com.scand3d.render

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class OverlayGeometryTest {

    private fun assertMatrix(expected: DoubleArray, actual: DoubleArray, tolerance: Double = 1e-9) {
        assertEquals(expected.size, actual.size)
        for (index in expected.indices) {
            assertEquals("element $index", expected[index], actual[index], tolerance)
        }
    }

    @Test
    fun identityQuaternionGivesIdentityRotation() {
        assertMatrix(
            OverlayGeometry.IDENTITY_ROTATION,
            OverlayGeometry.rotationFromQuaternion(doubleArrayOf(1.0, 0.0, 0.0, 0.0)),
        )
    }

    @Test
    fun quaternionIsNormalisedBeforeUse() {
        assertMatrix(
            OverlayGeometry.IDENTITY_ROTATION,
            OverlayGeometry.rotationFromQuaternion(doubleArrayOf(2.0, 0.0, 0.0, 0.0)),
        )
    }

    @Test
    fun quarterTurnAboutZIsAQuarterTurn() {
        val half = Math.sin(Math.PI / 4.0)
        val rotation = OverlayGeometry.rotationFromQuaternion(
            doubleArrayOf(half, 0.0, 0.0, half),
        )
        // A quarter turn about +Z maps body +X onto +Y.
        val x = OverlayGeometry.applyToVector(rotation, doubleArrayOf(1.0, 0.0, 0.0))
        assertArrayEquals(doubleArrayOf(0.0, 1.0, 0.0), x, 1e-9)
    }

    @Test
    fun unusableQuaternionFallsBackToIdentity() {
        assertMatrix(
            OverlayGeometry.IDENTITY_ROTATION,
            OverlayGeometry.rotationFromQuaternion(doubleArrayOf(Double.NaN, 0.0, 0.0, 0.0)),
        )
        assertMatrix(
            OverlayGeometry.IDENTITY_ROTATION,
            OverlayGeometry.rotationFromQuaternion(doubleArrayOf(0.0, 0.0, 0.0, 0.0)),
        )
        assertMatrix(
            OverlayGeometry.IDENTITY_ROTATION,
            OverlayGeometry.rotationFromQuaternion(doubleArrayOf(1.0, 0.0)),
        )
    }

    @Test
    fun backCameraLooksOutOfTheBackOfTheDevice() {
        val bodyFromCamera = OverlayGeometry.bodyFromCamera(
            frontFacing = false,
            displayRotationDeg = 0,
        )
        // Camera forward (+Z) is body -Z, camera +X is body +X.
        val forward = OverlayGeometry.applyToVector(bodyFromCamera, doubleArrayOf(0.0, 0.0, 1.0))
        assertArrayEquals(doubleArrayOf(0.0, 0.0, -1.0), forward, 1e-9)
        val right = OverlayGeometry.applyToVector(bodyFromCamera, doubleArrayOf(1.0, 0.0, 0.0))
        assertArrayEquals(doubleArrayOf(1.0, 0.0, 0.0), right, 1e-9)
    }

    @Test
    fun displayRotationTurnsTheCameraAboutTheViewingAxis() {
        val turned = OverlayGeometry.bodyFromCamera(frontFacing = false, displayRotationDeg = 90)
        val forward = OverlayGeometry.applyToVector(turned, doubleArrayOf(0.0, 0.0, 1.0))
        // A quarter turn must leave the optical axis alone: it rotates the roll about the view.
        assertArrayEquals(doubleArrayOf(0.0, 0.0, -1.0), forward, 1e-9)
        // One step is the turn the display reports for ROTATION_90, which rolls the image axes
        // one quarter turn; the physical sign still has to be confirmed against the preview.
        val right = OverlayGeometry.applyToVector(turned, doubleArrayOf(1.0, 0.0, 0.0))
        assertArrayEquals(doubleArrayOf(0.0, 1.0, 0.0), right, 1e-9)
    }

    @Test
    fun livePoseEqualToReferenceLeavesTheRestPose() {
        val quaternion = doubleArrayOf(0.5, 0.5, 0.5, 0.5)
        val bodyFromCamera = OverlayGeometry.bodyFromCamera(false, 0)

        val rotation = OverlayGeometry.overlayRotation(quaternion, quaternion, bodyFromCamera)

        assertMatrix(bodyFromCamera, rotation, 1e-9)
    }

    @Test
    fun rotatingThePhoneTurnsTheOverlayCamera() {
        val bodyFromCamera = OverlayGeometry.bodyFromCamera(false, 0)
        // Live pose is 90 degrees of yaw away from the reference about the world vertical.
        val reference = doubleArrayOf(1.0, 0.0, 0.0, 0.0)
        val live = doubleArrayOf(Math.cos(Math.PI / 4.0), 0.0, 0.0, Math.sin(Math.PI / 4.0))

        val rotation = OverlayGeometry.overlayRotation(reference, live, bodyFromCamera)

        // Yaw is about the world vertical, so the optical axis stays put and the image axes turn.
        val forward = OverlayGeometry.applyToVector(rotation, doubleArrayOf(0.0, 0.0, 1.0))
        assertArrayEquals(doubleArrayOf(0.0, 0.0, -1.0), forward, 1e-9)
        val right = OverlayGeometry.applyToVector(rotation, doubleArrayOf(1.0, 0.0, 0.0))
        assertArrayEquals(doubleArrayOf(0.0, 1.0, 0.0), right, 1e-9)
    }

    @Test
    fun compositionStaysOrthonormal() {
        val quaternion = doubleArrayOf(0.36, -0.48, 0.72, 0.36)
        val rotation = OverlayGeometry.overlayRotation(
            doubleArrayOf(1.0, 0.0, 0.0, 0.0),
            quaternion,
            OverlayGeometry.bodyFromCamera(false, 180),
        )
        for (row in 0 until 3) {
            for (column in 0 until 3) {
                var dot = 0.0
                for (k in 0 until 3) {
                    dot += rotation[row * 3 + k] * rotation[column * 3 + k]
                }
                assertEquals("row $row column $column", if (row == column) 1.0 else 0.0, dot, 1e-9)
            }
        }
    }
}
