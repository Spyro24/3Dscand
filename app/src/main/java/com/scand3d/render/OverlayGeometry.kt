package com.scand3d.render

/**
 * Turns the fused IMU orientation into the overlay camera, following the contract in D031: the
 * overlay camera is camera-to-world on the reconstruction frame, camera axes +X right, +Y down,
 * +Z forward (D012), and the frame is anchored at the pose the scan started from.
 *
 * Matrices are row-major 3x3 double arrays so they can be handed to the native projection without
 * a conversion.
 */
object OverlayGeometry {

    val IDENTITY_ROTATION = doubleArrayOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

    /**
     * Body-to-camera rotation: the camera axes expressed in the device's own frame (+X right,
     * +Y up, +Z out of the screen). The camera looks out of the back of the device, so its +Z is
     * body -Z and its +Y runs opposite body +Y. `displayRotationDeg` rotates the whole relation
     * about the viewing axis, because the preview is drawn in the display's orientation.
     */
    fun bodyFromCamera(frontFacing: Boolean, displayRotationDeg: Int): DoubleArray {
        val base = if (frontFacing) {
            // Front camera looks along +Z_body and its image is mirrored.
            doubleArrayOf(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0)
        } else {
            doubleArrayOf(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0)
        }
        val steps = ((displayRotationDeg % 360) + 360) % 360 / 90
        if (steps == 0) {
            return base
        }
        // Quarter turns about the body's viewing axis, one step per 90 degrees of display rotation.
        var turn = IDENTITY_ROTATION
        repeat(steps) {
            turn = multiply(ROTATE_Z_90, turn)
        }
        return multiply(turn, base)
    }

    /** Rotation matrix of a unit quaternion {w, x, y, z}; identity for unusable input. */
    fun rotationFromQuaternion(quaternion: DoubleArray): DoubleArray {
        if (quaternion.size < 4 || quaternion.any { !it.isFinite() }) {
            return IDENTITY_ROTATION.copyOf()
        }
        val w = quaternion[0]
        val x = quaternion[1]
        val y = quaternion[2]
        val z = quaternion[3]
        val norm = Math.sqrt(w * w + x * x + y * y + z * z)
        if (norm < 1e-12) {
            return IDENTITY_ROTATION.copyOf()
        }
        val nw = w / norm
        val nx = x / norm
        val ny = y / norm
        val nz = z / norm
        return doubleArrayOf(
            1.0 - 2.0 * (ny * ny + nz * nz), 2.0 * (nx * ny - nz * nw), 2.0 * (nx * nz + ny * nw),
            2.0 * (nx * ny + nz * nw), 1.0 - 2.0 * (nx * nx + nz * nz), 2.0 * (ny * nz - nx * nw),
            2.0 * (nx * nz - ny * nw), 2.0 * (ny * nz + nx * nw), 1.0 - 2.0 * (nx * nx + ny * ny),
        )
    }

    /**
     * Camera-to-world rotation for the overlay: the live body orientation relative to the pose the
     * overlay was anchored at, applied to the camera axes. When the reference equals the live
     * orientation the result is just the camera's rest pose in the anchored frame, so a cloud does
     * not jump when the overlay is switched on.
     */
    fun overlayRotation(
        referenceQuaternion: DoubleArray,
        liveQuaternion: DoubleArray,
        bodyFromCamera: DoubleArray,
    ): DoubleArray {
        val reference = rotationFromQuaternion(referenceQuaternion)
        val live = rotationFromQuaternion(liveQuaternion)
        // R_world_from_camera = R_reference^T * R_live * R_body_from_camera.
        return multiply(transpose(reference), multiply(live, bodyFromCamera))
    }

    /** Row-major 3x3 multiply. */
    fun multiply(a: DoubleArray, b: DoubleArray): DoubleArray {
        val out = DoubleArray(9)
        for (row in 0 until 3) {
            for (column in 0 until 3) {
                var sum = 0.0
                for (k in 0 until 3) {
                    sum += a[row * 3 + k] * b[k * 3 + column]
                }
                out[row * 3 + column] = sum
            }
        }
        return out
    }

    /** Row-major 3x3 applied to a 3-vector. */
    fun applyToVector(matrix: DoubleArray, vector: DoubleArray): DoubleArray {
        require(matrix.size == 9 && vector.size == 3) { "expected a 3x3 matrix and a 3-vector" }
        return doubleArrayOf(
            matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
            matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
            matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
        )
    }

    /** Row-major 3x3 transpose. */
    fun transpose(a: DoubleArray): DoubleArray = doubleArrayOf(
        a[0], a[3], a[6],
        a[1], a[4], a[7],
        a[2], a[5], a[8],
    )

    // Quarter turn about the device's viewing axis, used to follow the display orientation.
    private val ROTATE_Z_90 = doubleArrayOf(
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    )
}
