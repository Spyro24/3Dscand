package com.scand3d.core.bridge

/**
 * The only place Kotlin reaches the native core (agents.md section 3). Loading happens once,
 * when this object is first touched, and the failure is kept instead of thrown so the UI can
 * report a broken native build rather than crashing on launch.
 *
 * Every call below degrades to a safe value when the library is absent, so host unit tests and a
 * damaged APK take the same path.
 */
object NativeCore {

    private const val LIBRARY_NAME = "scand3d_core"
    private const val QUATERNION_SIZE = 4

    /** Null when the shared library loaded successfully. */
    val loadError: Throwable? = try {
        System.loadLibrary(LIBRARY_NAME)
        null
    } catch (t: UnsatisfiedLinkError) {
        t
    }

    val isLoaded: Boolean
        get() = loadError == null

    /**
     * Version reported by the native core, or null when the library did not load. Calling this
     * is what proves the bridge works end to end.
     */
    fun coreVersionOrNull(): String? = if (isLoaded) nativeCoreVersion() else null

    /** Opens a fusion session; 0 when the native core is unavailable. */
    fun imuCreate(sample_rate_hz: Double): Long =
        if (isLoaded) nativeImuCreate(sample_rate_hz) else 0L

    fun imuUpdate(handle: Long, values: DoubleArray): Boolean =
        isLoaded && handle != 0L && nativeImuUpdate(handle, values)

    fun imuOrientation(handle: Long, out: DoubleArray): Boolean =
        isLoaded && handle != 0L && out.size >= QUATERNION_SIZE && nativeImuOrientation(handle, out)

    fun imuReset(handle: Long) {
        if (isLoaded && handle != 0L) nativeImuReset(handle)
    }

    fun imuDestroy(handle: Long) {
        if (isLoaded && handle != 0L) nativeImuDestroy(handle)
    }

    fun angularDisplacementRad(a: List<Double>, b: List<Double>): Double =
        if (isLoaded && a.size == QUATERNION_SIZE && b.size == QUATERNION_SIZE) {
            nativeAngularDisplacementRad(a.toDoubleArray(), b.toDoubleArray())
        } else {
            Double.NaN
        }

    fun keyframeBaselineAccept(
        reference: List<Double>,
        candidate: List<Double>,
        min_baseline_rad: Double,
        max_baseline_rad: Double,
    ): Boolean =
        isLoaded &&
            reference.size == QUATERNION_SIZE &&
            candidate.size == QUATERNION_SIZE &&
            nativeKeyframeBaselineAccept(
                reference.toDoubleArray(),
                candidate.toDoubleArray(),
                min_baseline_rad,
                max_baseline_rad,
            )

    /** Opens a reconstruction session; 0 when the native core is unavailable. */
    fun reconstructionCreate(): Long = if (isLoaded) nativeReconstructionCreate() else 0L

    fun reconstructionDestroy(handle: Long) {
        if (isLoaded && handle != 0L) nativeReconstructionDestroy(handle)
    }

    /** Adds one 8-bit gray view; returns its index, or -1. */
    fun reconstructionAddView(
        handle: Long,
        gray: ByteArray,
        width_px: Int,
        height_px: Int,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
    ): Int =
        if (isLoaded && handle != 0L && width_px > 0 && height_px > 0 &&
            gray.size == width_px * height_px
        ) {
            nativeReconstructionAddView(handle, gray, width_px, height_px, focal_px, cx_px, cy_px)
        } else {
            -1
        }

    /** Runs the reconstruction; returns the number of registered views. */
    fun reconstructionRun(handle: Long): Int =
        if (isLoaded && handle != 0L) nativeReconstructionRun(handle) else 0

    fun reconstructionPointCount(handle: Long): Int =
        if (isLoaded && handle != 0L) nativeReconstructionPointCount(handle) else 0

    /** Fills `out_positions` with xyz triples; returns how many points were written. */
    fun reconstructionCopyPoints(handle: Long, out_positions: DoubleArray): Int =
        if (isLoaded && handle != 0L && out_positions.size >= 3) {
            nativeReconstructionCopyPoints(handle, out_positions)
        } else {
            0
        }

    /** Tracks a gray frame against the map; returns inliers, 0 when it did not register. */
    fun reconstructionTrackPose(
        handle: Long,
        gray: ByteArray,
        width_px: Int,
        height_px: Int,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        out_rotation: DoubleArray,
        out_centre_m: DoubleArray,
    ): Int =
        if (isLoaded && handle != 0L && width_px > 0 && height_px > 0 &&
            gray.size == width_px * height_px && out_rotation.size == 9 && out_centre_m.size == 3
        ) {
            nativeReconstructionTrackPose(
                handle,
                gray,
                width_px,
                height_px,
                focal_px,
                cx_px,
                cy_px,
                out_rotation,
                out_centre_m,
            )
        } else {
            0
        }

    /** Copies one placed view's pose after a run; false when that view was not registered. */
    fun reconstructionCopyCamera(
        handle: Long,
        index: Int,
        out_rotation: DoubleArray,
        out_centre_m: DoubleArray,
    ): Boolean =
        isLoaded &&
            handle != 0L &&
            index >= 0 &&
            out_rotation.size == 9 &&
            out_centre_m.size == 3 &&
            nativeReconstructionCopyCamera(handle, index, out_rotation, out_centre_m)

    /**
     * Dense stage (roadmap 6): posed gray views in, fused cloud out. [denseRun] blocks for as long
     * as the pass takes; [denseProgress] and [denseCancel] answer from another thread.
     */
    fun denseCreate(): Long = if (isLoaded) nativeDenseCreate() else 0L

    fun denseDestroy(handle: Long) {
        if (isLoaded && handle != 0L) nativeDenseDestroy(handle)
    }

    /** Adds one posed view; returns its index, or -1. */
    fun denseAddView(
        handle: Long,
        gray: ByteArray,
        width_px: Int,
        height_px: Int,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        rotation: DoubleArray,
        centre_m: DoubleArray,
    ): Int =
        if (isLoaded && handle != 0L && width_px > 0 && height_px > 0 &&
            gray.size == width_px * height_px && rotation.size == 9 && centre_m.size == 3
        ) {
            nativeDenseAddView(
                handle,
                gray,
                width_px,
                height_px,
                focal_px,
                cx_px,
                cy_px,
                rotation,
                centre_m,
            )
        } else {
            -1
        }

    /** Sparse points that narrow the depth search; they and the view poses share one frame. */
    fun denseSetHints(handle: Long, positions_m: DoubleArray) {
        if (isLoaded && handle != 0L && positions_m.size % 3 == 0) {
            nativeDenseSetHints(handle, positions_m)
        }
    }

    /** Runs the dense stage; returns the fused point count, 0 when cancelled. Blocks. */
    fun denseRun(
        handle: Long,
        working_width_px: Int,
        min_depth_m: Double,
        max_depth_m: Double,
    ): Int =
        if (isLoaded && handle != 0L && working_width_px > 0) {
            nativeDenseRun(handle, working_width_px, min_depth_m, max_depth_m)
        } else {
            0
        }

    fun densePointCount(handle: Long): Int =
        if (isLoaded && handle != 0L) nativeDensePointCount(handle) else 0

    /** Fills `out_positions` with xyz triples; returns how many points were written. */
    fun denseCopyPoints(handle: Long, out_positions: DoubleArray): Int =
        if (isLoaded && handle != 0L && out_positions.size >= 3) {
            nativeDenseCopyPoints(handle, out_positions)
        } else {
            0
        }

    fun denseProgress(handle: Long): Double =
        if (isLoaded && handle != 0L) nativeDenseProgress(handle) else 0.0

    fun denseCancel(handle: Long) {
        if (isLoaded && handle != 0L) nativeDenseCancel(handle)
    }

    /** Floats per overlay vertex: {x_ndc, y_ndc, r, g, b}; mirrors kOverlayVertexValueCount. */
    const val OVERLAY_VERTEX_VALUE_COUNT = 5

    /** Opens an empty overlay scene; 0 when the native core is unavailable. */
    fun overlayCreate(): Long = if (isLoaded) nativeOverlayCreate() else 0L

    fun overlayDestroy(handle: Long) {
        if (isLoaded && handle != 0L) nativeOverlayDestroy(handle)
    }

    /** Replaces the scene: xyz per point in meters, then 0..1 rgb per point. */
    fun overlaySetPoints(handle: Long, positions_m: DoubleArray, rgb: FloatArray): Boolean =
        isLoaded &&
            handle != 0L &&
            positions_m.size % 3 == 0 &&
            rgb.size == positions_m.size &&
            nativeOverlaySetPoints(handle, positions_m, rgb)

    /** Writes visible vertices into `out` (multiples of [OVERLAY_VERTEX_VALUE_COUNT]). */
    fun overlayProject(
        handle: Long,
        rotation: DoubleArray,
        centre_m: DoubleArray,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        image_width_px: Int,
        image_height_px: Int,
        out: FloatArray,
    ): Int =
        if (isLoaded && handle != 0L && rotation.size == 9 && centre_m.size == 3) {
            nativeOverlayProject(
                handle,
                rotation,
                centre_m,
                focal_px,
                cx_px,
                cy_px,
                image_width_px,
                image_height_px,
                out,
            )
        } else {
            0
        }

    /**
     * Re-samples every point's colour from a keyframe (roadmap 5.3). `rgb` is packed 3 bytes per
     * pixel at `keyframe_width_px` x `keyframe_height_px`, in the camera's pixel space.
     */
    fun overlaySampleColors(
        handle: Long,
        rotation: DoubleArray,
        centre_m: DoubleArray,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        image_width_px: Int,
        image_height_px: Int,
        rgb: ByteArray,
        keyframe_width_px: Int,
        keyframe_height_px: Int,
    ): Int =
        if (isLoaded && handle != 0L && rotation.size == 9 && centre_m.size == 3) {
            nativeOverlaySampleColors(
                handle,
                rotation,
                centre_m,
                focal_px,
                cx_px,
                cy_px,
                image_width_px,
                image_height_px,
                rgb,
                keyframe_width_px,
                keyframe_height_px,
            )
        } else {
            0
        }

    private external fun nativeCoreVersion(): String

    private external fun nativeImuCreate(sample_rate_hz: Double): Long

    private external fun nativeImuDestroy(handle: Long)

    private external fun nativeImuReset(handle: Long)

    private external fun nativeImuUpdate(handle: Long, values: DoubleArray): Boolean

    private external fun nativeImuOrientation(handle: Long, out: DoubleArray): Boolean

    private external fun nativeAngularDisplacementRad(a: DoubleArray, b: DoubleArray): Double

    private external fun nativeKeyframeBaselineAccept(
        reference: DoubleArray,
        candidate: DoubleArray,
        min_baseline_rad: Double,
        max_baseline_rad: Double,
    ): Boolean

    private external fun nativeReconstructionCreate(): Long

    private external fun nativeReconstructionDestroy(handle: Long)

    private external fun nativeReconstructionAddView(
        handle: Long,
        gray: ByteArray,
        width_px: Int,
        height_px: Int,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
    ): Int

    private external fun nativeReconstructionRun(handle: Long): Int

    private external fun nativeReconstructionPointCount(handle: Long): Int

    private external fun nativeReconstructionCopyPoints(
        handle: Long,
        out_positions: DoubleArray,
    ): Int

    private external fun nativeReconstructionTrackPose(
        handle: Long,
        gray: ByteArray,
        width_px: Int,
        height_px: Int,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        out_rotation: DoubleArray,
        out_centre_m: DoubleArray,
    ): Int

    private external fun nativeOverlayCreate(): Long

    private external fun nativeOverlayDestroy(handle: Long)

    private external fun nativeOverlaySetPoints(
        handle: Long,
        positions_m: DoubleArray,
        rgb: FloatArray,
    ): Boolean

    private external fun nativeOverlaySampleColors(
        handle: Long,
        rotation: DoubleArray,
        centre_m: DoubleArray,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        image_width_px: Int,
        image_height_px: Int,
        rgb: ByteArray,
        keyframe_width_px: Int,
        keyframe_height_px: Int,
    ): Int

    private external fun nativeOverlayProject(
        handle: Long,
        rotation: DoubleArray,
        centre_m: DoubleArray,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        image_width_px: Int,
        image_height_px: Int,
        out: FloatArray,
    ): Int

    private external fun nativeReconstructionCopyCamera(
        handle: Long,
        index: Int,
        out_rotation: DoubleArray,
        out_centre_m: DoubleArray,
    ): Boolean

    private external fun nativeDenseCreate(): Long

    private external fun nativeDenseDestroy(handle: Long)

    private external fun nativeDenseAddView(
        handle: Long,
        gray: ByteArray,
        width_px: Int,
        height_px: Int,
        focal_px: Double,
        cx_px: Double,
        cy_px: Double,
        rotation: DoubleArray,
        centre_m: DoubleArray,
    ): Int

    private external fun nativeDenseSetHints(handle: Long, positions_m: DoubleArray)

    private external fun nativeDenseRun(
        handle: Long,
        working_width_px: Int,
        min_depth_m: Double,
        max_depth_m: Double,
    ): Int

    private external fun nativeDensePointCount(handle: Long): Int

    private external fun nativeDenseCopyPoints(handle: Long, out_positions: DoubleArray): Int

    private external fun nativeDenseProgress(handle: Long): Double

    private external fun nativeDenseCancel(handle: Long)
}
