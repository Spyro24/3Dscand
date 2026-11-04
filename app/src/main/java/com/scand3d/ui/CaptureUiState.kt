package com.scand3d.ui

import com.scand3d.capture.LockedCaptureSettings
import com.scand3d.reconstruct.MvsQuality
import com.scand3d.sensors.ImuMode
import com.scand3d.sensors.MotionWarning

/** The five app states from agents.md section 4; Phase 1 exercises the first four. */
enum class CapturePhase {
    WaitingForPermission,
    PermissionDenied,
    Starting,
    Preview,
    Locked,
    Capturing,
    CameraLost,
    Failed,
}

data class CaptureUiState(
    val phase: CapturePhase = CapturePhase.WaitingForPermission,
    val previewWidthPx: Int = 0,
    val previewHeightPx: Int = 0,
    /** Clockwise quarter turns that make the sensor image upright (D040). */
    val previewRotationDeg: Int = 0,
    val fps: Float = 0f,
    val locked: LockedCaptureSettings? = null,
    val savedFrames: Int = 0,

    /** The saved frames are too dark for the feature detector; the scan needs more light (D052). */
    val low_light: Boolean = false,
    val sessionRelativePath: String? = null,
    val message: String? = null,
    /** Null until the sensor layer has reported what the device can do. */
    val imu_mode: ImuMode? = null,
    val imu_reduced_accuracy: Boolean = false,
    val motion: MotionWarning = MotionWarning.Steady,
    val keyframe_candidates: Int = 0,
    /** Overlay toggle and opacity (roadmap 5.4). */
    val overlayEnabled: Boolean = true,
    val overlayOpacity: Float = 1.0f,
    /** Manual capture settings (D059); 0 ns / negative dioptres mean the locked auto value. */
    val manualExposureNs: Long = 0L,
    val manualFocusDiopter: Float = -1f,
    /** Draw one cloud per keyframe instead of one merged, photo-coloured cloud (D059). */
    val keyframeClouds: Boolean = true,
    /** Dense stage (roadmap 6): whether a finished capture gets a dense pass, at which preset, and
     * its progress while it runs (null when idle). */
    val denseEnabled: Boolean = false,
    val denseQuality: MvsQuality = MvsQuality.Normal,
    val dense_progress: Float? = null,
    val dense_points: Int = 0,
)
