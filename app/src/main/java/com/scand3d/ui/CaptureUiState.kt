package com.scand3d.ui

import com.scand3d.capture.LockedCaptureSettings

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
    val fps: Float = 0f,
    val locked: LockedCaptureSettings? = null,
    val savedFrames: Int = 0,
    val sessionRelativePath: String? = null,
    val message: String? = null,
)
