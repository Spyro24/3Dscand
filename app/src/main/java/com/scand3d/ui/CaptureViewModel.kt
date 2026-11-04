package com.scand3d.ui

import android.app.Application
import android.graphics.SurfaceTexture
import android.os.Build
import android.util.Log
import android.view.Surface
import androidx.lifecycle.AndroidViewModel
import com.scand3d.benchmark.BenchmarkRecorder
import com.scand3d.capture.CameraCaptureController
import com.scand3d.capture.FrameRecord
import com.scand3d.capture.LockedCaptureSettings
import com.scand3d.capture.SessionStore
import com.scand3d.debug.CaptureTestBridge
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.File

/**
 * Owns the camera controller and turns camera callbacks into UI state. Callbacks arrive on the
 * camera thread; `StateFlow` is the only thing that crosses to the UI thread.
 */
class CaptureViewModel(application: Application) :
    AndroidViewModel(application),
    CameraCaptureController.Listener,
    CaptureTestBridge.Actions {

    private val _state = MutableStateFlow(CaptureUiState())
    val state: StateFlow<CaptureUiState> = _state.asStateFlow()

    private var controller: CameraCaptureController? = null
    private var benchmark: BenchmarkRecorder? = null
    private var savedFrameIndex = 0
    private var memoryTick = 0

    init {
        CaptureTestBridge.actions = this
    }

    fun onPermissionGranted() {
        Log.i(LOG_TAG, "onPermissionGranted controller=${controller != null} phase=${_state.value.phase}")
        if (controller != null) {
            return
        }
        val created = CameraCaptureController(getApplication(), this)
        controller = created
        val error = created.initialisationError
        if (error != null) {
            _state.value = _state.value.copy(phase = CapturePhase.Failed, message = error)
            return
        }
        _state.value = _state.value.copy(
            phase = CapturePhase.Starting,
            previewWidthPx = created.previewSizePx.width,
            previewHeightPx = created.previewSizePx.height,
        )
    }

    fun onPermissionDenied() {
        _state.value = _state.value.copy(phase = CapturePhase.PermissionDenied)
    }

    fun onPreviewTextureReady(texture: SurfaceTexture) {
        val controller = controller ?: return
        // A surface can arrive while the app is showing a terminal state; starting the camera
        // then would silently undo a camera-loss screen the user has not dismissed yet.
        val phase = _state.value.phase
        if (phase == CapturePhase.CameraLost || phase == CapturePhase.Failed) {
            Log.i(LOG_TAG, "ignoring preview surface while phase=$phase")
            return
        }
        Log.i(LOG_TAG, "preview surface ready, starting camera")
        controller.start(Surface(texture))
        _state.value = _state.value.copy(phase = CapturePhase.Preview, message = null)
    }

    fun toggleCapture() {
        // Logged with its caller so a capture that nobody asked for is traceable.
        val caller = Throwable().stackTrace.getOrNull(2)?.toString() ?: "unknown"
        Log.i(
            LOG_TAG,
            "toggleCapture phase=${_state.value.phase} frames=${_state.value.savedFrames} caller=$caller",
        )
        if (_state.value.phase == CapturePhase.Capturing) {
            stopCapture()
        } else {
            startCapture()
        }
    }

    override fun startCapture() {
        val controller = controller ?: return
        if (_state.value.phase == CapturePhase.Capturing) {
            return
        }

        val store = SessionStore(File(getApplication<Application>().filesDir, SESSIONS_DIRECTORY))
        benchmark = BenchmarkRecorder(getApplication(), Build.MODEL)
        savedFrameIndex = 0
        memoryTick = 0
        _state.value = _state.value.copy(
            phase = CapturePhase.Capturing,
            savedFrames = 0,
            sessionRelativePath = store.relativeRoot,
            message = null,
        )
        controller.startCapture(store)
    }

    override fun stopCapture() {
        controller?.stopCapture()
        closeBenchmark()
        _state.value = _state.value.copy(
            phase = CapturePhase.Locked,
            message = _state.value.sessionRelativePath,
        )
    }

    override fun savedFrames(): Int = _state.value.savedFrames

    fun retry() {
        // Logged with its caller so a retry nobody asked for is traceable.
        val caller = Throwable().stackTrace.getOrNull(2)?.toString() ?: "unknown"
        Log.i(LOG_TAG, "retry requested from phase=${_state.value.phase} caller=$caller")
        releaseController()
        _state.value = CaptureUiState(phase = CapturePhase.WaitingForPermission)
        onPermissionGranted()
    }

    override fun onPreviewStats(fps: Float, frame: FrameRecord) {
        // Derived from the camera clock over a full second, not from this callback's own
        // 250 ms throttle, which would otherwise be logged as the frame time.
        if (fps > 0f) {
            benchmark?.recordFrameTime(savedFrameIndex, 1_000.0 / fps)
        }
        _state.value = _state.value.copy(fps = fps)
    }

    override fun onLocked(settings: LockedCaptureSettings) {
        val phase = if (_state.value.phase == CapturePhase.Capturing) {
            CapturePhase.Capturing
        } else {
            CapturePhase.Locked
        }
        _state.value = _state.value.copy(locked = settings, phase = phase)
    }

    override fun onFrameSaved(savedFrameCount: Int, frame: FrameRecord) {
        savedFrameIndex = savedFrameCount
        memoryTick += 1
        if (memoryTick % MEMORY_SAMPLE_EVERY == 0) {
            benchmark?.recordMemory()
            benchmark?.recordThermal()
        }
        _state.value = _state.value.copy(savedFrames = savedFrameCount)
    }

    override fun onCameraLost(reason: String) {
        // The controller has already torn the device down; only the UI state is left to settle.
        closeBenchmark()
        Log.i(LOG_TAG, "camera lost: $reason")
        _state.value = _state.value.copy(phase = CapturePhase.CameraLost, message = reason)
    }

    override fun simulateCameraLoss() {
        controller?.simulateCameraLoss()
    }

    override fun onFailure(message: String) {
        _state.value = _state.value.copy(phase = CapturePhase.Failed, message = message)
    }

    override fun onCleared() {
        if (CaptureTestBridge.actions === this) {
            CaptureTestBridge.actions = null
        }
        releaseController()
        closeBenchmark()
    }

    private fun releaseController() {
        controller?.release()
        controller = null
    }

    private fun closeBenchmark() {
        benchmark?.close()
        benchmark = null
    }

    private companion object {
        const val LOG_TAG = "CaptureViewModel"
        const val SESSIONS_DIRECTORY = "sessions"
        const val MEMORY_SAMPLE_EVERY = 10
    }
}
