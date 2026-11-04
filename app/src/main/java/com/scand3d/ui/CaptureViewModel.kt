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
import com.scand3d.core.bridge.NativeCore
import com.scand3d.core.bridge.NativeOrientationMath
import com.scand3d.debug.CaptureTestBridge
import com.scand3d.render.OverlayFeed
import com.scand3d.sensors.AndroidImuSource
import com.scand3d.sensors.AngularRateMonitor
import com.scand3d.sensors.DeviceClockProfile
import com.scand3d.sensors.DeviceClockProfileStore
import com.scand3d.sensors.FusedImu
import com.scand3d.sensors.ImuFrameAnnotator
import com.scand3d.sensors.ImuFusion
import com.scand3d.sensors.ImuMode
import com.scand3d.sensors.ImuRingBuffer
import com.scand3d.sensors.ImuSample
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.File

/**
 * Owns the camera controller and the sensor layer, and turns their callbacks into UI state.
 * Callbacks arrive on the camera and sensor threads; `StateFlow` is the only thing that crosses
 * to the UI thread.
 */
class CaptureViewModel(application: Application) :
    AndroidViewModel(application),
    CameraCaptureController.Listener,
    CaptureTestBridge.Actions {

    private val _state = MutableStateFlow(CaptureUiState())
    val state: StateFlow<CaptureUiState> = _state.asStateFlow()

    private val imuSource = AndroidImuSource(getApplication())
    private val fusedImu = FusedImu(
        source = imuSource,
        filter = ImuFusion(SENSOR_RATE_HZ),
        ring = ImuRingBuffer(),
        onSample = ::onImuSample,
    )
    private val motionMonitor = AngularRateMonitor()
    private val clockProfiles = DeviceClockProfileStore(
        File(getApplication<Application>().filesDir, DEVICES_DIRECTORY),
    )

    private var controller: CameraCaptureController? = null
    private var recorder: ImuFrameAnnotator? = null
    private var benchmark: BenchmarkRecorder? = null
    private var savedFrameIndex = 0
    private var memoryTick = 0
    private var overlayHandle = 0L
    private var overlayPointCount = 0
    private var overlayDemo = false

    init {
        CaptureTestBridge.actions = this
    }

    fun onPermissionGranted() {
        Log.i(LOG_TAG, "onPermissionGranted controller=${controller != null} phase=${_state.value.phase}")
        if (controller != null) {
            return
        }

        val availability = imuSource.availability
        fusedImu.start()

        val annotator = ImuFrameAnnotator(
            snapshots = fusedImu::snapshotAt,
            math = NativeOrientationMath,
            availability = availability,
            // A profile from an earlier session on this device centres the pairing lookup; the
            // first session starts at zero and measures the offset as it goes.
            applied_offset_ns = runCatching { clockProfiles.load(Build.MODEL)?.offset_ns }.getOrNull()
                ?: 0L,
        )
        recorder = annotator
        _state.value = _state.value.copy(
            imu_mode = availability.mode,
            imu_reduced_accuracy = availability.reduced_accuracy,
        )
        Log.i(
            LOG_TAG,
            "imu mode=${availability.mode} reduced_accuracy=${availability.reduced_accuracy}",
        )

        val created = CameraCaptureController(getApplication(), this, annotator)
        controller = created
        val error = created.initialisationError
        if (error != null) {
            // Nothing will consume samples behind the error screen, so do not keep the sensor
            // thread and its listeners alive.
            fusedImu.stop()
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
        if (overlayDemo) {
            publishOverlay()
        }
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
            keyframe_candidates = 0,
            sessionRelativePath = store.relativeRoot,
            message = null,
        )
        controller.startCapture(store)
    }

    override fun stopCapture() {
        controller?.stopCapture()
        persistClockProfile()
        closeBenchmark()
        _state.value = _state.value.copy(
            phase = CapturePhase.Locked,
            message = _state.value.sessionRelativePath,
        )
    }

    override fun savedFrames(): Int = _state.value.savedFrames

    override fun setOverlayDemo(enabled: Boolean) {
        overlayDemo = enabled
        if (!enabled) {
            releaseOverlay()
            OverlayFeed.publish(FloatArray(0))
            return
        }
        if (overlayHandle == 0L) {
            val handle = NativeCore.overlayCreate()
            if (handle == 0L) {
                Log.w(LOG_TAG, "overlay scene unavailable, native core not loaded")
                return
            }
            overlayHandle = handle
            val (positions, colours) = demoCloud()
            if (!NativeCore.overlaySetPoints(handle, positions, colours)) {
                Log.w(LOG_TAG, "overlay points rejected by the native scene")
            }
            overlayPointCount = positions.size / 3
            Log.i(LOG_TAG, "overlay demo scene: $overlayPointCount points")
        }
        publishOverlay()
    }

    /** Projects the current scene and hands the vertices to the GL renderer (roadmap 5.1). */
    private fun publishOverlay() {
        val handle = overlayHandle
        if (handle == 0L) {
            return
        }
        val width = _state.value.previewWidthPx
        val height = _state.value.previewHeightPx
        if (width <= 0 || height <= 0) {
            return
        }

        // STUB: placeholder intrinsics and an identity pose. Roadmap 5.2 replaces both with the
        // camera model and the tracked pose; until then this only proves the render path.
        val out = FloatArray(MAX_OVERLAY_VERTICES * NativeCore.OVERLAY_VERTEX_VALUE_COUNT)
        val count = NativeCore.overlayProject(
            handle,
            IDENTITY_ROTATION,
            ZERO_CENTRE_M,
            DEMO_FOCAL_FRACTION * width,
            width / 2.0,
            height / 2.0,
            width,
            height,
            out,
        )
        val vertices = if (count > 0) {
            out.copyOf(count * NativeCore.OVERLAY_VERTEX_VALUE_COUNT)
        } else {
            FloatArray(0)
        }
        OverlayFeed.publish(vertices)
        Log.i(LOG_TAG, "overlay published $count of $overlayPointCount points")
    }

    // STUB: Phase 4 output is not wired into the app yet (D030), so the render path is verified
    // with a canned ring of points one meter in front of the camera. It is only drawn when the
    // debug receiver asks for it, never during a real scan.
    private fun demoCloud(): Pair<DoubleArray, FloatArray> {
        val positions = DoubleArray(DEMO_POINT_COUNT * 3)
        val colours = FloatArray(DEMO_POINT_COUNT * 3)
        for (index in 0 until DEMO_POINT_COUNT) {
            val angle = 2.0 * Math.PI * index / DEMO_POINT_COUNT
            positions[index * 3] = DEMO_RING_RADIUS_M * Math.cos(angle)
            positions[index * 3 + 1] = DEMO_RING_RADIUS_M * Math.sin(angle)
            positions[index * 3 + 2] = DEMO_RING_DISTANCE_M
            colours[index * 3] = (0.5 + 0.5 * Math.cos(angle)).toFloat()
            colours[index * 3 + 1] = (0.5 + 0.5 * Math.cos(angle - 2.0 * Math.PI / 3.0)).toFloat()
            colours[index * 3 + 2] = (0.5 + 0.5 * Math.cos(angle - 4.0 * Math.PI / 3.0)).toFloat()
        }
        return positions to colours
    }

    private fun releaseOverlay() {
        if (overlayHandle != 0L) {
            NativeCore.overlayDestroy(overlayHandle)
            overlayHandle = 0L
        }
        overlayPointCount = 0
    }

    fun retry() {
        // Logged with its caller so a retry nobody asked for is traceable.
        val caller = Throwable().stackTrace.getOrNull(2)?.toString() ?: "unknown"
        Log.i(LOG_TAG, "retry requested from phase=${_state.value.phase} caller=$caller")
        releaseController()
        fusedImu.reset()
        motionMonitor.reset()
        recorder = null
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
        _state.value = _state.value.copy(
            savedFrames = savedFrameCount,
            keyframe_candidates = recorder?.keyframeCandidates() ?: _state.value.keyframe_candidates,
        )
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
        releaseOverlay()
        fusedImu.close()
        recorder = null
    }

    /** Sensor thread: keep the blur warning current without touching capture state. */
    private fun onImuSample(sample: ImuSample) {
        val warning = motionMonitor.update(
            gyro_rad_s = sample.gyro_rad_s,
            has_gyro = sample.has_gyro,
            exposure_ns = _state.value.locked?.exposure_ns ?: 0L,
        )
        if (warning != _state.value.motion) {
            _state.value = _state.value.copy(motion = warning)
        }
    }

    /** Stores this device's measured camera/sensor residual for the next session (roadmap 2.3). */
    private fun persistClockProfile() {
        val annotator = recorder ?: return
        val metadata = annotator.metadata(controller?.cameraClockSource ?: "unknown")
        val offset_ns = metadata.clock_offset_ns ?: return
        val saved = clockProfiles.save(
            DeviceClockProfile(
                device_model = Build.MODEL,
                camera_clock_source = metadata.camera_clock_source,
                sensor_clock = metadata.sensor_clock,
                offset_ns = offset_ns,
                offset_mad_ns = metadata.clock_offset_mad_ns ?: 0L,
                samples = metadata.clock_samples,
                imu_mode = metadata.imu_mode ?: ImuMode.Unavailable.name,
                measured_at_ms = System.currentTimeMillis(),
            ),
        )
        if (!saved) {
            Log.w(LOG_TAG, "clock profile write failed")
        }
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
        const val DEVICES_DIRECTORY = "devices"
        const val SENSOR_RATE_HZ = 200.0
        const val MEMORY_SAMPLE_EVERY = 10
        const val MAX_OVERLAY_VERTICES = 2048
        const val DEMO_POINT_COUNT = 64
        const val DEMO_RING_RADIUS_M = 0.18
        const val DEMO_RING_DISTANCE_M = 1.0
        const val DEMO_FOCAL_FRACTION = 0.9
        val IDENTITY_ROTATION = doubleArrayOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        val ZERO_CENTRE_M = doubleArrayOf(0.0, 0.0, 0.0)
    }
}
