package com.scand3d.ui

import android.app.Application
import android.graphics.SurfaceTexture
import android.os.Build
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import android.view.WindowManager
import androidx.lifecycle.AndroidViewModel
import com.scand3d.benchmark.BenchmarkRecorder
import com.scand3d.capture.CameraCaptureController
import com.scand3d.capture.FrameRecord
import com.scand3d.capture.KeyframeColorSampler
import com.scand3d.capture.LockedCaptureSettings
import com.scand3d.capture.SessionStore
import com.scand3d.core.bridge.NativeCore
import com.scand3d.core.bridge.NativeOrientationMath
import com.scand3d.debug.CaptureTestBridge
import com.scand3d.reconstruct.CloudPlyWriter
import com.scand3d.reconstruct.DenseRequest
import com.scand3d.reconstruct.LiveReconstruction
import com.scand3d.reconstruct.LiveTracking
import com.scand3d.reconstruct.MvsQuality
import com.scand3d.reconstruct.ReconstructionRunner
import com.scand3d.render.MonochromePreviewRenderer
import com.scand3d.render.OverlayCameraParams
import com.scand3d.render.OverlayFeed
import com.scand3d.render.OverlayGeometry
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
    private var liveReconstruction: LiveReconstruction? = null
    private var denseRunner: ReconstructionRunner? = null
    private var savedFrameIndex = 0
    private var memoryTick = 0
    private var overlayHandle = 0L
    private var overlayPointCount = 0
    private var overlayDemo = false
    private var overlayReference: DoubleArray? = null
    private var overlayBasis: DoubleArray? = null
    private var lastOverlayPublishMs = 0L
    private var lastDriftLogMs = 0L

    // Live pose tracking against the map (D051). The IMU only knows rotation, so translation is
    // what keeps the cloud on the object while the phone moves.
    private val tracking = LiveTracking()

    /** Scale the map was rescaled by for the overlay; a tracked camera centre has to follow it. */
    private var overlayScale = 1.0

    private var analysisFrames = 0

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
            // The app is portrait-locked, so the sensor image needs this many quarter turns to be
            // upright on screen.
            previewRotationDeg =
                ((created.sensorOrientationDeg - displayRotationDeg()) % 360 + 360) % 360,
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
        // The keyframes are reconstructed as they arrive, so the map on screen during the scan is
        // this capture's own (roadmap 5.2) rather than something a debug command builds afterwards.
        liveReconstruction?.close()
        // A dense pass on the previous scan would compete with this capture's live map.
        denseRunner?.cancel()
        liveReconstruction = LiveReconstruction(
            on_cloud = { points, colours, reference, tracker ->
                applyReconstruction(
                    points,
                    reference,
                    tracker_handle = tracker,
                    keyframe_colours = colours,
                )
            },
        )
        savedFrameIndex = 0
        memoryTick = 0
        _state.value = _state.value.copy(
            phase = CapturePhase.Capturing,
            savedFrames = 0,
            low_light = false,
            keyframe_candidates = 0,
            sessionRelativePath = store.relativeRoot,
            message = null,
        )
        controller.startCapture(store)
    }

    override fun stopCapture() {
        // Lets the worker place the last keyframes, so the cloud is the whole capture.
        liveReconstruction?.finish()
        controller?.stopCapture()
        persistClockProfile()
        closeBenchmark()
        _state.value = _state.value.copy(
            phase = CapturePhase.Locked,
            message = _state.value.sessionRelativePath,
        )
        if (_state.value.denseEnabled) {
            // The dense stage reads the finished session from disk, so it starts after the stop.
            runDensePass()
        }
    }

    override fun savedFrames(): Int = _state.value.savedFrames

    override fun setPreviewRotationDeg(deg: Int) {
        // Debug only: the renderer reads this every frame, so the preview turns without a rebuild.
        MonochromePreviewRenderer.quarter_turn_override = deg
        Log.i(LOG_TAG, "preview rotation override: $deg deg")
    }

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
        // Anchor the overlay frame where the phone is when the overlay comes on.
        overlayReference = fusedImu.latest()?.quaternion_wxyz?.toDoubleArray()
        publishOverlay()
        sampleKeyframeColors()
    }

    /** UI: overlay visibility (roadmap 5.4); the renderer applies it without re-projecting. */
    fun setOverlayEnabled(enabled: Boolean) {
        _state.value = _state.value.copy(overlayEnabled = enabled)
        OverlayFeed.setStyle(enabled, _state.value.overlayOpacity)
    }

    /** UI: overlay opacity from 0 to 1 (roadmap 5.4). */
    fun setOverlayOpacity(opacity: Float) {
        val clamped = opacity.coerceIn(0f, 1f)
        _state.value = _state.value.copy(overlayOpacity = clamped)
        OverlayFeed.setStyle(_state.value.overlayEnabled, clamped)
    }

    /** UI: manual exposure for the stills; 0 returns to the locked value (D059). */
    fun setManualExposure(ns: Long) {
        val clamped = ns.coerceIn(0L, 100_000_000L)
        _state.value = _state.value.copy(manualExposureNs = clamped)
        controller?.setManualExposure(clamped)
    }

    /** UI: manual focus in dioptres; negative returns to the locked value (D059). */
    fun setManualFocus(diopter: Float) {
        val clamped = if (diopter < 0f) -1f else diopter.coerceIn(0f, 10f)
        _state.value = _state.value.copy(manualFocusDiopter = clamped)
        controller?.setManualFocus(clamped)
    }

    /** UI: one cloud per keyframe instead of photo colours (D059). */
    fun setKeyframeClouds(enabled: Boolean) {
        _state.value = _state.value.copy(keyframeClouds = enabled)
        Log.i(LOG_TAG, "keyframe clouds: $enabled")
    }

    /** UI: give a finished capture a dense pass (roadmap 6). */
    fun setDenseEnabled(enabled: Boolean) {
        _state.value = _state.value.copy(denseEnabled = enabled)
        if (!enabled) {
            denseRunner?.cancel()
            denseRunner = null
            _state.value = _state.value.copy(dense_progress = null)
        }
    }

    /** UI: Draft -> Normal -> High, the working width of the dense pass (roadmap 6.4). */
    fun cycleDenseQuality() {
        _state.value = _state.value.copy(denseQuality = _state.value.denseQuality.next())
    }

    /**
     * Runs the dense stage on the newest saved session: the sparse pass places the views, then
     * PatchMatch depth, the consistency filter and fusion produce a cloud that replaces the overlay
     * (roadmap 6). Wired to the stop path and to the debug receiver.
     */
    override fun runDensePass() {
        val frames = latestSessionFrames(MAX_DENSE_FRAMES)
        val camera = overlayCameraParams()
        if (frames.isEmpty() || camera == null) {
            Log.w(LOG_TAG, "dense: ${frames.size} frames, camera=${camera != null}")
            return
        }
        val quality = _state.value.denseQuality
        // One pass at a time: the new one replaces whatever was still running on the old scan.
        denseRunner?.cancel()
        val runner = ReconstructionRunner()
        denseRunner = runner
        _state.value = _state.value.copy(dense_progress = 0f, dense_points = 0)
        Log.i(
            LOG_TAG,
            "dense: ${frames.size} frames at ${quality.name} (${quality.working_width_px} px)",
        )
        val worker = Thread(
            {
                val outcome = runner.run(
                    frames,
                    camera,
                    dense = DenseRequest(quality.working_width_px),
                    on_dense_progress = { fraction ->
                        _state.value = _state.value.copy(dense_progress = fraction.toFloat())
                    },
                )
                if (outcome.cancelled) {
                    Log.i(LOG_TAG, "dense: cancelled")
                    _state.value = _state.value.copy(dense_progress = null)
                    return@Thread
                }
                if (outcome.dense_points == 0) {
                    Log.w(
                        LOG_TAG,
                        "dense: ${outcome.registered_views} registered, " +
                            "${outcome.dense_views} placed views, no points",
                    )
                    _state.value = _state.value.copy(dense_progress = null)
                    return@Thread
                }
                // The dense cloud shares the sparse map's frame, so the overlay camera and the
                // anchoring (D043) carry over unchanged.
                applyReconstruction(
                    outcome.dense_positions_m,
                    source = "dense",
                    publish_stream = false,
                )
                _state.value = _state.value.copy(
                    dense_progress = null,
                    dense_points = outcome.dense_points,
                )
                Log.i(
                    LOG_TAG,
                    "dense: ${outcome.dense_points} points from ${outcome.dense_views} views",
                )
            },
            "dense-reconstruction",
        )
        worker.isDaemon = true
        worker.start()
    }

    /**
     * Reconstructs the newest saved session on a worker thread and loads the cloud into the
     * overlay. This is the app-side path from captured frames to a sparse map that D030 records as
     * missing; it is driven from the debug receiver because there is no review screen yet.
     */
    override fun reconstructLatestSession() {
        val frames = latestSessionFrames(MAX_RECONSTRUCTION_FRAMES)
        val camera = overlayCameraParams()
        if (frames.isEmpty() || camera == null) {
            Log.w(LOG_TAG, "reconstruct: ${frames.size} frames, camera=${camera != null}")
            return
        }
        Log.i(LOG_TAG, "reconstruct: ${frames.size} frames")
        Thread(
            {
                val outcome = ReconstructionRunner().run(frames, camera)
                if (outcome.points == 0) {
                    Log.w(
                        LOG_TAG,
                        "reconstruct: ${outcome.added_views} views, " +
                            "${outcome.registered_views} registered, no points",
                    )
                    return@Thread
                }
                applyReconstruction(
                    outcome.positions_m,
                    keyframe_colours = outcome.keyframe_colours,
                )
            },
            "reconstruction",
        ).start()
    }

    private fun applyReconstruction(
        positions_m: DoubleArray,
        reference_quaternion: DoubleArray? = null,
        source: String = "sparse",
        publish_stream: Boolean = true,
        tracker_handle: Long = 0L,
        keyframe_colours: FloatArray? = null,
    ) {
        // The snapshot is consumed here whether or not the cloud is usable, so no native handle
        // is leaked when a run produces points but the overlay cannot take them.
        if (tracker_handle != 0L) {
            tracking.replaceTracker(tracker_handle)
        }
        // The Blender add-on polls this file over adb (tools/blender, D044). It is written before
        // the overlay work so a viewer still gets the map when the preview is not running. Only the
        // sparse map is streamed: that file is the live map the add-on reads.
        if (publish_stream) {
            publishCloudForStream(positions_m)
        }

        // The session's first view is the reconstruction's origin, so the overlay is anchored to
        // the pose the phone held when that keyframe was captured, not to this moment (D043).
        if (overlayReference == null && reference_quaternion != null &&
            reference_quaternion.size >= 4
        ) {
            overlayReference = reference_quaternion
        }
        val handle = overlayHandle.takeIf { it != 0L } ?: NativeCore.overlayCreate().also {
            overlayHandle = it
        }
        if (handle == 0L) {
            Log.w(LOG_TAG, "$source: overlay scene unavailable")
            return
        }

        // Anchors the overlay pose on first use, which also gives the frame the cloud lives in.
        val camera = overlayCameraParams() ?: run {
            Log.w(LOG_TAG, "$source: camera intrinsics unavailable")
            return
        }
        // Only the first cloud fixes the frame: re-anchoring on every live update would make the
        // cloud jump whenever the map grows.
        // The cloud is in the first keyframe's camera frame and the overlay scene is the body frame
        // at that keyframe, so the cloud takes the camera's own extrinsic and nothing else.
        val basis = overlayBasis
            ?: OverlayGeometry.bodyFromCamera(false, displayRotationDeg()).also { overlayBasis = it }

        // Colours start white and are replaced by the keyframe sample below.
        val world = toOverlayFrame(positions_m, basis)
        val by_keyframe = _state.value.keyframeClouds && keyframe_colours != null &&
            keyframe_colours.size == world.size
        val colours = if (by_keyframe) {
            keyframe_colours!!
        } else {
            FloatArray(world.size) { 1.0f }
        }
        if (!NativeCore.overlaySetPoints(handle, world, colours)) {
            Log.w(LOG_TAG, "$source: overlay rejected the cloud")
            return
        }
        overlayPointCount = world.size / 3
        overlayDemo = true
        Log.i(LOG_TAG, "$source: $overlayPointCount points in the overlay")
        if (by_keyframe) {
            Log.i(LOG_TAG, "$source: keyframe-coloured clouds")
        } else {
            sampleKeyframeColors()
        }
        val vertices = publishOverlay()
        Log.i(LOG_TAG, "$source: $vertices of $overlayPointCount points project into the preview")
    }

    /**
     * Writes the live sparse map where the Blender add-on can poll it (tools/blender, D044). The
     * points stay in the reconstruction's frame; converting them for display is the viewer's job.
     */
    private fun publishCloudForStream(positions_m: DoubleArray) {
        val relative = _state.value.sessionRelativePath ?: return
        val file = File(
            File(getApplication<Application>().filesDir, relative),
            CloudPlyWriter.FILE_NAME,
        )
        if (!CloudPlyWriter.write(file, positions_m)) {
            Log.w(LOG_TAG, "cloud stream file write failed: $relative/${CloudPlyWriter.FILE_NAME}")
        }
    }

    /**
     * Reconstruction points are in the first camera's frame, whose +Z is forward. The overlay scene
     * lives in the anchored body frame, where that same axis is body -Z (D031), so the cloud has to
     * be rotated by the anchor camera pose before projection: without it every point sits behind
     * the camera and the preview stays empty.
     *
     * The reconstruction's scale is free (D028) and a real capture measured 17.6 m across, so the
     * rotation is followed by a rescale to [OVERLAY_CLOUD_MEDIAN_M]. Only the shape is meaningful.
     */
    private fun toOverlayFrame(positions_m: DoubleArray, camera_to_world: DoubleArray): DoubleArray {
        val count = positions_m.size / 3
        if (count == 0) {
            return positions_m
        }
        val rotated = DoubleArray(positions_m.size)
        val distances = DoubleArray(count)
        for (index in 0 until count) {
            val x = positions_m[index * 3]
            val y = positions_m[index * 3 + 1]
            val z = positions_m[index * 3 + 2]
            val world_x = camera_to_world[0] * x + camera_to_world[1] * y + camera_to_world[2] * z
            val world_y = camera_to_world[3] * x + camera_to_world[4] * y + camera_to_world[5] * z
            val world_z = camera_to_world[6] * x + camera_to_world[7] * y + camera_to_world[8] * z
            rotated[index * 3] = world_x
            rotated[index * 3 + 1] = world_y
            rotated[index * 3 + 2] = world_z
            distances[index] = Math.sqrt(world_x * world_x + world_y * world_y + world_z * world_z)
        }
        distances.sort()
        val median = distances[count / 2]
        if (!median.isFinite() || median <= 0.0) {
            return rotated
        }
        val scale = OVERLAY_CLOUD_MEDIAN_M / median
        // The tracked camera centre is in the map's free-scale frame, so it takes the same factor
        // the points just did; otherwise the pose and the cloud would disagree about distance.
        overlayScale = scale
        for (index in rotated.indices) {
            rotated[index] *= scale
        }
        Log.i(
            LOG_TAG,
            "reconstruct: cloud median ${median.format(3)} m, " +
                "scaled by ${scale.format(4)} for the overlay",
        )
        return rotated
    }

    /** Evenly spaced frames keep the baseline spread instead of clustering at the start. */
    private fun latestSessionFrames(limit: Int): List<File> {
        val sessions = File(getApplication<Application>().filesDir, SESSIONS_DIRECTORY)
        val latest = sessions.listFiles()?.filter { it.isDirectory }?.maxByOrNull { it.name }
            ?: return emptyList()
        val frames = File(latest, FRAMES_DIRECTORY).listFiles()
            ?.filter { it.isFile && it.extension == "jpg" }
            ?.sortedBy { it.name }
            ?: return emptyList()
        if (frames.size <= limit) {
            return frames
        }
        val step = frames.size.toDouble() / limit
        return (0 until limit)
            .map { frames[(it * step).toInt().coerceAtMost(frames.lastIndex)] }
            .distinct()
    }

    /** Projects the scene on the live pose and hands the vertices to the renderer (5.1, 5.2). */
    private fun publishOverlay(): Int {
        val handle = overlayHandle
        if (handle == 0L) {
            return 0
        }
        val camera = overlayCameraParams() ?: return 0

        val out = FloatArray(MAX_OVERLAY_VERTICES * NativeCore.OVERLAY_VERTEX_VALUE_COUNT)
        val count = NativeCore.overlayProject(
            handle,
            camera.rotation,
            camera.centre_m,
            camera.focal_px,
            camera.cx_px,
            camera.cy_px,
            camera.image_width_px,
            camera.image_height_px,
            out,
        )
        val vertices = if (count > 0) {
            out.copyOf(count * NativeCore.OVERLAY_VERTEX_VALUE_COUNT)
        } else {
            FloatArray(0)
        }
        OverlayFeed.publish(vertices)
        logOverlayDrift(vertices)
        return count
    }

    /**
     * Roadmap 5.5 asks for a number, not an impression. Once a second, log the angle between the
     * anchored pose and the live one plus the projected centre of the cloud: with the phone held
     * still both should stop changing. That is pose drift; alignment to a real object still needs
     * a real cloud and a tracked pose (D030, D032).
     */
    private fun logOverlayDrift(vertices: FloatArray) {
        val now = SystemClock.uptimeMillis()
        if (now - lastDriftLogMs < DRIFT_LOG_INTERVAL_MS) {
            return
        }
        lastDriftLogMs = now

        val reference = overlayReference
        val live = fusedImu.latest()?.quaternion_wxyz
        val angle_rad = if (reference != null && live != null) {
            NativeCore.angularDisplacementRad(reference.toList(), live)
        } else {
            Double.NaN
        }

        val stride = NativeCore.OVERLAY_VERTEX_VALUE_COUNT
        val count = vertices.size / stride
        var sum_x = 0.0
        var sum_y = 0.0
        for (index in 0 until count) {
            sum_x += vertices[index * stride]
            sum_y += vertices[index * stride + 1]
        }
        val centre_x = if (count > 0) sum_x / count else Double.NaN
        val centre_y = if (count > 0) sum_y / count else Double.NaN
        val tracked = tracking.pose()
        val tracking_text = if (tracked != null) {
            "tracked ${tracked.inliers} inliers, camera in overlay frame " +
                "(${(tracked.centre_m[0] * overlayScale).format(3)}, " +
                "${(tracked.centre_m[1] * overlayScale).format(3)}, " +
                "${(tracked.centre_m[2] * overlayScale).format(3)}) m"
        } else {
            "tracked none, IMU rotation only"
        }
        Log.i(
            LOG_TAG,
            "overlay drift: pose ${(angle_rad * 180.0 / Math.PI).format(2)} deg, " +
                "centre (${centre_x.format(4)}, ${centre_y.format(4)}), $tracking_text",
        )
    }

    private fun Double.format(decimals: Int): String = String.format(java.util.Locale.US, "%.${decimals}f", this)

    /**
     * Overlay camera for this frame (roadmap 5.2): the camera's own focal length, the display
     * orientation as the body-to-camera extrinsic, and the live fused orientation relative to the
     * pose the overlay was anchored at. Null until the camera reported both the preview size and
     * its physical sensor numbers.
     */
    private fun overlayCameraParams(): OverlayCameraParams? {
        val preview = controller ?: return null
        val width = _state.value.previewWidthPx
        val height = _state.value.previewHeightPx
        if (width <= 0 || height <= 0) {
            return null
        }
        val focal_px = preview.focalPxFor(maxOf(width, height))
        if (!(focal_px > 0.0)) {
            return null
        }

        val extrinsic = OverlayGeometry.bodyFromCamera(
            frontFacing = false,
            displayRotationDeg = displayRotationDeg(),
        )
        // A tracked pose carries translation and is the real overlay camera; the IMU path below is
        // the fallback for the seconds before the first frame registers (D051).
        val tracked = tracking.pose()
        val anchor_basis = overlayBasis
        if (tracked != null && anchor_basis != null && overlayPointCount > 0) {
            val centre = OverlayGeometry.applyToVector(anchor_basis, tracked.centre_m)
            return OverlayCameraParams(
                // The cloud lives in basis * map, so the map's camera pose is rotated by the same
                // basis and its centre scaled by the same factor: the projection then agrees with
                // what the map's own camera would have produced.
                rotation = OverlayGeometry.multiply(anchor_basis, tracked.rotation),
                centre_m = doubleArrayOf(
                    centre[0] * overlayScale,
                    centre[1] * overlayScale,
                    centre[2] * overlayScale,
                ),
                focal_px = focal_px,
                cx_px = width / 2.0,
                cy_px = height / 2.0,
                image_width_px = width,
                image_height_px = height,
            )
        }

        val live = fusedImu.latest()?.quaternion_wxyz?.toDoubleArray()
        val rotation = when {
            live == null -> OverlayGeometry.IDENTITY_ROTATION
            overlayReference == null -> {
                overlayReference = live
                OverlayGeometry.overlayRotation(live, live, extrinsic)
            }
            else -> OverlayGeometry.overlayRotation(overlayReference!!, live, extrinsic)
        }

        return OverlayCameraParams(
            rotation = rotation,
            centre_m = ZERO_CENTRE_M,
            focal_px = focal_px,
            cx_px = width / 2.0,
            cy_px = height / 2.0,
            image_width_px = width,
            image_height_px = height,
        )
    }

    /** Colours the scene from the newest saved keyframe (roadmap 5.3). */
    private fun sampleKeyframeColors() {
        val handle = overlayHandle
        val camera = overlayCameraParams()
        val keyframe = newestKeyframe()
        if (handle == 0L || camera == null || keyframe == null) {
            return
        }
        if (KeyframeColorSampler.sample(handle, camera, keyframe) > 0) {
            publishOverlay()
        }
    }

    private fun newestKeyframe(): File? {
        val sessions = File(getApplication<Application>().filesDir, SESSIONS_DIRECTORY)
        val latest = sessions.listFiles()?.filter { it.isDirectory }?.maxByOrNull { it.name }
            ?: return null
        return File(latest, FRAMES_DIRECTORY).listFiles()
            ?.filter { it.isFile && it.extension == "jpg" }
            ?.maxByOrNull { it.name }
    }

    @Suppress("DEPRECATION")
    private fun displayRotationDeg(): Int {
        val manager = getApplication<Application>().getSystemService(WindowManager::class.java)
            ?: return 0
        val display = manager.defaultDisplay ?: return 0
        return when (display.rotation) {
            Surface.ROTATION_90 -> 90
            Surface.ROTATION_180 -> 180
            Surface.ROTATION_270 -> 270
            else -> 0
        }
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
            // The anchored body frame puts the camera's forward axis on -Z (D031), which is where
            // the demo ring has to sit to be in front of the lens.
            positions[index * 3 + 2] = -DEMO_RING_DISTANCE_M
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
        overlayBasis = null
        overlayPointCount = 0
        // Nothing on screen to hold in place, so stop tracking against the map as well.
        tracking.replaceTracker(0L)
    }

    fun retry() {
        // Logged with its caller so a retry nobody asked for is traceable.
        val caller = Throwable().stackTrace.getOrNull(2)?.toString() ?: "unknown"
        Log.i(LOG_TAG, "retry requested from phase=${_state.value.phase} caller=$caller")
        releaseController()
        tracking.replaceTracker(0L)
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

    /**
     * Camera thread: one gray preview frame for tracking. Only the live map is worth matching
     * against, so this costs nothing until a cloud is on screen.
     */
    override fun onAnalysisFrame(width_px: Int, height_px: Int, gray: ByteArray) {
        // Logged once: it is the only way to tell a camera that provides no analysis output apart
        // from tracking that has simply not locked yet (D051).
        if (analysisFrames == 0) {
            Log.i(LOG_TAG, "analysis stream: ${width_px}x$height_px gray frames arriving")
        }
        analysisFrames += 1
        if (overlayHandle == 0L || overlayPointCount == 0) {
            return
        }
        val controller = controller ?: return
        tracking.onFrame(gray, width_px, height_px, controller.focalPxFor(maxOf(width_px, height_px)))
    }

    /** Camera thread: the saved frames are too dark for features, so the scan needs more light. */
    override fun onLowLight(mean_luma: Int) {
        Log.w(LOG_TAG, "low light: mean luma $mean_luma/255 in the saved frames")
        _state.value = _state.value.copy(low_light = true)
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

        // Keyframes are the frames the scan can actually register, so they feed the map while the
        // capture runs. The camera parameters follow the preview, and the worker scales them to
        // the decoded keyframe size.
        val path = frame.relative_path
        if (frame.keyframe_candidate && path != null) {
            val camera = overlayCameraParams()
            if (camera == null) {
                Log.w(LOG_TAG, "keyframe $path skipped: camera intrinsics unavailable")
            } else {
                liveReconstruction?.onKeyframe(
                    File(getApplication<Application>().filesDir, path),
                    camera,
                    frame.orientation_wxyz,
                )
            }
        }
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
        denseRunner?.cancel()
        denseRunner = null
        releaseController()
        closeBenchmark()
        releaseOverlay()
        tracking.close()
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

        // The overlay follows the live pose, refreshed at the preview rate rather than the
        // 200 Hz sensor rate: re-projecting faster than the screen can show is wasted work.
        if (overlayDemo) {
            val now = SystemClock.uptimeMillis()
            if (now - lastOverlayPublishMs >= OVERLAY_PUBLISH_INTERVAL_MS) {
                lastOverlayPublishMs = now
                publishOverlay()
            }
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
        liveReconstruction?.close()
        liveReconstruction = null
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
        const val OVERLAY_CLOUD_MEDIAN_M = 0.55
        const val MAX_RECONSTRUCTION_FRAMES = 12
        // A dense pass costs roughly a second per view on the host at Draft, several on the phone,
        // so it runs on this many evenly spaced keyframes rather than every frame saved.
        const val MAX_DENSE_FRAMES = 24
        const val DEMO_POINT_COUNT = 64
        const val DEMO_RING_RADIUS_M = 0.18
        const val DEMO_RING_DISTANCE_M = 1.0
        const val OVERLAY_PUBLISH_INTERVAL_MS = 33L
        const val DRIFT_LOG_INTERVAL_MS = 1000L
        const val FRAMES_DIRECTORY = "frames"
        val ZERO_CENTRE_M = doubleArrayOf(0.0, 0.0, 0.0)
    }
}
