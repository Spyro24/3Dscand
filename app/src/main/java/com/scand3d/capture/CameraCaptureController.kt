package com.scand3d.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.RggbChannelVector
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.media.ExifInterface
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Range
import android.util.Size
import android.view.Surface
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executor
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

/**
 * Camera2 capture session for a scan: preview plus full-resolution JPEG stills, with
 * auto-exposure, auto-white-balance and auto-focus locked after a warm-up period.
 *
 * All camera calls run on one callback thread; results reach the UI through [Listener].
 */
class CameraCaptureController(
    context: Context,
    private val listener: Listener,
    private val recorder: SessionRecorder? = null,
) {

    interface Listener {
        fun onPreviewStats(fps: Float, frame: FrameRecord)
        fun onLocked(settings: LockedCaptureSettings)
        fun onFrameSaved(savedFrameCount: Int, frame: FrameRecord)
        fun onCameraLost(reason: String)
        fun onFailure(message: String)
    }

    private val cameraManager = context.applicationContext.getSystemService(CameraManager::class.java)
    private val appContext = context.applicationContext

    /** Null when the device has no usable back camera; the UI reports this instead of crashing. */
    val initialisationError: String?

    val previewSizePx: Size
    val stillSizePx: Size

    /** Camera2's timestamp source: "realtime" means the same clock as the sensors. */
    val cameraClockSource: String

    private val cameraId: String?

    /** Sensor image rotation for this camera; the overlay extrinsic needs it (roadmap 5.2). */
    val sensorOrientationDeg: Int

    /** Camera focal length in millimeters, 0 when the device did not report one. */
    val focalLengthMm: Float

    /** Long edge of the sensor in millimeters, 0 when the device did not report one. */
    val sensorLongEdgeMm: Double
    private val fpsRange: Range<Int>?
    private var cameraThread: HandlerThread? = null
    private var handler: Handler? = null
    private var cameraDevice: CameraDevice? = null
    private var deviceStateCallback: CameraDevice.StateCallback? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null
    private var previewSurface: Surface? = null
    private var lockedSettings: LockedCaptureSettings? = null
    private var pendingStill: FrameRecord? = null

    private val ringBuffer = FrameRingBuffer(RING_BUFFER_CAPACITY)
    private val capturing = AtomicBoolean(false)
    private var sessionStore: SessionStore? = null
    private val savedFrames = AtomicInteger(0)

    // JPEG writes run here so a 3 MB write never stalls the camera callbacks.
    private val fileWriter = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "frame-writer").apply { isDaemon = true }
    }
    private var warmUpFrames = 0
    private var fpsWindowStartNs = 0L
    private var fpsWindowFrames = 0
    private var lastStatsReportMs = 0L

    init {
        var resolvedId: String? = null
        var preview = Size(1280, 720)
        var still = Size(1920, 1080)
        var orientation = 90
        var fps: Range<Int>? = null
        var clockSource = "unknown"
        var error: String? = null
        var focal_mm = 0f
        var sensor_long_mm = 0.0

        try {
            resolvedId = findBackCameraId()
            val characteristics = cameraManager.getCameraCharacteristics(resolvedId)
            val map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            if (map == null) {
                error = "camera reports no stream configuration"
            } else {
                preview = choosePreviewSize(map.getOutputSizes(SurfaceTexture::class.java) ?: emptyArray())
                still = chooseStillSize(map.getOutputSizes(ImageFormat.JPEG) ?: emptyArray())
                fps = chooseFpsRange(characteristics)
            }
            orientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90
            focal_mm = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                ?.firstOrNull() ?: 0f
            val physical = characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            if (physical != null) {
                sensor_long_mm = maxOf(physical.width, physical.height).toDouble()
            }
            clockSource = if (
                characteristics.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) ==
                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
            ) {
                "realtime"
            } else {
                "unknown"
            }
        } catch (t: Throwable) {
            error = t.message ?: t.javaClass.simpleName
        }

        initialisationError = error
        cameraId = resolvedId
        previewSizePx = preview
        stillSizePx = still
        sensorOrientationDeg = orientation
        fpsRange = fps
        cameraClockSource = clockSource
        focalLengthMm = focal_mm
        sensorLongEdgeMm = sensor_long_mm
    }

    /**
     * Focal length in pixels for an image whose long edge is `imageLongEdgePx` (roadmap 5.2).
     * Square pixels make one focal length serve both axes, so the long edge is enough; the
     * principal point is taken as the image centre. Returns 0 when the camera did not report the
     * physical numbers, which tells the overlay to stay off rather than guess.
     */
    fun focalPxFor(imageLongEdgePx: Int): Double {
        if (focalLengthMm <= 0f || sensorLongEdgeMm <= 0.0 || imageLongEdgePx <= 0) {
            return 0.0
        }
        return focalLengthMm.toDouble() * imageLongEdgePx / sensorLongEdgeMm
    }

    /** Loads total numbers of preview frames buffered; used by later phases and diagnostics. */
    fun bufferedFrameCount(): Int = ringBuffer.size()

    fun lockedSettings(): LockedCaptureSettings? = lockedSettings

    @SuppressLint("MissingPermission")
    fun start(surface: Surface) {
        val id = cameraId
        if (initialisationError != null || id == null) {
            listener.onFailure(initialisationError ?: "no camera")
            return
        }
        previewSurface = surface
        ensureThread()

        val thread = cameraThread ?: return
        val cameraHandler = Handler(thread.looper)
        handler = cameraHandler

        imageReader = ImageReader.newInstance(
            stillSizePx.width,
            stillSizePx.height,
            ImageFormat.JPEG,
            IMAGE_READER_BUFFERS,
        ).also { reader ->
            reader.setOnImageAvailableListener({ onJpegAvailable(it) }, cameraHandler)
        }

        try {
            val callback = object : CameraDevice.StateCallback() {
                override fun onOpened(device: CameraDevice) {
                    cameraDevice = device
                    createSession(device)
                }

                override fun onDisconnected(device: CameraDevice) {
                    handleCameraLoss("camera disconnected", device)
                }

                override fun onError(device: CameraDevice, error: Int) {
                    handleCameraLoss("camera error $error", device)
                }
            }
            // Kept so a debug build can fire this very callback when the HAL cannot be forced
            // to drop the session from adb; the teardown and UI path stay production code.
            deviceStateCallback = callback
            cameraManager.openCamera(id, callback, cameraHandler)
        } catch (t: Throwable) {
            listener.onFailure(t.message ?: "openCamera failed")
        }
    }

    /**
     * The single teardown path for a dead camera. Both OS callbacks and the debug hook land here,
     * so what a device test exercises is what a real disconnect runs.
     */
    private fun handleCameraLoss(reason: String, device: CameraDevice?) {
        Log.w(LOG_TAG, "camera lost: $reason")
        capturing.set(false)
        flushManifest(shutdownWriter = true)
        runCatching { captureSession?.close() }
        runCatching { device?.close() }
        runCatching { imageReader?.close() }
        captureSession = null
        cameraDevice = null
        imageReader = null
        previewSurface = null
        deviceStateCallback = null
        cameraThread?.quitSafely()
        // A quit thread cannot hand out a looper, so keep the field clear or the next start builds
        // a Handler with a null looper and takes the GL thread down with it.
        cameraThread = null
        handler = null
        listener.onCameraLost(reason)
    }

    /** Debug hook: delivers the OS disconnect event from the camera thread. */
    fun simulateCameraLoss() {
        val cameraHandler = handler
        val device = cameraDevice
        val callback = deviceStateCallback
        if (cameraHandler == null || device == null || callback == null) {
            listener.onCameraLost("camera not open")
            return
        }
        cameraHandler.post { callback.onDisconnected(device) }
    }

    fun startCapture(store: SessionStore) {
        handler?.post {
            sessionStore = store
            savedFrames.set(0)
            capturing.set(true)
            captureStill()
        }
    }

    fun stopCapture() {
        handler?.post {
            capturing.set(false)
            flushManifest(shutdownWriter = false)
        }
    }

    /** Queued behind any pending frame writes, so the manifest counts every saved frame. */
    private fun flushManifest(shutdownWriter: Boolean) {
        val store = sessionStore
        val settings = lockedSettings
        val queued = runCatching {
            fileWriter.execute {
                store?.writeManifest(settings, android.os.Build.MODEL, recorder?.metadata(cameraClockSource))
                if (shutdownWriter) {
                    fileWriter.shutdown()
                }
            }
        }.isSuccess

        // After a loss the writer may already be shut down; write inline rather than lose the
        // frames that did reach disk.
        if (!queued && store != null) {
            runCatching {
                store.writeManifest(settings, android.os.Build.MODEL, recorder?.metadata(cameraClockSource))
                if (shutdownWriter) {
                    fileWriter.shutdown()
                }
            }
        }
    }

    fun release() {
        val thread = cameraThread
        val cameraHandler = handler
        cameraThread = null
        handler = null
        if (cameraHandler == null) {
            thread?.quitSafely()
            return
        }
        cameraHandler.post {
            capturing.set(false)
            flushManifest(shutdownWriter = true)
            runCatching { captureSession?.close() }
            runCatching { cameraDevice?.close() }
            runCatching { imageReader?.close() }
            captureSession = null
            cameraDevice = null
            imageReader = null
            previewSurface = null
            deviceStateCallback = null
            thread?.quitSafely()
        }
    }

    private fun ensureThread() {
        val existing = cameraThread
        if (existing != null && existing.isAlive) {
            return
        }
        cameraThread = HandlerThread("camera-capture").also { it.start() }
        handler = Handler(cameraThread!!.looper)
    }

    private fun findBackCameraId(): String {
        val ids = cameraManager.cameraIdList
        for (id in ids) {
            val facing = cameraManager.getCameraCharacteristics(id)
                .get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_BACK) {
                return id
            }
        }
        return ids.firstOrNull() ?: error("no camera available")
    }

    private fun choosePreviewSize(sizes: Array<Size>): Size {
        // A 4:3 stream matches the stills and the scan's field of view, and turning it upright then
        // cropping it to the preview box costs about 14% of the height; a 16:9 stream loses a third
        // of it, which reads as a zoomed, stretched preview (D042).
        sizes.firstOrNull { it.width == 1280 && it.height == 960 }?.let { return it }
        sizes.filter { it.width * 3 == it.height * 4 && it.width <= 1440 }
            .maxByOrNull { it.width * it.height }
            ?.let { return it }
        sizes.firstOrNull { it.width == 1280 && it.height == 720 }?.let { return it }
        return sizes.filter { it.width <= 1920 }.maxByOrNull { it.width * it.height } ?: sizes.first()
    }

    private fun chooseStillSize(sizes: Array<Size>): Size =
        sizes.maxByOrNull { it.width.toLong() * it.height } ?: Size(1920, 1080)

    private fun chooseFpsRange(characteristics: CameraCharacteristics): Range<Int>? {
        val ranges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            ?: return null
        // A fixed 30 fps range keeps preview timing predictable; otherwise take the highest cap.
        return ranges.firstOrNull { it.lower == 30 && it.upper == 30 }
            ?: ranges.maxByOrNull { it.upper }
    }

    private fun createSession(device: CameraDevice) {
        val preview = previewSurface ?: return
        val reader = imageReader ?: return

        val outputs = listOf(
            OutputConfiguration(preview),
            OutputConfiguration(reader.surface),
        )
        val executor = Executor { command -> handler?.post(command) }
        val configuration = SessionConfiguration(
            SessionConfiguration.SESSION_REGULAR,
            outputs,
            executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(session: CameraCaptureSession) {
                    captureSession = session
                    startPreview(session)
                }

                override fun onConfigureFailed(session: CameraCaptureSession) {
                    listener.onFailure("capture session configuration failed")
                }
            },
        )

        runCatching { device.createCaptureSession(configuration) }
            .onFailure { listener.onFailure(it.message ?: "createCaptureSession failed") }
    }

    private fun startPreview(session: CameraCaptureSession) {
        val preview = previewSurface ?: return
        val request = cameraDevice?.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW) ?: return
        request.addTarget(preview)
        fpsRange?.let { request.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
        request.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
        request.set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)
        request.set(CaptureRequest.CONTROL_AWB_MODE, CameraMetadata.CONTROL_AWB_MODE_AUTO)
        request.set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
        request.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
        request.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF)

        runCatching { session.setRepeatingRequest(request.build(), previewCallback, handler) }
            .onFailure { listener.onFailure(it.message ?: "setRepeatingRequest failed") }
    }

    private val previewCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            session: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult,
        ) {
            // Registered only on repeating requests, so every callback here is a preview frame.
            val frame = result.toFrameRecord(relativePath = null)
            ringBuffer.add(frame)
            reportFps(frame)
            warmUpFrames += 1

            if (lockedSettings == null) {
                tryLock(session, result)
            }

            val now = System.currentTimeMillis()
            if (now - lastStatsReportMs >= STATS_INTERVAL_MS) {
                lastStatsReportMs = now
                listener.onPreviewStats(currentFps(), frame)
            }
        }
    }

    /**
     * Locks only once the camera reported that exposure, white balance and focus settled.
     * Freezing the first frame after warm-up used to be enough to catch the lens mid-travel.
     */
    private fun tryLock(session: CameraCaptureSession, result: TotalCaptureResult) {
        if (warmUpFrames < WARM_UP_FRAMES) {
            return
        }

        val aeState = result.get(CaptureResult.CONTROL_AE_STATE)
        val aeSettled = aeState == null ||
            aeState == CaptureResult.CONTROL_AE_STATE_CONVERGED ||
            aeState == CaptureResult.CONTROL_AE_STATE_LOCKED

        val afState = result.get(CaptureResult.CONTROL_AF_STATE)
        val afSettled = afState == null ||
            afState == CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED ||
            afState == CaptureResult.CONTROL_AF_STATE_PASSIVE_FOCUSED ||
            afState == CaptureResult.CONTROL_AF_STATE_PASSIVE_UNFOCUSED

        val lensStationary = result.get(CaptureResult.LENS_STATE) == CameraMetadata.LENS_STATE_STATIONARY

        if (!aeSettled || !afSettled || !lensStationary) {
            if (warmUpFrames < MAX_WARM_UP_FRAMES) {
                return
            }
            Log.w(LOG_TAG, "locking without convergence ae=$aeState af=$afState lens=$lensStationary")
        }
        lockSettings(session, result)
    }

    private fun lockSettings(session: CameraCaptureSession, result: TotalCaptureResult) {
        val gains = result.get(CaptureResult.COLOR_CORRECTION_GAINS)
        val settings = LockedCaptureSettings(
            exposure_ns = result.get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: 0L,
            iso = result.get(CaptureResult.SENSOR_SENSITIVITY) ?: 0,
            focus_diopter = result.get(CaptureResult.LENS_FOCUS_DISTANCE) ?: 0f,
            awb_red = gains?.red ?: 1f,
            awb_green_even = gains?.greenEven ?: 1f,
            awb_green_odd = gains?.greenOdd ?: 1f,
            awb_blue = gains?.blue ?: 1f,
        )
        if (settings.exposure_ns <= 0L || settings.iso <= 0) {
            // Not enough information yet; try again on the next frame.
            warmUpFrames = 0
            return
        }
        lockedSettings = settings
        applyLockedSettings(session)
        listener.onLocked(settings)
        Log.i(
            LOG_TAG,
            "locked exposure=${settings.exposure_ns}ns iso=${settings.iso} " +
                "focus=${settings.focus_diopter}diopter",
        )
    }

    /** Rebuilds the repeating request so every later frame shares one photometric setting. */
    private fun applyLockedSettings(session: CameraCaptureSession) {
        val settings = lockedSettings ?: return
        val preview = previewSurface ?: return
        val request = cameraDevice?.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW) ?: return
        request.addTarget(preview)
        fpsRange?.let { request.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
        applySettings(request, settings)
        runCatching { session.setRepeatingRequest(request.build(), previewCallback, handler) }
            .onFailure { Log.e(LOG_TAG, "locked repeating request failed", it) }
    }

    private fun applySettings(request: CaptureRequest.Builder, settings: LockedCaptureSettings) {
        request.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_OFF)
        request.set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_OFF)
        request.set(CaptureRequest.SENSOR_EXPOSURE_TIME, settings.exposure_ns)
        request.set(CaptureRequest.SENSOR_SENSITIVITY, settings.iso)
        request.set(CaptureRequest.CONTROL_AWB_MODE, CameraMetadata.CONTROL_AWB_MODE_OFF)
        request.set(
            CaptureRequest.COLOR_CORRECTION_GAINS,
            RggbChannelVector(
                settings.awb_red,
                settings.awb_green_even,
                settings.awb_green_odd,
                settings.awb_blue,
            ),
        )
        request.set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_OFF)
        request.set(CaptureRequest.LENS_FOCUS_DISTANCE, settings.focus_diopter)
        request.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
        request.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF)
    }

    private fun captureStill() {
        val session = captureSession ?: return
        val device = cameraDevice ?: return
        val preview = previewSurface ?: return
        val reader = imageReader ?: return

        val request = device.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE).apply {
            addTarget(reader.surface)
            addTarget(preview)
            fpsRange?.let { set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
            lockedSettings?.let { applySettings(this, it) }
            set(CaptureRequest.JPEG_ORIENTATION, sensorOrientationDeg)
            set(CaptureRequest.JPEG_QUALITY, JPEG_QUALITY)
        }

        runCatching { session.capture(request.build(), stillCallback, handler) }
            .onFailure { Log.e(LOG_TAG, "still capture failed", it) }
    }

    private val stillCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            session: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult,
        ) {
            // Registered only on still capture requests; the JPEG listener runs on the same
            // thread, so this metadata is always ready before the image arrives.
            pendingStill = result.toFrameRecord(relativePath = null)
        }
    }

    private fun onJpegAvailable(reader: ImageReader) {
        val image = runCatching { reader.acquireNextImage() }.getOrNull() ?: return
        val store = sessionStore
        val metadata = pendingStill ?: ringBuffer.latest()
        val timestamp = image.timestamp

        val bytes: ByteArray
        try {
            if (store == null) {
                return
            }
            val buffer = image.planes[0].buffer
            bytes = ByteArray(buffer.remaining())
            buffer.get(bytes)
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "reading frame failed", t)
            listener.onFailure(t.message ?: "reading frame failed")
            return
        } finally {
            image.close()
        }

        pendingStill = null
        val index = savedFrames.incrementAndGet()

        // The write happens off the camera thread: at ~3 MB per frame, doing it inline dropped
        // the preview to a few frames per second while a burst was running.
        fileWriter.execute {
            try {
                val file = store.frameFile(index)
                file.writeBytes(bytes)
                writeExif(file, metadata, timestamp)

                val record = FrameRecord(
                    timestamp_ns = if (timestamp > 0) timestamp else (metadata?.timestamp_ns ?: 0L),
                    exposure_ns = metadata?.exposure_ns ?: 0L,
                    iso = metadata?.iso ?: 0,
                    focus_diopter = metadata?.focus_diopter ?: 0f,
                    relative_path = store.frameRelativePath(index),
                )
                // The recorder adds orientation, the measured clock residual and the keyframe
                // prior; without a sensor layer the frame is stored exactly as captured.
                val annotated = recorder?.annotate(record) ?: record
                store.appendFrame(annotated)
                listener.onFrameSaved(index, annotated)
                if (index % 10 == 0) {
                    Log.i(LOG_TAG, "saved frame $index")
                }

                // A still that was already in flight when capture stopped reaches disk after the
                // stop flush; rewrite the manifest so it lists every frame a reader can find. It
                // must go through flushManifest: a bare writeManifest() drops the session metadata
                // and the clock/imu blocks, which made the last frame's session unreadable for
                // Phase 2's acceptance.
                if (!capturing.get()) {
                    flushManifest(shutdownWriter = false)
                }

                if (capturing.get()) {
                    // Serialise captures: the next request waits until this frame is on disk.
                    handler?.postDelayed({ if (capturing.get()) captureStill() }, NEXT_CAPTURE_DELAY_MS)
                }
            } catch (t: Throwable) {
                Log.e(LOG_TAG, "saving frame failed", t)
                listener.onFailure(t.message ?: "saving frame failed")
            }
        }
    }

    /** Adds exposure and ISO to the camera's own EXIF block so a frame is readable on its own. */
    private fun writeExif(file: File, metadata: FrameRecord?, timestampNs: Long) {
        if (metadata == null) {
            return
        }
        runCatching {
            val exif = ExifInterface(file)
            if (metadata.exposure_ns > 0) {
                exif.setAttribute(
                    ExifInterface.TAG_EXPOSURE_TIME,
                    (metadata.exposure_ns / 1_000_000_000.0).toString(),
                )
            }
            if (metadata.iso > 0) {
                exif.setAttribute(ExifInterface.TAG_ISO_SPEED_RATINGS, metadata.iso.toString())
            }
            exif.setAttribute(ExifInterface.TAG_DATETIME, exifDateTime())
            exif.saveAttributes()
        }.onFailure { Log.w(LOG_TAG, "exif update failed", it) }
    }

    /**
     * EXIF wants wall-clock time, but the frame timestamp is the camera's boot-relative clock,
     * so the capture moment is written instead of the sensor timestamp.
     */
    private fun exifDateTime(): String {
        val stamp = SimpleDateFormat("yyyy:MM:dd HH:mm:ss", Locale.US)
        return stamp.format(Date(System.currentTimeMillis()))
    }

    private fun reportFps(frame: FrameRecord) {
        if (fpsWindowStartNs == 0L) {
            fpsWindowStartNs = frame.timestamp_ns
            fpsWindowFrames = 0
            return
        }
        fpsWindowFrames += 1
        val elapsedNs = frame.timestamp_ns - fpsWindowStartNs
        if (elapsedNs >= FPS_WINDOW_NS) {
            measuredFps = fpsWindowFrames * 1_000_000_000f / elapsedNs
            fpsWindowStartNs = frame.timestamp_ns
            fpsWindowFrames = 0
        }
    }

    private var measuredFps = 0f

    private fun currentFps(): Float = measuredFps

    private fun CaptureResult.toFrameRecord(relativePath: String?): FrameRecord = FrameRecord(
        timestamp_ns = get(CaptureResult.SENSOR_TIMESTAMP) ?: System.nanoTime(),
        exposure_ns = get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: 0L,
        iso = get(CaptureResult.SENSOR_SENSITIVITY) ?: 0,
        focus_diopter = get(CaptureResult.LENS_FOCUS_DISTANCE) ?: 0f,
        relative_path = relativePath,
    )

    private companion object {
        const val LOG_TAG = "CameraCapture"
        const val RING_BUFFER_CAPACITY = 120
        const val IMAGE_READER_BUFFERS = 3
        const val WARM_UP_FRAMES = 45
        const val MAX_WARM_UP_FRAMES = 240
        const val FPS_WINDOW_NS = 1_000_000_000L
        const val STATS_INTERVAL_MS = 250L
        const val NEXT_CAPTURE_DELAY_MS = 120L

        /** Camera2 exposes JPEG quality as a byte-valued key. */
        const val JPEG_QUALITY: Byte = 95
    }
}
