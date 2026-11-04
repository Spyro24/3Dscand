package com.scand3d.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
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
import android.os.SystemClock
import android.util.Log
import android.util.Range
import android.util.Size
import android.view.Surface
import java.io.ByteArrayOutputStream
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executor
import java.util.concurrent.Executors
import java.util.concurrent.RejectedExecutionException
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

        /** One downscaled gray preview frame for tracking; at most [ANALYSIS_INTERVAL_MS] apart. */
        fun onAnalysisFrame(width_px: Int, height_px: Int, gray: ByteArray)

        /** A saved frame averages this luma (0..255); reports at most once per capture (D052). */
        fun onLowLight(mean_luma: Int)
        fun onCameraLost(reason: String)
        fun onFailure(message: String)
    }

    private val cameraManager = context.applicationContext.getSystemService(CameraManager::class.java)
    private val appContext = context.applicationContext

    /** Null when the device has no usable back camera; the UI reports this instead of crashing. */
    val initialisationError: String?

    val previewSizePx: Size
    val stillSizePx: Size

    /** Gray analysis stream the tracker matches against; 0x0 when the camera has no YUV sizes. */
    val analysisSizePx: Size

    /** Camera2's timestamp source: "realtime" means the same clock as the sensors. */
    val cameraClockSource: String

    private val cameraId: String?

    /** Sensor image rotation for this camera; the overlay extrinsic needs it (roadmap 5.2). */
    val sensorOrientationDeg: Int

    /** Camera focal length in millimeters, 0 when the device did not report one. */
    val focalLengthMm: Float

    /** Long edge of the sensor in millimeters, 0 when the device did not report one. */
    val sensorLongEdgeMm: Double

    /** Longest exposure the sensor accepts for a still, 0 when unreported (D052). */
    private val maxStillExposureNs: Long

    private var lowLightReported = false
    private val fpsRange: Range<Int>?
    private var cameraThread: HandlerThread? = null
    private var handler: Handler? = null
    private var cameraDevice: CameraDevice? = null
    private var deviceStateCallback: CameraDevice.StateCallback? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null
    private var analysisReader: ImageReader? = null
    private var lastAnalysisMs = 0L
    private var previewSurface: Surface? = null
    private var lockedSettings: LockedCaptureSettings? = null
    private var pendingStill: FrameRecord? = null

    private val ringBuffer = FrameRingBuffer(RING_BUFFER_CAPACITY)
    private val capturing = AtomicBoolean(false)
    private var sessionStore: SessionStore? = null
    private val savedFrames = AtomicInteger(0)

    /** Stills that were captured but not worth a file since the last stored frame (D050). */
    private var stillsSinceKept = 0

    // JPEG writes run here so a 3 MB write never stalls the camera callbacks.
    private val fileWriter = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "frame-writer").apply { isDaemon = true }
    }
    private var warmUpFrames = 0
    // Manual slider values (D059); 0 / negative mean "keep the locked value".
    private var manualExposureNs = 0L
    private var manualFocusDiopter = -1f
    private var fpsWindowStartNs = 0L
    private var fpsWindowFrames = 0
    private var lastStatsReportMs = 0L

    init {
        var resolvedId: String? = null
        var preview = Size(1280, 720)
        var still = Size(1920, 1080)
        var analysis = Size(0, 0)
        var orientation = 90
        var fps: Range<Int>? = null
        var clockSource = "unknown"
        var error: String? = null
        var focal_mm = 0f
        var sensor_long_mm = 0.0
        var max_exposure_ns = 0L

        try {
            resolvedId = findBackCameraId()
            val characteristics = cameraManager.getCameraCharacteristics(resolvedId)
            val map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            if (map == null) {
                error = "camera reports no stream configuration"
            } else {
                preview = choosePreviewSize(map.getOutputSizes(SurfaceTexture::class.java) ?: emptyArray())
                still = chooseStillSize(map.getOutputSizes(ImageFormat.JPEG) ?: emptyArray())
                // Tracking needs a small gray stream; the full-resolution stills are for the map.
                analysis = chooseAnalysisSize(map.getOutputSizes(ImageFormat.YUV_420_888) ?: emptyArray())
                fps = chooseFpsRange(characteristics)
            }
            orientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90
            // Stills do not have to fit the 30 fps preview, so they may expose for longer; this is
            // what makes a dim room recordable instead of black (D052).
            max_exposure_ns =
                characteristics.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE)?.upper
                    ?: 0L
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
        analysisSizePx = analysis
        sensorOrientationDeg = orientation
        fpsRange = fps
        cameraClockSource = clockSource
        focalLengthMm = focal_mm
        sensorLongEdgeMm = sensor_long_mm
        maxStillExposureNs = max_exposure_ns
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

    /** Manual exposure for the stills; 0 keeps the locked value (D059). */
    fun setManualExposure(ns: Long) {
        handler?.post { manualExposureNs = if (ns > 0L) ns else 0L }
    }

    /** Manual focus in dioptres for the stills; negative keeps the locked value (D059). */
    fun setManualFocus(diopter: Float) {
        handler?.post { manualFocusDiopter = if (diopter >= 0f) diopter else -1f }
    }

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

        // The analysis stream is an extra output, so a camera that cannot provide one simply runs
        // without tracking rather than failing the session.
        analysisReader = analysisSizePx
            .takeIf { it.width > 0 && it.height > 0 }
            ?.let { size ->
                ImageReader.newInstance(
                    size.width,
                    size.height,
                    ImageFormat.YUV_420_888,
                    ANALYSIS_BUFFERS,
                ).also { reader ->
                    reader.setOnImageAvailableListener({ onAnalysisAvailable(it) }, cameraHandler)
                }
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
        runCatching { analysisReader?.close() }
        captureSession = null
        cameraDevice = null
        imageReader = null
        analysisReader = null
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
            stillsSinceKept = 0
            lowLightReported = false
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

    /**
     * Hands work to the frame writer, or runs it inline once that writer is gone. `release()` and a
     * camera loss shut the pool down, and a still that was already in flight arrives after that;
     * submitting it threw RejectedExecutionException on the camera thread and took the app down
     * with it (measured on the SM-G930F, D055).
     */
    private fun submitWrite(block: () -> Unit) {
        try {
            fileWriter.execute(block)
        } catch (rejected: RejectedExecutionException) {
            Log.w(LOG_TAG, "frame writer closed, writing inline", rejected)
            runCatching(block)
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
            runCatching { analysisReader?.close() }
            captureSession = null
            cameraDevice = null
            imageReader = null
            analysisReader = null
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

    private fun chooseAnalysisSize(sizes: Array<Size>): Size {
        // Tracking matches features, so 640 px is plenty and keeps the per-frame cost small; 4:3
        // matches the preview and the stills, so the intrinsics scale without a crop.
        sizes.firstOrNull { it.width == 640 && it.height == 480 }?.let { return it }
        sizes.filter { it.width * 3 == it.height * 4 && it.width in 320..1280 }
            .maxByOrNull { it.width * it.height }
            ?.let { return it }
        return sizes.minByOrNull { it.width * it.height } ?: Size(0, 0)
    }

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

        val outputs = mutableListOf(
            OutputConfiguration(preview),
            OutputConfiguration(reader.surface),
        )
        analysisReader?.let { outputs.add(OutputConfiguration(it.surface)) }
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
        // A surface the session knows but no request targets never receives a frame, so the
        // analysis stream has to be a target of every repeating preview request (D051).
        analysisReader?.let { request.addTarget(it.surface) }
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
        // A surface the session knows but no request targets never receives a frame, so the
        // analysis stream has to be a target of every repeating preview request (D051).
        analysisReader?.let { request.addTarget(it.surface) }
        fpsRange?.let { request.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
        applySettings(request, settings)
        runCatching { session.setRepeatingRequest(request.build(), previewCallback, handler) }
            .onFailure { Log.e(LOG_TAG, "locked repeating request failed", it) }
    }

    /**
     * Still exposure for a locked preview exposure. Stills run one at a time, so they can trade
     * shutter for light; the multiplier is bounded so a scan does not turn into a blurry smear.
     */
    private fun stillExposureNs(locked_ns: Long): Long {
        if (locked_ns <= 0L) {
            return 0L
        }
        val target = minOf((locked_ns * STILL_LIGHT_GAIN).toLong(), STILL_MAX_EXPOSURE_NS)
        val capped = if (maxStillExposureNs > 0L) minOf(target, maxStillExposureNs) else target
        return maxOf(capped, locked_ns)
    }

    /**
     * Mean luma of a still, read from a heavily downsampled decode. A frame this dark has no
     * corners to extract, so the scan that follows can never reconstruct (D052).
     */
    private fun meanLuma(bytes: ByteArray): Int {
        val options = BitmapFactory.Options().apply { inSampleSize = LUMA_SAMPLE_SIZE }
        val thumb = BitmapFactory.decodeByteArray(bytes, 0, bytes.size, options) ?: return -1
        return try {
            val pixels = IntArray(thumb.width * thumb.height)
            thumb.getPixels(pixels, 0, thumb.width, 0, 0, thumb.width, thumb.height)
            var sum = 0L
            for (colour in pixels) {
                val red = (colour shr 16) and 0xFF
                val green = (colour shr 8) and 0xFF
                val blue = colour and 0xFF
                sum += (red * 77 + green * 150 + blue * 29) shr 8
            }
            if (pixels.isEmpty()) -1 else (sum / pixels.size).toInt()
        } finally {
            thumb.recycle()
        }
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
        // A slider only overrides the value it was moved for; exposure, gain, white balance and the
        // other axis stay locked, so a scan keeps one photometric setting (D059).
        if (manualExposureNs > 0L) {
            request.set(CaptureRequest.SENSOR_EXPOSURE_TIME, manualExposureNs)
        }
        if (manualFocusDiopter >= 0f) {
            request.set(CaptureRequest.LENS_FOCUS_DISTANCE, manualFocusDiopter)
        }
        request.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
        request.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF)
    }

    /** Longest a still may expose; beyond this the phone's frame timing falls apart (D052). */
    private val MAX_STILL_EXPOSURE_NS = 100_000_000L

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
            // The lock was taken from a preview that has to fit 30 fps, so it is never longer than
            // about 33 ms. A still may expose for longer, which is the difference between a frame
            // with corners and a black one in dim light (D052).
            val still_exposure_ns = when {
                manualExposureNs > 0L -> minOf(manualExposureNs, MAX_STILL_EXPOSURE_NS)
                lockedSettings != null -> stillExposureNs(lockedSettings!!.exposure_ns)
                else -> 0L
            }
            if (still_exposure_ns > 0L) {
                set(CaptureRequest.SENSOR_EXPOSURE_TIME, still_exposure_ns)
                set(CaptureRequest.SENSOR_FRAME_DURATION, still_exposure_ns + STILL_FRAME_MARGIN_NS)
            }
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

        // Deciding, downscaling and writing run off the camera thread: done inline they dropped the
        // preview to a few frames per second while a burst was running.
        submitWrite { persistStill(bytes, metadata, timestamp) }
    }

    /**
     * Copies the luma plane of one analysis frame. The plane is row-padded, so each row is copied
     * on its own; handing the padded buffer to the matcher would shear every feature position.
     */
    private fun onAnalysisAvailable(reader: ImageReader) {
        val image = runCatching { reader.acquireNextImage() }.getOrNull() ?: return
        try {
            val now = SystemClock.elapsedRealtime()
            if (now - lastAnalysisMs < ANALYSIS_INTERVAL_MS) {
                return
            }
            lastAnalysisMs = now

            val plane = image.planes[0]
            val width = image.width
            val height = image.height
            val row_stride = plane.rowStride
            val buffer = plane.buffer
            if (width <= 0 || height <= 0 || row_stride < width) {
                return
            }
            val gray = ByteArray(width * height)
            for (row in 0 until height) {
                val offset = row * row_stride
                if (offset + width > buffer.limit()) {
                    break
                }
                buffer.position(offset)
                buffer.get(gray, row * width, width)
            }
            listener.onAnalysisFrame(width, height, gray)
        } catch (t: Throwable) {
            Log.w(LOG_TAG, "analysis frame failed", t)
        } finally {
            image.close()
        }
    }

    /**
     * Runs on the frame writer: annotate the still, decide whether it is worth disk, store it, then
     * ask for the next one. Only keyframes are stored (D050); the other stills cost about 3 MB each
     * at full resolution and no later stage ever reads them.
     */
    private fun persistStill(bytes: ByteArray, metadata: FrameRecord?, timestamp: Long) {
        val store = sessionStore ?: return
        try {
            val record = FrameRecord(
                timestamp_ns = if (timestamp > 0) timestamp else (metadata?.timestamp_ns ?: 0L),
                exposure_ns = metadata?.exposure_ns ?: 0L,
                iso = metadata?.iso ?: 0,
                focus_diopter = metadata?.focus_diopter ?: 0f,
                relative_path = null,
            )
            // The recorder adds orientation, the measured clock residual and the keyframe prior;
            // without a sensor layer the frame is stored exactly as captured. Every still goes
            // through it, so a skipped frame leaves the prior on the same rotation reference.
            val annotated = recorder?.annotate(record) ?: record
            val keep = SessionStoragePolicy.shouldKeep(
                has_keyframe_prior = recorder != null,
                keyframe_candidate = annotated.keyframe_candidate,
                stills_since_kept = stillsSinceKept,
                kept = savedFrames.get(),
            )

            val mean_luma = meanLuma(bytes)
            if (mean_luma in 0 until MIN_MEAN_LUMA && !lowLightReported) {
                lowLightReported = true
                Log.w(LOG_TAG, "frame luma $mean_luma/255: too dark for features")
                listener.onLowLight(mean_luma)
            }

            if (!keep) {
                stillsSinceKept += 1
            } else {
                val index = savedFrames.incrementAndGet()
                stillsSinceKept = 0
                val file = store.frameFile(index)
                file.writeBytes(encodedForStorage(bytes))
                writeExif(file, annotated, timestamp)

                val stored = annotated.copy(relative_path = store.frameRelativePath(index))
                store.appendFrame(stored)
                listener.onFrameSaved(index, stored)
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
            }

            if (!capturing.get()) {
                return
            }
            if (savedFrames.get() >= SessionStoragePolicy.MAX_FRAMES) {
                // The scan holds a full set of keyframes; the preview stays up until the user stops.
                Log.i(LOG_TAG, "session holds ${savedFrames.get()} frames, no further stills")
                return
            }
            // Serialise captures: the next request waits until this frame is decided.
            handler?.postDelayed({ if (capturing.get()) captureStill() }, NEXT_CAPTURE_DELAY_MS)
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "saving frame failed", t)
            listener.onFailure(t.message ?: "saving frame failed")
        }
    }

    /**
     * Re-encodes a still for storage. The camera writes full-resolution JPEGs and the pipeline
     * decodes them at 480 px or less, so pixels past 1600 px on the long edge are disk cost only
     * (D050). Returns the original bytes when the JPEG cannot be read, rather than dropping a frame.
     */
    private fun encodedForStorage(bytes: ByteArray): ByteArray {
        val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
        BitmapFactory.decodeByteArray(bytes, 0, bytes.size, bounds)
        if (bounds.outWidth <= 0 || bounds.outHeight <= 0) {
            return bytes
        }
        val options = BitmapFactory.Options().apply {
            inSampleSize = SessionStoragePolicy.sampleSize(maxOf(bounds.outWidth, bounds.outHeight))
        }
        val decoded = BitmapFactory.decodeByteArray(bytes, 0, bytes.size, options) ?: return bytes
        return try {
            val (width, height) = SessionStoragePolicy.scaledSize(decoded.width, decoded.height)
            val scaled = if (width == decoded.width && height == decoded.height) {
                decoded
            } else {
                Bitmap.createScaledBitmap(decoded, width, height, true)
            }
            try {
                ByteArrayOutputStream(bytes.size / 8).use { out ->
                    scaled.compress(Bitmap.CompressFormat.JPEG, SessionStoragePolicy.JPEG_QUALITY, out)
                    out.toByteArray()
                }
            } finally {
                if (scaled !== decoded) {
                    scaled.recycle()
                }
            }
        } finally {
            decoded.recycle()
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
        const val ANALYSIS_BUFFERS = 2

        /** 10 Hz is the most tracking needs; the preview runs at 30. */
        const val ANALYSIS_INTERVAL_MS = 100L

        /** A still may expose this much longer than the 30 fps preview could (D052). */
        const val STILL_LIGHT_GAIN = 4.0
        const val STILL_MAX_EXPOSURE_NS = 100_000_000L
        const val STILL_FRAME_MARGIN_NS = 8_000_000L

        /** Below this a frame is too dark for the feature detector to find corners. */
        const val MIN_MEAN_LUMA = 45
        const val LUMA_SAMPLE_SIZE = 32
        const val WARM_UP_FRAMES = 45
        const val MAX_WARM_UP_FRAMES = 240
        const val FPS_WINDOW_NS = 1_000_000_000L
        const val STATS_INTERVAL_MS = 250L
        const val NEXT_CAPTURE_DELAY_MS = 120L

        /** Camera2 exposes JPEG quality as a byte-valued key. */
        const val JPEG_QUALITY: Byte = 95
    }
}
