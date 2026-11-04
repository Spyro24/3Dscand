package com.scand3d.ui

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.graphics.SurfaceTexture
import android.net.Uri
import android.opengl.GLSurfaceView
import android.provider.Settings
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.Slider
import androidx.compose.material3.SliderDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.scand3d.R
import com.scand3d.render.MonochromePreviewRenderer
import com.scand3d.render.OverlayFeed
import com.scand3d.sensors.ImuMode
import com.scand3d.sensors.MotionWarning
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.File

/**
 * Camera-app layout (D041): the preview fills the top of the screen edge to edge, a control strip
 * with the focus indicator, zoom value and overlay toggle sits under it, and the shutter row and
 * mode tabs occupy the panel below. The scan has no photo gallery, video mode or second camera, so
 * those slots are drawn as the reference shows them and are not interactive.
 */
@Composable
fun CaptureRoute(viewModel: CaptureViewModel = viewModel()) {
    val state by viewModel.state.collectAsStateWithLifecycle()
    val context = LocalContext.current
    var granted by remember {
        mutableStateOf(
            ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) ==
                PackageManager.PERMISSION_GRANTED,
        )
    }

    val launcher = rememberLauncherForActivityResult(ActivityResultContracts.RequestPermission()) { allowed ->
        granted = allowed
        if (allowed) viewModel.onPermissionGranted() else viewModel.onPermissionDenied()
    }

    // Ask once on entry, then hand control to the camera layer.
    LaunchedEffect(granted) {
        if (granted) viewModel.onPermissionGranted() else launcher.launch(Manifest.permission.CAMERA)
    }

    val openSettings = {
        context.startActivity(
            Intent(
                Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                Uri.fromParts("package", context.packageName, null),
            ),
        )
        Unit
    }

    Box(modifier = Modifier.fillMaxSize().background(Color.Black)) {
        when (state.phase) {
            CapturePhase.WaitingForPermission -> Unit

            CapturePhase.PermissionDenied -> MessageScreen(
                title = stringResource(R.string.permission_denied_title),
                body = stringResource(R.string.permission_denied_body),
                actionLabel = stringResource(R.string.capture_action_settings),
                onAction = openSettings,
            )

            CapturePhase.CameraLost -> MessageScreen(
                title = stringResource(R.string.camera_lost_title),
                body = state.message.orEmpty(),
                actionLabel = stringResource(R.string.capture_action_retry),
                onAction = viewModel::retry,
            )

            CapturePhase.Failed -> MessageScreen(
                title = stringResource(R.string.camera_failed_title),
                body = state.message.orEmpty(),
                actionLabel = stringResource(R.string.capture_action_retry),
                onAction = viewModel::retry,
            )

            else -> CameraScreen(
                state = state,
                onTextureReady = viewModel::onPreviewTextureReady,
                onToggleCapture = viewModel::toggleCapture,
                onOverlayEnabledChange = viewModel::setOverlayEnabled,
                onOverlayOpacityChange = viewModel::setOverlayOpacity,
            )
        }
    }
}

@Composable
private fun CameraScreen(
    state: CaptureUiState,
    onTextureReady: (SurfaceTexture) -> Unit,
    onToggleCapture: () -> Unit,
    onOverlayEnabledChange: (Boolean) -> Unit,
    onOverlayOpacityChange: (Float) -> Unit,
) {
    Column(modifier = Modifier.fillMaxSize().background(Color.Black)) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f)
                .background(Color.Black),
        ) {
            if (state.previewWidthPx > 0 && state.previewHeightPx > 0) {
                CameraPreview(
                    widthPx = state.previewWidthPx,
                    heightPx = state.previewHeightPx,
                    rotationDeg = state.previewRotationDeg,
                    onTextureReady = onTextureReady,
                )
            }
            StatusLine(
                state = state,
                modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 10.dp),
            )
        }

        ControlStrip(state = state, onOverlayEnabledChange = onOverlayEnabledChange)
        if (state.overlayEnabled) {
            OpacitySlider(state = state, onOverlayOpacityChange = onOverlayOpacityChange)
        }
        Box(modifier = Modifier.fillMaxWidth().height(1.dp).background(DIVIDER_COLOR))
        CaptureControls(
            state = state,
            onToggleCapture = onToggleCapture,
        )
        ModeTabs()
    }
}

@Composable
private fun CameraPreview(
    widthPx: Int,
    heightPx: Int,
    rotationDeg: Int,
    onTextureReady: (SurfaceTexture) -> Unit,
) {
    AndroidView(
        modifier = Modifier.fillMaxSize(),
        factory = { context ->
            GLSurfaceView(context).apply {
                setEGLContextClientVersion(2)
                val renderer =
                    MonochromePreviewRenderer(widthPx, heightPx, rotationDeg, onTextureReady)
                renderer.bindRequestRender {
                    // Redraw only when the camera delivers a frame.
                    requestRender()
                }
                setRenderer(renderer)
                renderMode = GLSurfaceView.RENDERMODE_WHEN_DIRTY
            }
        },
        // The renderer is rebuilt with the view; stop feeding the old one.
        onRelease = { OverlayFeed.bind(null, null) },
    )
}

/** One grey line over the preview: the numbers the scan needs, without a status panel. */
@Composable
private fun StatusLine(state: CaptureUiState, modifier: Modifier = Modifier) {
    val fps = if (state.fps > 0f) stringResource(R.string.capture_status_fps, state.fps) else null
    val frames =
        if (state.savedFrames > 0) stringResource(R.string.capture_status_frames, state.savedFrames) else null
    val views = if (state.keyframe_candidates > 0) {
        stringResource(R.string.capture_status_keyframes, state.keyframe_candidates)
    } else {
        null
    }
    val summary = listOfNotNull(fps, frames, views).joinToString("  ·  ")

    Column(modifier = modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        if (summary.isNotEmpty()) {
            Text(text = summary, color = SECONDARY_TEXT, fontSize = 12.sp)
        }
        if (state.motion == MotionWarning.BlurRisk) {
            Text(
                text = stringResource(R.string.capture_motion_blur),
                color = Color(0xFFFF6B6B),
                fontSize = 12.sp,
            )
        }
        if (state.imu_mode == null || state.imu_mode == ImuMode.Unavailable) {
            Text(
                text = stringResource(R.string.capture_imu_unavailable),
                color = SECONDARY_TEXT,
                fontSize = 12.sp,
                textAlign = TextAlign.Center,
            )
        }
    }
}

/** Focus state, the camera's fixed zoom, and the overlay toggle (roadmap 5.4). */
@Composable
private fun ControlStrip(state: CaptureUiState, onOverlayEnabledChange: (Boolean) -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth().padding(horizontal = 34.dp, vertical = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        FocusIndicator(locked = state.locked != null)
        ZoomValue()
        OverlayToggle(enabled = state.overlayEnabled, onChange = onOverlayEnabledChange)
    }
}

/** Exposure and focus lock indicator (roadmap 1.4); the square fills in once the scan locks. */
@Composable
private fun FocusIndicator(locked: Boolean) {
    val description = stringResource(R.string.capture_status_locked)
    val tint = if (locked) Color.White else Color(0x80FFFFFF)
    Canvas(
        modifier = Modifier
            .size(28.dp)
            .semantics { contentDescription = description },
    ) {
        val stroke = 2.dp.toPx()
        drawRoundRect(
            color = tint,
            topLeft = Offset(stroke / 2f, stroke / 2f),
            size = Size(size.width - stroke, size.height - stroke),
            cornerRadius = CornerRadius(7.dp.toPx()),
            style = Stroke(stroke),
        )
        drawCircle(color = tint, radius = 3.dp.toPx(), center = center)
    }
}

/** The camera has no zoom control, so the pill states the value instead of offering a choice. */
@Composable
private fun ZoomValue() {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(14.dp))
            .background(PILL_COLOR)
            .padding(horizontal = 16.dp, vertical = 7.dp),
    ) {
        Text(
            text = stringResource(R.string.capture_zoom_value),
            color = Color.White,
            fontSize = 14.sp,
            fontWeight = FontWeight.Medium,
        )
    }
}

@Composable
private fun OverlayToggle(enabled: Boolean, onChange: (Boolean) -> Unit) {
    val label = stringResource(R.string.overlay_toggle_label)
    val tint = if (enabled) Color.White else Color(0x66FFFFFF)
    Box(
        modifier = Modifier
            .size(40.dp)
            .clip(RoundedCornerShape(10.dp))
            .background(PANEL_COLOR)
            .border(1.dp, tint, RoundedCornerShape(10.dp))
            .clickable(role = Role.Switch) { onChange(!enabled) }
            .semantics { contentDescription = label },
        contentAlignment = Alignment.Center,
    ) {
        Canvas(modifier = Modifier.size(18.dp)) {
            val step = size.width / 3f
            for (row in 0 until 3) {
                for (column in 0 until 3) {
                    drawCircle(
                        color = tint,
                        radius = step * 0.28f,
                        center = Offset(step * (column + 0.5f), step * (row + 0.5f)),
                    )
                }
            }
        }
    }
}

@Composable
private fun OpacitySlider(state: CaptureUiState, onOverlayOpacityChange: (Float) -> Unit) {
    val label = stringResource(R.string.overlay_opacity_label)
    Slider(
        value = state.overlayOpacity,
        onValueChange = onOverlayOpacityChange,
        valueRange = 0f..1f,
        colors = SliderDefaults.colors(
            thumbColor = Color.White,
            activeTrackColor = Color(0x99FFFFFF),
            inactiveTrackColor = Color(0x33FFFFFF),
            activeTickColor = Color.Transparent,
            inactiveTickColor = Color.Transparent,
        ),
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 34.dp)
            .semantics { contentDescription = label },
    )
}

@Composable
private fun CaptureControls(state: CaptureUiState, onToggleCapture: () -> Unit) {
    val capturing = state.phase == CapturePhase.Capturing
    val enabled = state.phase == CapturePhase.Locked || capturing ||
        state.phase == CapturePhase.Preview
    val label = stringResource(
        if (capturing) R.string.capture_action_stop else R.string.capture_action_start,
    )

    Row(
        modifier = Modifier.fillMaxWidth().padding(horizontal = 34.dp, vertical = 10.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        GalleryTile(frames = state.savedFrames)
        ShutterButton(
            capturing = capturing,
            enabled = enabled,
            label = label,
            onToggleCapture = onToggleCapture,
        )
        FlipGlyph()
    }
}

/** The newest saved keyframe stands in for the gallery a camera app would offer. */
@Composable
private fun GalleryTile(frames: Int) {
    val context = LocalContext.current
    var thumbnail by remember { mutableStateOf<ImageBitmap?>(null) }
    LaunchedEffect(frames) {
        thumbnail = withContext(Dispatchers.IO) { newestThumbnail(context) }
    }

    val description = stringResource(R.string.capture_gallery_description)
    Box(
        modifier = Modifier
            .size(52.dp)
            .clip(RoundedCornerShape(10.dp))
            .background(PANEL_COLOR)
            .semantics { contentDescription = description },
        contentAlignment = Alignment.Center,
    ) {
        val image = thumbnail
        if (image != null) {
            Image(
                bitmap = image,
                contentDescription = description,
                contentScale = ContentScale.Crop,
                modifier = Modifier.fillMaxSize(),
            )
        } else {
            Canvas(modifier = Modifier.size(22.dp)) {
                val stroke = 1.5.dp.toPx()
                drawRoundRect(
                    color = Color(0x99FFFFFF),
                    topLeft = Offset(stroke / 2f, stroke / 2f),
                    size = Size(size.width - stroke, size.height - stroke),
                    cornerRadius = CornerRadius(3.dp.toPx()),
                    style = Stroke(stroke),
                )
                drawCircle(
                    color = Color(0x99FFFFFF),
                    radius = size.width * 0.1f,
                    center = Offset(size.width * 0.32f, size.height * 0.34f),
                )
                val mountain = Path().apply {
                    moveTo(size.width * 0.16f, size.height * 0.82f)
                    lineTo(size.width * 0.46f, size.height * 0.44f)
                    lineTo(size.width * 0.72f, size.height * 0.82f)
                    close()
                }
                drawPath(mountain, color = Color(0x99FFFFFF))
            }
        }
    }
}

@Composable
private fun ShutterButton(
    capturing: Boolean,
    enabled: Boolean,
    label: String,
    onToggleCapture: () -> Unit,
) {
    Box(
        modifier = Modifier
            .size(76.dp)
            .clip(CircleShape)
            .border(4.dp, if (enabled) Color.White else Color(0x66FFFFFF), CircleShape)
            .clickable(enabled = enabled, role = Role.Button, onClick = onToggleCapture)
            .semantics { contentDescription = label },
        contentAlignment = Alignment.Center,
    ) {
        Box(
            modifier = Modifier
                .size(if (capturing) 32.dp else 56.dp)
                .clip(if (capturing) RoundedCornerShape(7.dp) else CircleShape)
                .background(if (enabled) SHUTTER_COLOR else Color(0x44FFFFFF)),
        )
    }
}

/** Decorative: the device has one camera, so this slot does not switch anything. */
@Composable
private fun FlipGlyph() {
    val label = stringResource(R.string.capture_flip_label)
    Canvas(
        modifier = Modifier
            .size(30.dp)
            .semantics { contentDescription = label },
    ) {
        val stroke = 2.dp.toPx()
        val inset = stroke / 2f
        val arcSize = Size(size.width - stroke, size.height - stroke)
        drawArc(
            color = Color.White,
            startAngle = 30f,
            sweepAngle = 250f,
            useCenter = false,
            topLeft = Offset(inset, inset),
            size = arcSize,
            style = Stroke(stroke),
        )
        val head = Path().apply {
            moveTo(size.width * 0.72f, size.height * 0.02f)
            lineTo(size.width * 0.96f, size.height * 0.2f)
            lineTo(size.width * 0.68f, size.height * 0.28f)
            close()
        }
        drawPath(head, color = Color.White)
    }
}

@Composable
private fun ModeTabs() {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(start = 20.dp, end = 20.dp, bottom = 16.dp),
        horizontalArrangement = Arrangement.SpaceEvenly,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        // The app only scans; the other two slots are shown disabled rather than pretending to work.
        TabLabel(text = stringResource(R.string.capture_tab_photo), active = false)
        TabLabel(text = stringResource(R.string.capture_tab_video), active = false)
        TabLabel(text = stringResource(R.string.capture_tab_scan), active = true)
    }
}

@Composable
private fun TabLabel(text: String, active: Boolean) {
    if (active) {
        Box(
            modifier = Modifier
                .clip(CircleShape)
                .background(PILL_COLOR)
                .padding(horizontal = 18.dp, vertical = 8.dp),
        ) {
            Text(text = text, color = Color.White, fontSize = 13.sp, letterSpacing = 1.sp)
        }
    } else {
        Text(
            text = text,
            color = Color(0x59FFFFFF),
            fontSize = 13.sp,
            letterSpacing = 1.sp,
            modifier = Modifier.padding(horizontal = 18.dp, vertical = 8.dp),
        )
    }
}

@Composable
private fun MessageScreen(
    title: String,
    body: String,
    actionLabel: String,
    onAction: () -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(24.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text(text = title, color = Color.White, fontSize = 24.sp)
        Text(
            text = body,
            color = SECONDARY_TEXT,
            fontSize = 14.sp,
            textAlign = TextAlign.Center,
            modifier = Modifier.padding(vertical = 12.dp),
        )
        Button(onClick = onAction, modifier = Modifier.fillMaxWidth()) {
            Text(text = actionLabel, fontSize = 16.sp)
        }
    }
}

/** Decodes the newest keyframe small enough for a 52 dp tile. */
private fun newestThumbnail(context: Context): ImageBitmap? {
    val sessions = File(context.filesDir, SESSIONS_DIRECTORY)
    val latest = sessions.listFiles()?.filter { it.isDirectory }?.maxByOrNull { it.name } ?: return null
    val frame = File(latest, FRAMES_DIRECTORY).listFiles()
        ?.filter { it.isFile && it.extension == "jpg" }
        ?.maxByOrNull { it.name }
        ?: return null

    val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
    BitmapFactory.decodeFile(frame.absolutePath, bounds)
    var sample = 1
    while (bounds.outWidth / (sample * 2) >= THUMBNAIL_MAX_EDGE_PX) {
        sample *= 2
    }
    val bitmap = BitmapFactory.decodeFile(
        frame.absolutePath,
        BitmapFactory.Options().apply { inSampleSize = sample },
    ) ?: return null
    return bitmap.asImageBitmap()
}

private const val SESSIONS_DIRECTORY = "sessions"
private const val FRAMES_DIRECTORY = "frames"
private const val THUMBNAIL_MAX_EDGE_PX = 128

private val PANEL_COLOR = Color(0xFF15181A)
private val PILL_COLOR = Color(0xFF2C3237)
private val DIVIDER_COLOR = Color(0xFF23262A)
private val SHUTTER_COLOR = Color(0xFF5B6A73)
private val SECONDARY_TEXT = Color(0xB3FFFFFF)
