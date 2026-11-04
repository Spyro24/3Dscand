package com.scand3d.ui

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.net.Uri
import android.opengl.GLSurfaceView
import android.provider.Settings
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.scand3d.R
import com.scand3d.render.MonochromePreviewRenderer
import com.scand3d.render.OverlayFeed
import com.scand3d.sensors.ImuMode
import com.scand3d.sensors.MotionWarning

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

            else -> {
                if (state.previewWidthPx > 0 && state.previewHeightPx > 0) {
                    CameraPreview(
                        widthPx = state.previewWidthPx,
                        heightPx = state.previewHeightPx,
                        onTextureReady = viewModel::onPreviewTextureReady,
                    )
                }
                CaptureOverlay(
                    state = state,
                    onToggleCapture = viewModel::toggleCapture,
                    onOverlayEnabledChange = viewModel::setOverlayEnabled,
                    onOverlayOpacityChange = viewModel::setOverlayOpacity,
                )
            }
        }
    }
}

@Composable
private fun CameraPreview(
    widthPx: Int,
    heightPx: Int,
    onTextureReady: (SurfaceTexture) -> Unit,
) {
    AndroidView(
        modifier = Modifier.fillMaxSize(),
        factory = { context ->
            GLSurfaceView(context).apply {
                setEGLContextClientVersion(2)
                val renderer = MonochromePreviewRenderer(widthPx, heightPx, onTextureReady)
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

@Composable
private fun CaptureOverlay(
    state: CaptureUiState,
    onToggleCapture: () -> Unit,
    onOverlayEnabledChange: (Boolean) -> Unit,
    onOverlayOpacityChange: (Float) -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .safeDrawingPadding()
            .padding(20.dp),
        verticalArrangement = Arrangement.SpaceBetween,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        StatusPanel(state)
        OverlayControls(
            state = state,
            onEnabledChange = onOverlayEnabledChange,
            onOpacityChange = onOverlayOpacityChange,
        )
        PrimaryCaptureButton(state = state, onToggleCapture = onToggleCapture)
    }
}

/** Overlay toggle and opacity (roadmap 5.4); both act on the renderer, not on capture. */
@Composable
private fun OverlayControls(
    state: CaptureUiState,
    onEnabledChange: (Boolean) -> Unit,
    onOpacityChange: (Float) -> Unit,
) {
    val opacityLabel = stringResource(R.string.overlay_opacity_label)

    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center,
            modifier = Modifier.fillMaxWidth(),
        ) {
            Text(
                text = stringResource(R.string.overlay_toggle_label),
                color = Color.White,
                style = MaterialTheme.typography.bodyMedium,
            )
            Switch(
                checked = state.overlayEnabled,
                onCheckedChange = onEnabledChange,
                modifier = Modifier.padding(start = 12.dp),
            )
        }
        Slider(
            value = state.overlayOpacity,
            onValueChange = onOpacityChange,
            valueRange = 0f..1f,
            enabled = state.overlayEnabled,
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 8.dp)
                .semantics { contentDescription = opacityLabel },
        )
    }
}

@Composable
private fun StatusPanel(state: CaptureUiState) {
    val statusText = when (state.phase) {
        CapturePhase.Starting -> stringResource(R.string.capture_status_starting)
        CapturePhase.Preview -> stringResource(R.string.capture_status_warming)
        CapturePhase.Locked, CapturePhase.Capturing -> stringResource(R.string.capture_status_locked)
        else -> stringResource(R.string.state_idle)
    }

    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(
            text = statusText,
            color = Color.White,
            style = MaterialTheme.typography.titleMedium,
        )
        Text(
            text = stringResource(R.string.capture_status_fps, state.fps),
            color = MaterialTheme.colorScheme.outline,
            style = MaterialTheme.typography.bodySmall,
        )
        Text(
            text = stringResource(R.string.capture_status_frames, state.savedFrames),
            color = MaterialTheme.colorScheme.outline,
            style = MaterialTheme.typography.bodySmall,
        )
        state.locked?.let { locked ->
            Text(
                text = stringResource(
                    R.string.capture_locked_values,
                    locked.exposure_ns / 1_000_000.0,
                    locked.iso,
                ),
                color = MaterialTheme.colorScheme.outline,
                style = MaterialTheme.typography.bodySmall,
            )
        }
        if (state.keyframe_candidates > 0) {
            Text(
                text = stringResource(R.string.capture_status_keyframes, state.keyframe_candidates),
                color = MaterialTheme.colorScheme.outline,
                style = MaterialTheme.typography.bodySmall,
            )
        }
        // agents.md section 4: say so when the fallback is active instead of showing a pose the
        // device cannot actually know.
        if (state.imu_mode == null || state.imu_mode == ImuMode.Unavailable) {
            Text(
                text = stringResource(R.string.capture_imu_unavailable),
                color = MaterialTheme.colorScheme.outline,
                style = MaterialTheme.typography.bodySmall,
                textAlign = TextAlign.Center,
            )
        } else if (state.imu_reduced_accuracy) {
            Text(
                text = stringResource(R.string.capture_imu_reduced_accuracy),
                color = MaterialTheme.colorScheme.outline,
                style = MaterialTheme.typography.bodySmall,
                textAlign = TextAlign.Center,
            )
        }
        if (state.motion == MotionWarning.BlurRisk) {
            Text(
                text = stringResource(R.string.capture_motion_blur),
                color = MaterialTheme.colorScheme.error,
                style = MaterialTheme.typography.bodySmall,
            )
        }
    }
}

@Composable
private fun PrimaryCaptureButton(state: CaptureUiState, onToggleCapture: () -> Unit) {
    val capturing = state.phase == CapturePhase.Capturing
    val enabled = state.phase == CapturePhase.Locked || state.phase == CapturePhase.Capturing ||
        state.phase == CapturePhase.Preview
    val label = if (capturing) {
        stringResource(R.string.capture_action_stop)
    } else {
        stringResource(R.string.capture_action_start)
    }

    // The button's own text is its accessibility label; adding a separate semantics node
    // would split the clickable node and hide the action from TalkBack.
    Button(
        onClick = onToggleCapture,
        enabled = enabled,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Text(text = label, style = MaterialTheme.typography.titleMedium)
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
            .safeDrawingPadding()
            .padding(24.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text(text = title, color = Color.White, style = MaterialTheme.typography.headlineSmall)
        Text(
            text = body,
            color = MaterialTheme.colorScheme.outline,
            style = MaterialTheme.typography.bodyMedium,
            textAlign = TextAlign.Center,
            modifier = Modifier.padding(vertical = 12.dp),
        )
        Button(onClick = onAction, modifier = Modifier.fillMaxWidth()) {
            Text(text = actionLabel)
        }
    }
}
