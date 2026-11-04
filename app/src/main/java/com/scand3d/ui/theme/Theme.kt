package com.scand3d.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable

private val DarkColors = darkColorScheme(
    primary = Accent,
    onPrimary = Gray05,
    background = Gray05,
    onBackground = Gray90,
    surface = Gray10,
    onSurface = Gray90,
    surfaceVariant = Gray20,
    onSurfaceVariant = Gray40,
    outline = Gray40,
)

private val LightColors = lightColorScheme(
    primary = Gray20,
    onPrimary = Gray90,
)

/**
 * Dark is the default (agents.md section 4); light only follows an explicit system choice
 * so the capture screen keeps its low-glare appearance.
 */
@Composable
fun Scand3dTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    content: @Composable () -> Unit,
) {
    MaterialTheme(
        colorScheme = if (darkTheme) DarkColors else LightColors,
        typography = Typography(),
        content = content,
    )
}
