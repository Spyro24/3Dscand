package com.scand3d

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.scand3d.core.bridge.NativeCore
import com.scand3d.ui.theme.Scand3dTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            Scand3dTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    FoundationScreen(modifier = Modifier.padding(innerPadding))
                }
            }
        }
    }
}

/** Placeholder for the Idle app state until Phase 1 adds the capture flow. */
@Composable
fun FoundationScreen(modifier: Modifier = Modifier) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(24.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text(
            text = stringResource(R.string.app_name),
            style = MaterialTheme.typography.headlineMedium,
        )
        Text(
            text = stringResource(R.string.state_idle),
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Text(
            text = nativeCoreStatus(),
            style = MaterialTheme.typography.labelSmall,
            color = MaterialTheme.colorScheme.outline,
        )
    }
}

/** Proves the JNI bridge answered, or reports that the native core is missing. */
@Composable
private fun nativeCoreStatus(): String {
    val version = NativeCore.coreVersionOrNull()
    return if (version == null) {
        stringResource(R.string.native_core_missing)
    } else {
        stringResource(R.string.native_core_loaded, version)
    }
}

@Preview(showBackground = true)
@Composable
private fun FoundationScreenPreview() {
    Scand3dTheme(darkTheme = true) {
        FoundationScreen()
    }
}
