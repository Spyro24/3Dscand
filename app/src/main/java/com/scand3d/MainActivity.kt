package com.scand3d

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import com.scand3d.ui.CaptureRoute
import com.scand3d.ui.theme.Scand3dTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            Scand3dTheme {
                CaptureRoute()
            }
        }
    }
}
