package com.scand3d.debug

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.util.Log

/**
 * Debug-build only: `adb shell am broadcast -a com.scand3d.debug.CAPTURE --es cmd start` drives a
 * capture run without UI taps, which keeps device verification deterministic.
 */
class CaptureTestReceiver : BroadcastReceiver() {

    override fun onReceive(context: Context, intent: Intent) {
        val actions = CaptureTestBridge.actions
        if (actions == null) {
            Log.w(LOG_TAG, "capture bridge not registered, is the app in the foreground?")
            return
        }
        when (intent.getStringExtra(COMMAND)) {
            START -> {
                actions.startCapture()
                Log.i(LOG_TAG, "start requested")
            }

            STOP -> {
                actions.stopCapture()
                Log.i(LOG_TAG, "stop requested, frames=${actions.savedFrames()}")
            }

            LOSS -> {
                actions.simulateCameraLoss()
                Log.i(LOG_TAG, "simulated camera loss requested")
            }

            RECONSTRUCT -> {
                actions.reconstructLatestSession()
                Log.i(LOG_TAG, "reconstruct requested")
            }

            OVERLAY -> {
                val enabled = intent.getStringExtra(VALUE) == "on"
                actions.setOverlayDemo(enabled)
                Log.i(LOG_TAG, "overlay demo ${if (enabled) "on" else "off"}")
            }

            else -> Log.i(LOG_TAG, "status frames=${actions.savedFrames()}")
        }
    }

    private companion object {
        const val LOG_TAG = "CaptureTest"
        const val COMMAND = "cmd"
        const val VALUE = "value"
        const val START = "start"
        const val STOP = "stop"
        const val LOSS = "loss"
        const val OVERLAY = "overlay"
        const val RECONSTRUCT = "reconstruct"
    }
}
