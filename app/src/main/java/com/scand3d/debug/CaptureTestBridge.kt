package com.scand3d.debug

/**
 * Debug-only hook so device tests can drive capture without tapping the screen. Compiled into
 * every build but inert: nothing registers as the actions unless the app's view model is alive.
 */
object CaptureTestBridge {

    interface Actions {
        fun startCapture()
        fun stopCapture()
        fun savedFrames(): Int
    }

    @Volatile
    var actions: Actions? = null
}
