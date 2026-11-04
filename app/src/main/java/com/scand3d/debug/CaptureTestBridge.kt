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

        /** Debug only: forces the preview's quarter turn for orientation calibration. */
        fun setPreviewRotationDeg(deg: Int)

        /** Runs the production camera-loss path (the same callback the OS invokes). */
        fun simulateCameraLoss()

        /** Shows or clears the canned overlay cloud used to verify the Phase 5 render path. */
        fun setOverlayDemo(enabled: Boolean)

        /** Reconstructs the newest saved session and loads it into the overlay. */
        fun reconstructLatestSession()

        /** Runs the dense stage on the newest saved session (roadmap 6). */
        fun runDensePass()
    }

    @Volatile
    var actions: Actions? = null
}
