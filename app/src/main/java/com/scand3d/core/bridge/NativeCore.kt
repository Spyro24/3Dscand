package com.scand3d.core.bridge

/**
 * The only place Kotlin reaches the native core (agents.md section 3). Loading happens once,
 * when this object is first touched, and the failure is kept instead of thrown so the UI can
 * report a broken native build rather than crashing on launch.
 */
object NativeCore {

    private const val LIBRARY_NAME = "scand3d_core"

    /** Null when the shared library loaded successfully. */
    val loadError: Throwable? = try {
        System.loadLibrary(LIBRARY_NAME)
        null
    } catch (t: UnsatisfiedLinkError) {
        t
    }

    val isLoaded: Boolean
        get() = loadError == null
}
