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

    /**
     * Version reported by the native core, or null when the library did not load. Calling this
     * is what proves the bridge works end to end.
     */
    fun coreVersionOrNull(): String? = if (isLoaded) nativeCoreVersion() else null

    private external fun nativeCoreVersion(): String
}
