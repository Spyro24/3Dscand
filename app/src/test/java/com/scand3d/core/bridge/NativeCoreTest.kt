package com.scand3d.core.bridge

import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * Runs on the host JVM, where the arm64-v8a library cannot load. This pins the degradation path:
 * a missing native core must surface as a null result, never as an exception at the call site.
 */
class NativeCoreTest {

    @Test
    fun `load failure is captured instead of thrown`() {
        assertFalse(NativeCore.isLoaded)
        assertNull(NativeCore.coreVersionOrNull())
    }
}
