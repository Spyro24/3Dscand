package com.scand3d.capture

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class SessionStoragePolicyTest {

    @Test
    fun `only keyframes are kept when the IMU prior is available`() {
        assertTrue(SessionStoragePolicy.shouldKeep(true, true, 0, 0))
        assertFalse(SessionStoragePolicy.shouldKeep(true, false, 1, 0))
    }

    @Test
    fun `a still is kept when the prior stops producing keyframes`() {
        assertFalse(SessionStoragePolicy.shouldKeep(true, false, SessionStoragePolicy.FALLBACK_STILLS - 1, 3))
        assertTrue(SessionStoragePolicy.shouldKeep(true, false, SessionStoragePolicy.FALLBACK_STILLS, 3))
    }

    @Test
    fun `without a prior every still is kept`() {
        assertTrue(SessionStoragePolicy.shouldKeep(false, false, 0, 0))
    }

    @Test
    fun `a session stops at the frame cap`() {
        val cap = SessionStoragePolicy.MAX_FRAMES
        assertTrue(SessionStoragePolicy.shouldKeep(false, false, 0, cap - 1))
        assertFalse(SessionStoragePolicy.shouldKeep(false, false, 0, cap))
        assertFalse(SessionStoragePolicy.shouldKeep(true, true, 0, cap))
    }

    @Test
    fun `decode factor keeps the long edge at or above 1600 px`() {
        assertEquals(2, SessionStoragePolicy.sampleSize(4032))
        assertEquals(1, SessionStoragePolicy.sampleSize(1920))
        assertEquals(1, SessionStoragePolicy.sampleSize(1600))
        assertEquals(4, SessionStoragePolicy.sampleSize(6400))
    }

    @Test
    fun `stored size keeps the aspect and the 1600 px long edge`() {
        assertEquals(1600 to 1200, SessionStoragePolicy.scaledSize(2016, 1512))
        assertEquals(1600 to 900, SessionStoragePolicy.scaledSize(1920, 1080))
        assertEquals(1280 to 960, SessionStoragePolicy.scaledSize(1280, 960))
        assertEquals(1200 to 1600, SessionStoragePolicy.scaledSize(3024, 4032))
    }
}
