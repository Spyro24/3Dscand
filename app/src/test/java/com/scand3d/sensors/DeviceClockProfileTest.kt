package com.scand3d.sensors

import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import java.io.File
import java.nio.file.Files

class DeviceClockProfileTest {

    private lateinit var root: File

    private val profile = DeviceClockProfile(
        device_model = "SM-G930F",
        camera_clock_source = "realtime",
        sensor_clock = "elapsed_realtime_ns",
        offset_ns = -1_234_567,
        offset_mad_ns = 800_000,
        samples = 64,
        imu_mode = "NineAxis",
        measured_at_ms = 1_762_000_000_000,
    )

    @Before
    fun setUp() {
        root = Files.createTempDirectory("clock-profiles").toFile()
    }

    @After
    fun tearDown() {
        root.deleteRecursively()
    }

    @Test
    fun `saves and reloads a profile`() {
        val store = DeviceClockProfileStore(root)

        assertTrue(store.save(profile))
        assertEquals(profile, store.load("SM-G930F"))
    }

    @Test
    fun `creates its directory on first save`() {
        val nested = File(root, "devices")
        val store = DeviceClockProfileStore(nested)

        assertTrue(store.save(profile))
        assertTrue(nested.isDirectory)
    }

    @Test
    fun `a missing profile loads as null`() {
        assertNull(DeviceClockProfileStore(root).load("nothing-here"))
    }

    @Test
    fun `device models become safe file names`() {
        assertEquals("SM-G930F", DeviceClockProfileStore.sanitizeFileName("SM-G930F"))
        assertEquals("Pixel_7_Pro", DeviceClockProfileStore.sanitizeFileName("Pixel 7/Pro"))
        assertEquals("unknown", DeviceClockProfileStore.sanitizeFileName(""))
    }

    @Test
    fun `a truncated profile is ignored instead of crashing`() {
        val store = DeviceClockProfileStore(root)
        store.fileFor("broken").writeText("device_model=broken\noffset_ns=12\n")

        assertNull(store.load("broken"))
    }

    @Test
    fun `an unwritable location reports failure`() {
        val blocker = File(root, "blocked")
        blocker.writeText("not a directory")
        val store = DeviceClockProfileStore(File(blocker, "devices"))

        assertFalse(store.save(profile))
    }
}
