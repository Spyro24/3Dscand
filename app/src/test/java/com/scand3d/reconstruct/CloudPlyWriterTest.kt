package com.scand3d.reconstruct

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder

class CloudPlyWriterTest {

    private fun tempDirectory(): File {
        val directory = File.createTempFile("cloud-ply", "")
        directory.delete()
        directory.mkdirs()
        return directory
    }

    @Test
    fun writesBinaryLittleEndianPointsBehindAValidHeader() {
        val directory = tempDirectory()
        try {
            val file = File(directory, CloudPlyWriter.FILE_NAME)
            val positions = doubleArrayOf(
                1.5, -2.0, 0.25,
                0.0, 0.5, -7.0,
            )
            assertTrue(CloudPlyWriter.write(file, positions))

            val bytes = file.readBytes()
            val headerEnd = String(bytes, Charsets.US_ASCII).indexOf("end_header\n") + "end_header\n".length
            val header = String(bytes, 0, headerEnd, Charsets.US_ASCII)
            assertTrue(header.startsWith("ply\nformat binary_little_endian 1.0\n"))
            assertTrue(header.contains("element vertex 2\n"))
            assertTrue(header.contains("property float z\n"))

            val body = ByteBuffer.wrap(bytes, headerEnd, bytes.size - headerEnd)
                .order(ByteOrder.LITTLE_ENDIAN)
            val values = DoubleArray(6) { body.getFloat().toDouble() }
            for (index in positions.indices) {
                assertEquals("value $index", positions[index], values[index], 1e-6)
            }
        } finally {
            directory.deleteRecursively()
        }
    }

    @Test
    fun anEmptyCloudIsAValidFileWithoutABody() {
        val directory = tempDirectory()
        try {
            val file = File(directory, CloudPlyWriter.FILE_NAME)
            assertTrue(CloudPlyWriter.write(file, DoubleArray(0)))
            val text = file.readText(Charsets.US_ASCII)
            assertTrue(text.contains("element vertex 0\n"))
            assertTrue(text.endsWith("end_header\n"))
            // The temporary file of the atomic rename must not survive.
            assertFalse(File(directory, CloudPlyWriter.FILE_NAME + ".tmp").exists())
        } finally {
            directory.deleteRecursively()
        }
    }

    @Test
    fun aSecondWriteReplacesTheFirstCloud() {
        val directory = tempDirectory()
        try {
            val file = File(directory, CloudPlyWriter.FILE_NAME)
            assertTrue(CloudPlyWriter.write(file, doubleArrayOf(0.0, 0.0, 1.0, 0.0, 1.0, 0.0)))
            assertTrue(CloudPlyWriter.write(file, doubleArrayOf(0.0, 0.0, 2.0)))
            val text = file.readText(Charsets.US_ASCII)
            assertTrue(text.contains("element vertex 1\n"))
            assertFalse(text.contains("element vertex 2\n"))
        } finally {
            directory.deleteRecursively()
        }
    }
}
