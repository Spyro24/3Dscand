package com.scand3d.sensors

import java.io.File

/**
 * The measured clock relationship for one phone model, kept across sessions (roadmap 2.3 asks
 * for a per-device offset). It is deliberately a flat `key=value` file rather than JSON: this
 * class is covered by host unit tests, where the platform `org.json` classes are stubs, and a
 * file this small does not justify a test-only dependency. Callers log failures; the store only
 * reports them.
 */
data class DeviceClockProfile(
    val device_model: String,
    val camera_clock_source: String,
    val sensor_clock: String,
    val offset_ns: Long,
    val offset_mad_ns: Long,
    val samples: Int,
    val imu_mode: String,
    val measured_at_ms: Long,
)

class DeviceClockProfileStore(private val rootDirectory: File) {

    fun fileFor(device_model: String): File =
        File(rootDirectory, "${sanitizeFileName(device_model)}.profile")

    fun load(device_model: String): DeviceClockProfile? {
        val file = fileFor(device_model)
        if (!file.isFile) {
            return null
        }
        return runCatching {
            val values = file.readLines()
                .mapNotNull { line ->
                    val separator = line.indexOf('=')
                    if (separator <= 0) null else line.substring(0, separator) to line.substring(separator + 1)
                }
                .toMap()

            DeviceClockProfile(
                device_model = values.getValue(KEY_DEVICE_MODEL),
                camera_clock_source = values.getValue(KEY_CAMERA_CLOCK_SOURCE),
                sensor_clock = values.getValue(KEY_SENSOR_CLOCK),
                offset_ns = values.getValue(KEY_OFFSET_NS).toLong(),
                offset_mad_ns = values.getValue(KEY_OFFSET_MAD_NS).toLong(),
                samples = values.getValue(KEY_SAMPLES).toInt(),
                imu_mode = values.getValue(KEY_IMU_MODE),
                measured_at_ms = values.getValue(KEY_MEASURED_AT_MS).toLong(),
            )
        }.getOrNull()
    }

    fun save(profile: DeviceClockProfile): Boolean {
        val file = fileFor(profile.device_model)
        return runCatching {
            rootDirectory.mkdirs()
            file.writeText(
                buildString {
                    appendLine("$KEY_SCHEMA=$SCHEMA_VERSION")
                    appendLine("$KEY_DEVICE_MODEL=${profile.device_model}")
                    appendLine("$KEY_CAMERA_CLOCK_SOURCE=${profile.camera_clock_source}")
                    appendLine("$KEY_SENSOR_CLOCK=${profile.sensor_clock}")
                    appendLine("$KEY_OFFSET_NS=${profile.offset_ns}")
                    appendLine("$KEY_OFFSET_MAD_NS=${profile.offset_mad_ns}")
                    appendLine("$KEY_SAMPLES=${profile.samples}")
                    appendLine("$KEY_IMU_MODE=${profile.imu_mode}")
                    appendLine("$KEY_MEASURED_AT_MS=${profile.measured_at_ms}")
                },
            )
            true
        }.getOrDefault(false)
    }

    companion object {
        /** Device models reach this from `Build.MODEL`, so only filename-safe characters survive. */
        fun sanitizeFileName(device_model: String): String =
            device_model.replace(Regex("[^A-Za-z0-9._-]"), "_").ifEmpty { "unknown" }

        private const val SCHEMA_VERSION = 1
        private const val KEY_SCHEMA = "schema"
        private const val KEY_DEVICE_MODEL = "device_model"
        private const val KEY_CAMERA_CLOCK_SOURCE = "camera_clock_source"
        private const val KEY_SENSOR_CLOCK = "sensor_clock"
        private const val KEY_OFFSET_NS = "offset_ns"
        private const val KEY_OFFSET_MAD_NS = "offset_mad_ns"
        private const val KEY_SAMPLES = "samples"
        private const val KEY_IMU_MODE = "imu_mode"
        private const val KEY_MEASURED_AT_MS = "measured_at_ms"
    }
}
