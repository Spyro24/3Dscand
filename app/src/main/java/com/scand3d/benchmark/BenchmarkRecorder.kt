package com.scand3d.benchmark

import android.content.Context
import android.os.Debug
import android.os.PowerManager
import org.json.JSONObject
import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

/**
 * Records frame intervals, process memory and thermal state as JSON Lines so a run can be pulled
 * off the device and summarised in `docs/benchmarks/` (roadmap 0.6).
 *
 * Writes happen on a single worker thread: the caller may be the camera or render thread, and a
 * synchronous file write there would show up in the frame times being measured.
 */
class BenchmarkRecorder(context: Context, deviceLabel: String) : Closeable {

    private val worker = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "benchmark-writer").apply { isDaemon = true }
    }
    private val writer: BufferedWriter
    private val powerManager = context.applicationContext.getSystemService(PowerManager::class.java)

    /** File inside the app's private storage; the path is relative so nothing leaks an absolute path. */
    val file: File
    val relativePath: String

    init {
        val directory = File(context.filesDir, BENCHMARK_DIRECTORY)
        directory.mkdirs()

        val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        file = File(directory, "bench-$stamp.jsonl")
        relativePath = "$BENCHMARK_DIRECTORY/${file.name}"
        writer = BufferedWriter(FileWriter(file, true))

        append(
            JSONObject()
                .put("kind", "session")
                .put("device", deviceLabel)
                .put("started_at_ms", System.currentTimeMillis()),
        )
        recordMemory()
        recordThermal()
    }

    fun recordFrameTime(frameIndex: Int, intervalMs: Double) {
        append(
            JSONObject()
                .put("kind", "frame")
                .put("frame_index", frameIndex)
                .put("interval_ms", round(intervalMs)),
        )
    }

    fun recordMemory() {
        val info = Debug.MemoryInfo()
        Debug.getMemoryInfo(info)
        append(
            JSONObject()
                .put("kind", "memory")
                .put("total_pss_kb", info.totalPss)
                .put("java_heap_kb", Runtime.getRuntime().totalMemory() / 1024L),
        )
    }

    fun recordThermal() {
        val status = if (powerManager == null) {
            THERMAL_UNSUPPORTED
        } else {
            powerManager.currentThermalStatus
        }
        append(
            JSONObject()
                .put("kind", "thermal")
                .put("status", status)
                .put("status_name", thermalStatusName(status)),
        )
    }

    override fun close() {
        worker.shutdown()
        worker.awaitTermination(2, TimeUnit.SECONDS)
        runCatching { writer.close() }
    }

    private fun append(payload: JSONObject) {
        val line = payload.put("t_ms", System.currentTimeMillis()).toString()
        // A closed recorder must not take the app down; benchmarks are diagnostic only.
        worker.execute {
            runCatching {
                writer.write(line)
                writer.newLine()
                writer.flush()
            }
        }
    }

    private fun round(value: Double): Double = Math.round(value * 1000.0) / 1000.0

    private fun thermalStatusName(status: Int): String = when (status) {
        PowerManager.THERMAL_STATUS_NONE -> "NONE"
        PowerManager.THERMAL_STATUS_LIGHT -> "LIGHT"
        PowerManager.THERMAL_STATUS_MODERATE -> "MODERATE"
        PowerManager.THERMAL_STATUS_SEVERE -> "SEVERE"
        PowerManager.THERMAL_STATUS_CRITICAL -> "CRITICAL"
        PowerManager.THERMAL_STATUS_EMERGENCY -> "EMERGENCY"
        PowerManager.THERMAL_STATUS_SHUTDOWN -> "SHUTDOWN"
        THERMAL_UNSUPPORTED -> "UNSUPPORTED"
        else -> "UNKNOWN($status)"
    }

    private companion object {
        const val BENCHMARK_DIRECTORY = "benchmarks"
        const val THERMAL_UNSUPPORTED = -1
    }
}
