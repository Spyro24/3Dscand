package com.scand3d.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import android.util.Log

/**
 * Reads gyroscope, accelerometer, magnetometer and rotation vector on one handler thread.
 *
 * Samples are driven by the gyroscope when it exists and by the accelerometer otherwise, because
 * every sample must carry a timestamp that the fusion can integrate against. The rotation vector
 * is read too (roadmap 2.1) but only kept for diagnostics: the native filter does its own fusion,
 * and the OS value is useful as a reference when checking the result on a device.
 */
class AndroidImuSource(context: Context) : SensorSource {

    private val sensorManager = context.applicationContext.getSystemService(SensorManager::class.java)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    override val availability = ImuAvailability(
        has_gyroscope = gyroscope != null,
        has_accelerometer = accelerometer != null,
        has_magnetometer = magnetometer != null,
        has_rotation_vector = rotationVector != null,
    )

    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var listener: ((ImuSample) -> Unit)? = null

    // All fields below are written and read on the sensor thread only.
    private var gyro_rad_s = Vec3.ZERO
    private var accel_m_s2 = Vec3.ZERO
    private var mag_u_t = Vec3.ZERO
    private var accel_seen = false
    private var mag_seen = false
    private var rotation_vector_wxyz: List<Double>? = null

    private val gyroListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            gyro_rad_s = event.values.toVec3()
            emit(event.timestamp)
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    private val accelListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            accel_m_s2 = event.values.toVec3()
            accel_seen = true
            // Without a gyro the accelerometer is the clock every sample hangs off.
            if (gyroscope == null) {
                emit(event.timestamp)
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    private val magListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            mag_u_t = event.values.toVec3()
            mag_seen = true
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    private val rotationListener = object : SensorEventListener {
        private val quaternion = FloatArray(4)

        override fun onSensorChanged(event: SensorEvent) {
            SensorManager.getQuaternionFromVector(quaternion, event.values)
            rotation_vector_wxyz = quaternion.map { it.toDouble() }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    override fun start(listener: (ImuSample) -> Unit) {
        if (this.listener != null) {
            return
        }
        this.listener = listener

        val sensorThread = HandlerThread("imu-sensors").also { it.start() }
        thread = sensorThread
        val sensorHandler = Handler(sensorThread.looper)
        handler = sensorHandler

        try {
            // 5 ms for the gyro keeps integration steps short enough that a fast turn stays
            // accurate; the slower sensors do not gain anything from a higher rate.
            gyroscope?.let {
                sensorManager.registerListener(gyroListener, it, GYRO_PERIOD_US, sensorHandler)
            }
            accelerometer?.let {
                sensorManager.registerListener(accelListener, it, ACCEL_PERIOD_US, sensorHandler)
            }
            magnetometer?.let {
                sensorManager.registerListener(magListener, it, MAG_PERIOD_US, sensorHandler)
            }
            rotationVector?.let {
                sensorManager.registerListener(rotationListener, it, ROTATION_PERIOD_US, sensorHandler)
            }
        } catch (t: Throwable) {
            Log.e(LOG_TAG, "sensor registration failed", t)
            stop()
            return
        }

        Log.i(
            LOG_TAG,
            "sensors started: gyro=${gyroscope != null} accel=${accelerometer != null} " +
                "mag=${magnetometer != null} rotation_vector=${rotationVector != null}",
        )
    }

    override fun stop() {
        listener = null
        runCatching { sensorManager.unregisterListener(gyroListener) }
        runCatching { sensorManager.unregisterListener(accelListener) }
        runCatching { sensorManager.unregisterListener(magListener) }
        runCatching { sensorManager.unregisterListener(rotationListener) }
        thread?.quitSafely()
        thread = null
        handler = null
    }

    private fun emit(timestamp_ns: Long) {
        if (gyroscope == null && !accel_seen) {
            return // nothing has been measured yet
        }
        val sample = ImuSample(
            timestamp_ns = timestamp_ns,
            gyro_rad_s = if (gyroscope != null) gyro_rad_s else Vec3.ZERO,
            accel_m_s2 = if (accel_seen) accel_m_s2 else Vec3.ZERO,
            mag_u_t = if (mag_seen) mag_u_t else null,
            has_gyro = gyroscope != null,
            has_accel = accel_seen,
            rotation_vector_wxyz = rotation_vector_wxyz,
        )
        listener?.invoke(sample)
    }

    private fun FloatArray.toVec3(): Vec3 =
        if (size >= 3) Vec3(this[0].toDouble(), this[1].toDouble(), this[2].toDouble()) else Vec3.ZERO

    private companion object {
        const val LOG_TAG = "AndroidImuSource"
        const val GYRO_PERIOD_US = 5_000
        const val ACCEL_PERIOD_US = 10_000
        const val MAG_PERIOD_US = 20_000
        const val ROTATION_PERIOD_US = 20_000
    }
}
