package com.scand3d.sensors

/**
 * One motion sensor stream. The Android implementation owns `SensorManager`; tests substitute a
 * fake, because the platform classes only exist on a device (docs/DECISIONS.md D013).
 */
interface SensorSource {
    val availability: ImuAvailability

    fun start(listener: (ImuSample) -> Unit)

    fun stop()
}
