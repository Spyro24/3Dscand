package com.scand3d.sensors

/**
 * Ties a sensor source to a filter and keeps the recent orientations that frames are annotated
 * from. On a device this is the whole Phase 2 sensor path from `SensorEvent` to a per-frame
 * quaternion.
 */
class FusedImu(
    private val source: SensorSource,
    private val filter: OrientationFilter = ImuFusion(),
    private val ring: ImuRingBuffer = ImuRingBuffer(),
    private val onSample: (ImuSample) -> Unit = {},
) {
    val availability: ImuAvailability
        get() = source.availability

    fun start() {
        source.start(::ingest)
    }

    fun stop() {
        source.stop()
    }

    fun snapshotAt(timestamp_ns: Long): ImuOrientationSample? = ring.nearestTo(timestamp_ns)

    fun latest(): ImuOrientationSample? = ring.latest()

    fun reset() {
        filter.reset()
        ring.clear()
    }

    fun close() {
        source.stop()
        filter.close()
    }

    private fun ingest(sample: ImuSample) {
        onSample(sample)
        val quaternion = filter.push(sample) ?: return
        ring.add(ImuOrientationSample(sample.timestamp_ns, quaternion))
    }
}
