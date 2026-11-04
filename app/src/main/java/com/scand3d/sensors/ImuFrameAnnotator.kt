package com.scand3d.sensors

import com.scand3d.capture.FrameRecord
import com.scand3d.capture.SessionMetadata
import com.scand3d.capture.SessionRecorder
import com.scand3d.core.bridge.OrientationMath

/** Rotation window a new keyframe must fall into (roadmap 2.6, agents.md section 7). */
data class KeyframePriorPolicy(
    val min_baseline_rad: Double = 0.10,
    val max_baseline_rad: Double = 0.70,
)

/**
 * Attaches the IMU snapshot to each saved frame, measures the camera/sensor clock residual and
 * marks IMU keyframe candidates. The controller calls this from the single frame-writer thread.
 *
 * The first frame anchors the scan. A frame rotated past [KeyframePriorPolicy.max_baseline_rad]
 * is not a keyframe but does re-anchor, otherwise one fast turn would leave every later frame
 * outside the window and no keyframes would come out of the rest of the scan.
 */
class ImuFrameAnnotator(
    private val snapshots: (Long) -> ImuOrientationSample?,
    private val math: OrientationMath,
    private val availability: ImuAvailability? = null,
    private val sync: TimestampSync = TimestampSync(),
    private val policy: KeyframePriorPolicy = KeyframePriorPolicy(),
    private val applied_offset_ns: Long = 0L,
) : SessionRecorder {

    private var reference_wxyz: List<Double>? = null

    @Volatile
    private var keyframe_candidates = 0

    @Volatile
    private var last_offset_ns: Long? = null

    override fun annotate(frame: FrameRecord): FrameRecord {
        // The stored offset centres the lookup on where the sensor clock really is.
        val sample = snapshots(frame.timestamp_ns + applied_offset_ns)
            ?: return frame // no IMU yet: leave the frame as the camera recorded it

        val quaternion = sample.quaternion_wxyz
        last_offset_ns = sync.observe(frame.timestamp_ns, sample.timestamp_ns)

        val reference = reference_wxyz
        var candidate = false
        var reanchor = false

        if (reference == null) {
            candidate = true
            reanchor = true
        } else {
            val angle_rad = math.angularDisplacementRad(reference, quaternion)
            when {
                !angle_rad.isFinite() -> Unit
                angle_rad < policy.min_baseline_rad -> Unit
                angle_rad <= policy.max_baseline_rad -> {
                    candidate = true
                    reanchor = true
                }
                else -> reanchor = true
            }
        }

        if (reanchor) {
            reference_wxyz = quaternion
        }
        if (candidate) {
            keyframe_candidates += 1
        }

        return frame.copy(
            orientation_wxyz = quaternion,
            imu_timestamp_ns = sample.timestamp_ns,
            imu_time_offset_ns = last_offset_ns,
            keyframe_candidate = candidate,
        )
    }

    override fun metadata(camera_clock_source: String): SessionMetadata = SessionMetadata(
        camera_clock_source = camera_clock_source,
        sensor_clock = SENSOR_CLOCK,
        clock_offset_ns = sync.medianResidualNs(),
        clock_offset_mad_ns = sync.medianAbsoluteDeviationNs(),
        clock_samples = sync.samples(),
        imu_mode = availability?.mode?.name,
        imu_reduced_accuracy = availability?.reduced_accuracy ?: false,
        keyframe_candidates = keyframe_candidates,
    )

    fun keyframeCandidates(): Int = keyframe_candidates

    fun lastOffsetNs(): Long? = last_offset_ns

    private companion object {
        /** Both streams are read in the boot-relative nanosecond clock (docs/architecture.md). */
        const val SENSOR_CLOCK = "elapsed_realtime_ns"
    }
}
