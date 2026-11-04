package com.scand3d.capture

/**
 * Fixed-size buffer of the most recent frames. The camera callback thread writes, and the
 * view model reads recent history without copying unbounded data.
 */
class FrameRingBuffer(private val capacity: Int = 120) {

    private val frames = ArrayDeque<FrameRecord>(capacity)

    @Synchronized
    fun add(frame: FrameRecord) {
        if (frames.size == capacity) {
            frames.removeFirst()
        }
        frames.addLast(frame)
    }

    @Synchronized
    fun snapshot(): List<FrameRecord> = frames.toList()

    @Synchronized
    fun latest(): FrameRecord? = frames.lastOrNull()

    @Synchronized
    fun size(): Int = frames.size
}
