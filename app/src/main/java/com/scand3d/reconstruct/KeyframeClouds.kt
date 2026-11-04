package com.scand3d.reconstruct

/**
 * One colour per keyframe cloud (D059). Neighbouring keyframes get far-apart hues because a scan
 * walks through them in order, so adjacent clouds are the pair that must stay distinguishable.
 */
object KeyframeClouds {

    fun colours(ids: IntArray, point_count: Int): FloatArray {
        val colours = FloatArray(point_count * 3)
        for (index in 0 until point_count.coerceAtMost(ids.size)) {
            val hue = (((ids[index].coerceAtLeast(0) * 0.61803398875) % 1.0)).toFloat()
            val rgb = hsv(hue, 0.85f, 1.0f)
            colours[index * 3] = rgb[0]
            colours[index * 3 + 1] = rgb[1]
            colours[index * 3 + 2] = rgb[2]
        }
        return colours
    }

    private fun hsv(h: Float, s: Float, v: Float): FloatArray {
        val scaled = h * 6.0f
        val sector = scaled.toInt() % 6
        val fraction = scaled - scaled.toInt()
        val p = v * (1.0f - s)
        val q = v * (1.0f - s * fraction)
        val t = v * (1.0f - s * (1.0f - fraction))
        return when (sector) {
            0 -> floatArrayOf(v, t, p)
            1 -> floatArrayOf(q, v, p)
            2 -> floatArrayOf(p, v, t)
            3 -> floatArrayOf(p, q, v)
            4 -> floatArrayOf(t, p, v)
            else -> floatArrayOf(v, p, q)
        }
    }
}
