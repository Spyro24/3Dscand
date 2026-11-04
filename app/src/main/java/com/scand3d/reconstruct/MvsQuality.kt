package com.scand3d.reconstruct

import androidx.annotation.StringRes
import com.scand3d.R

/**
 * Working width of the dense stage (roadmap 6.4). The captured frames keep their size; only the
 * depth estimation runs at this width. Normal is the default because it halves the Chamfer error of
 * Draft for four times the runtime, while High adds points without improving the distance
 * (`docs/benchmarks/phase6-mvs-verification.md`).
 */
enum class MvsQuality(val working_width_px: Int, @StringRes val label_res: Int) {
    Draft(160, R.string.dense_quality_draft),
    Normal(320, R.string.dense_quality_normal),
    High(480, R.string.dense_quality_high);

    fun next(): MvsQuality = values()[(ordinal + 1) % values().size]
}
