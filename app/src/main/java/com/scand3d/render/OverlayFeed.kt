package com.scand3d.render

/**
 * Carries overlay vertices from whoever projects the scene (the capture view model) to the GL
 * renderer, which lives on the GL thread behind the Compose view. The last frame is kept so a
 * renderer that is rebuilt with the view still draws the current scene.
 */
object OverlayFeed {

    @Volatile
    private var sink: ((FloatArray) -> Unit)? = null

    private var lastVertices: FloatArray = FloatArray(0)

    @Synchronized
    fun bind(target: ((FloatArray) -> Unit)?) {
        sink = target
        if (target != null && lastVertices.isNotEmpty()) {
            target(lastVertices)
        }
    }

    /** Vertices are {x_ndc, y_ndc, r, g, b} per point; an empty array clears the overlay. */
    @Synchronized
    fun publish(vertices: FloatArray) {
        lastVertices = vertices
        sink?.invoke(vertices)
    }
}
