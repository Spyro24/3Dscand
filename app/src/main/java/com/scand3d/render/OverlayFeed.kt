package com.scand3d.render

/**
 * Carries overlay vertices from whoever projects the scene (the capture view model) to the GL
 * renderer, which lives on the GL thread behind the Compose view. The last frame is kept so a
 * renderer that is rebuilt with the view still draws the current scene.
 */
object OverlayFeed {

    @Volatile
    private var sink: ((FloatArray) -> Unit)? = null

    @Volatile
    private var styleSink: ((Boolean, Float) -> Unit)? = null

    private var lastVertices: FloatArray = FloatArray(0)
    private var lastEnabled: Boolean = true
    private var lastOpacity: Float = 1.0f

    /** The renderer registers here when its surface is created and unregisters when it goes. */
    @Synchronized
    fun bind(
        onVertices: ((FloatArray) -> Unit)?,
        onStyle: ((Boolean, Float) -> Unit)? = null,
    ) {
        sink = onVertices
        styleSink = onStyle
        if (onVertices != null && lastVertices.isNotEmpty()) {
            onVertices(lastVertices)
        }
        onStyle?.invoke(lastEnabled, lastOpacity)
    }

    /** Vertices are {x_ndc, y_ndc, r, g, b} per point; an empty array clears the overlay. */
    @Synchronized
    fun publish(vertices: FloatArray) {
        lastVertices = vertices
        sink?.invoke(vertices)
    }

    /** Overlay visibility and opacity, applied by the renderer without re-projecting (5.4). */
    @Synchronized
    fun setStyle(enabled: Boolean, opacity: Float) {
        lastEnabled = enabled
        lastOpacity = opacity
        styleSink?.invoke(enabled, opacity)
    }
}
