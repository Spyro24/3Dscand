package com.scand3d.render

import android.graphics.SurfaceTexture
import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.util.Log
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * Draws the camera preview as luminance only (roadmap 1.4, agents.md section 4): the fragment
 * shader collapses the sensor image to a single channel before it reaches the screen.
 */
class MonochromePreviewRenderer(
    private val textureWidthPx: Int,
    private val textureHeightPx: Int,
    private val onSurfaceTextureReady: (SurfaceTexture) -> Unit,
) : GLSurfaceView.Renderer, SurfaceTexture.OnFrameAvailableListener {

    private var program = 0
    private var textureId = 0
    private var surfaceTexture: SurfaceTexture? = null
    private var positionHandle = 0
    private var texCoordHandle = 0
    private var matrixHandle = 0
    private var textureHandle = 0
    private val textureMatrix = FloatArray(16)
    private val vertexBuffer = GlBuffers.floatBuffer(
        floatArrayOf(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f),
    )
    private val texCoordBuffer = GlBuffers.floatBuffer(
        floatArrayOf(0f, 0f, 1f, 0f, 0f, 1f, 1f, 1f),
    )
    private var frameAvailable = false
    private var viewportWidthPx = 1
    private var viewportHeightPx = 1
    private var requestRender: (() -> Unit)? = null

    private var pointsProgram = 0
    private var pointPositionHandle = 0
    private var pointColorHandle = 0
    private var pointOpacityHandle = 0
    private var pointSizeHandle = 0
    private var overlayBuffer: FloatBuffer? = null
    private var overlayVertexCount = 0
    private var pendingOverlay: FloatArray? = null
    private var overlayEnabled = true
    private var overlayOpacity = 1.0f

    /** The GLSurfaceView installs its redraw request here so frames are drawn when they arrive. */
    fun bindRequestRender(block: () -> Unit) {
        requestRender = block
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        program = GlShaders.createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        positionHandle = GLES20.glGetAttribLocation(program, "aPosition")
        texCoordHandle = GLES20.glGetAttribLocation(program, "aTexCoord")
        matrixHandle = GLES20.glGetUniformLocation(program, "uTexMatrix")
        textureHandle = GLES20.glGetUniformLocation(program, "uTexture")

        val textures = IntArray(1)
        GLES20.glGenTextures(1, textures, 0)
        textureId = textures[0]
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)

        val texture = SurfaceTexture(textureId)
        texture.setDefaultBufferSize(textureWidthPx, textureHeightPx)
        texture.setOnFrameAvailableListener(this)
        surfaceTexture = texture
        onSurfaceTextureReady(texture)
        Log.i(LOG_TAG, "preview texture ready ${textureWidthPx}x$textureHeightPx")

        // Sparse colour overlay (roadmap 5.1): a separate program drawn into the same letterboxed
        // rectangle as the camera image, so projected NDC lines up with the image one to one.
        pointsProgram = GlShaders.createProgram(POINT_VERTEX_SHADER, POINT_FRAGMENT_SHADER)
        pointPositionHandle = GLES20.glGetAttribLocation(pointsProgram, "aPosition")
        pointColorHandle = GLES20.glGetAttribLocation(pointsProgram, "aColor")
        pointOpacityHandle = GLES20.glGetUniformLocation(pointsProgram, "uOpacity")
        pointSizeHandle = GLES20.glGetUniformLocation(pointsProgram, "uPointSize")
        // Replays the last published scene and style into a renderer that was just rebuilt.
        OverlayFeed.bind(::submitOverlay, ::applyOverlayStyle)
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewportWidthPx = width.coerceAtLeast(1)
        viewportHeightPx = height.coerceAtLeast(1)
        GLES20.glViewport(0, 0, viewportWidthPx, viewportHeightPx)
    }

    override fun onDrawFrame(gl: GL10?) {
        val texture = surfaceTexture ?: return
        synchronized(this) {
            if (frameAvailable) {
                texture.updateTexImage()
                texture.getTransformMatrix(textureMatrix)
                frameAvailable = false
            }
        }

        GLES20.glClearColor(0f, 0f, 0f, 1f)
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)

        // Letterbox the camera image so its aspect ratio survives the surface's shape.
        val viewAspect = viewportWidthPx.toFloat() / viewportHeightPx.toFloat()
        val textureAspect = textureWidthPx.toFloat() / textureHeightPx.toFloat()
        val drawWidth: Int
        val drawHeight: Int
        if (viewAspect > textureAspect) {
            drawHeight = viewportHeightPx
            drawWidth = (viewportHeightPx * textureAspect).toInt()
        } else {
            drawWidth = viewportWidthPx
            drawHeight = (viewportWidthPx / textureAspect).toInt()
        }
        GLES20.glViewport((viewportWidthPx - drawWidth) / 2, (viewportHeightPx - drawHeight) / 2, drawWidth, drawHeight)

        GLES20.glUseProgram(program)
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES20.glUniform1i(textureHandle, 0)
        GLES20.glUniformMatrix4fv(matrixHandle, 1, false, textureMatrix, 0)

        GLES20.glEnableVertexAttribArray(positionHandle)
        GLES20.glVertexAttribPointer(positionHandle, 2, GLES20.GL_FLOAT, false, 0, vertexBuffer)
        GLES20.glEnableVertexAttribArray(texCoordHandle)
        GLES20.glVertexAttribPointer(texCoordHandle, 2, GLES20.GL_FLOAT, false, 0, texCoordBuffer)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
        GLES20.glDisableVertexAttribArray(positionHandle)
        GLES20.glDisableVertexAttribArray(texCoordHandle)

        drawOverlay()
    }

    /**
     * Takes a projected scene from any thread: {x_ndc, y_ndc, r, g, b} per point, far to near.
     * An empty array clears the overlay.
     */
    fun submitOverlay(vertices: FloatArray) {
        synchronized(this) { pendingOverlay = vertices }
        requestRender?.invoke()
    }

    /** Visibility and opacity reach the GL thread separately, so toggling never re-projects. */
    private fun applyOverlayStyle(enabled: Boolean, opacity: Float) {
        synchronized(this) {
            overlayEnabled = enabled
            overlayOpacity = opacity
        }
        requestRender?.invoke()
    }

    private fun drawOverlay() {
        val incoming = synchronized(this) {
            val pending = pendingOverlay
            pendingOverlay = null
            pending
        }
        if (incoming != null) {
            val buffer = overlayBuffer?.takeIf { it.capacity() >= incoming.size }
                ?: GlBuffers.sizedFloatBuffer(incoming.size).also { overlayBuffer = it }
            buffer.clear()
            buffer.put(incoming)
            buffer.position(0)
            overlayVertexCount = incoming.size / OVERLAY_VERTEX_FLOATS
        }
        val buffer = overlayBuffer
        if (!overlayEnabled || overlayOpacity <= 0.0f || overlayVertexCount == 0 ||
            pointsProgram == 0 || buffer == null
        ) {
            return
        }

        GLES20.glUseProgram(pointsProgram)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)
        GLES20.glUniform1f(pointOpacityHandle, overlayOpacity)
        GLES20.glUniform1f(pointSizeHandle, OVERLAY_POINT_SIZE_PX)

        GLES20.glEnableVertexAttribArray(pointPositionHandle)
        GLES20.glVertexAttribPointer(
            pointPositionHandle,
            2,
            GLES20.GL_FLOAT,
            false,
            OVERLAY_STRIDE_BYTES,
            buffer,
        )
        buffer.position(2)
        GLES20.glEnableVertexAttribArray(pointColorHandle)
        GLES20.glVertexAttribPointer(
            pointColorHandle,
            3,
            GLES20.GL_FLOAT,
            false,
            OVERLAY_STRIDE_BYTES,
            buffer,
        )
        buffer.position(0)
        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, overlayVertexCount)
        GLES20.glDisableVertexAttribArray(pointPositionHandle)
        GLES20.glDisableVertexAttribArray(pointColorHandle)
        GLES20.glDisable(GLES20.GL_BLEND)
    }

    override fun onFrameAvailable(surfaceTexture: SurfaceTexture?) {
        synchronized(this) { frameAvailable = true }
        requestRender?.invoke()
    }

    private companion object {
        const val LOG_TAG = "MonochromePreview"
        const val OVERLAY_VERTEX_FLOATS = 5
        const val OVERLAY_STRIDE_BYTES = OVERLAY_VERTEX_FLOATS * 4
        const val OVERLAY_POINT_SIZE_PX = 6.0f

        // Luminance from the sensor's RGB, then a single grey channel: no colour reaches the screen.
        const val FRAGMENT_SHADER = """
            #extension GL_OES_EGL_image_external : require
            precision mediump float;
            varying vec2 vTexCoord;
            uniform samplerExternalOES uTexture;
            void main() {
                vec4 colour = texture2D(uTexture, vTexCoord);
                float luminance = dot(colour.rgb, vec3(0.299, 0.587, 0.114));
                gl_FragColor = vec4(luminance, luminance, luminance, 1.0);
            }
        """

        const val VERTEX_SHADER = """
            attribute vec4 aPosition;
            attribute vec4 aTexCoord;
            uniform mat4 uTexMatrix;
            varying vec2 vTexCoord;
            void main() {
                gl_Position = aPosition;
                vTexCoord = (uTexMatrix * aTexCoord).xy;
            }
        """

        // Point sprites: the vertex stage receives NDC computed by the native overlay projection,
        // so this shader only sizes and colours a point.
        const val POINT_VERTEX_SHADER = """
            attribute vec2 aPosition;
            attribute vec3 aColor;
            uniform float uPointSize;
            varying vec3 vColor;
            void main() {
                gl_Position = vec4(aPosition, 0.0, 1.0);
                gl_PointSize = uPointSize;
                vColor = aColor;
            }
        """

        const val POINT_FRAGMENT_SHADER = """
            precision mediump float;
            uniform float uOpacity;
            varying vec3 vColor;
            void main() {
                gl_FragColor = vec4(vColor, uOpacity);
            }
        """
    }
}
