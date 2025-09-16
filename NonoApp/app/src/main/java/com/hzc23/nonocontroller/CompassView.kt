package com.hzc23.nonocontroller

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import kotlin.math.cos
import kotlin.math.sin

class CompassView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var heading: Float = 0f

    private val circlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 5f
        color = Color.DKGRAY
    }

    private val needlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 8f
        color = Color.RED
    }
    
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 30f
        color = Color.BLACK
        textAlign = Paint.Align.CENTER
    }

    fun setHeading(degrees: Int) {
        this.heading = degrees.toFloat()
        invalidate() // Request a redraw
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val centerX = width / 2f
        val centerY = height / 2f
        val radius = (width.coerceAtMost(height) / 2f) * 0.85f

        // Draw compass circle
        canvas.drawCircle(centerX, centerY, radius, circlePaint)

        // Draw North indicator text
        canvas.drawText("N", centerX, centerY - radius - 15f, textPaint)

        // Calculate needle end point
        // Angle is adjusted by -90 degrees because 0 is North (up) in compass, but 0 degrees is East (right) in canvas coordinates.
        val angleInRadians = Math.toRadians(heading.toDouble() - 90)
        val needleEndX = centerX + radius * cos(angleInRadians).toFloat()
        val needleEndY = centerY + radius * sin(angleInRadians).toFloat()

        // Draw needle
        canvas.drawLine(centerX, centerY, needleEndX, needleEndY, needlePaint)
    }
}
