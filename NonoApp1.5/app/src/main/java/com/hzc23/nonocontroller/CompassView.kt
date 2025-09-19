package com.hzc23.nonocontroller

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

class CompassView(context: Context, attrs: AttributeSet?) : View(context, attrs) {

    private var heading = 0f
    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)

    fun setHeading(heading: Int) {
        this.heading = heading.toFloat()
        invalidate() // Redraw the view
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val width = width.toFloat()
        val height = height.toFloat()
        val centerX = width / 2
        val centerY = height / 2
        val radius = Math.min(centerX, centerY) - 20

        // Draw compass circle
        paint.color = Color.GRAY
        paint.style = Paint.Style.STROKE
        paint.strokeWidth = 5f
        canvas.drawCircle(centerX, centerY, radius, paint)

        // Draw heading indicator (arrow)
        paint.color = Color.RED
        paint.style = Paint.Style.FILL
        canvas.save()
        canvas.rotate(-heading, centerX, centerY)

        val path = android.graphics.Path()
        path.moveTo(centerX, centerY - radius)
        path.lineTo(centerX - 20, centerY - radius + 40)
        path.lineTo(centerX + 20, centerY - radius + 40)
        path.close()
        canvas.drawPath(path, paint)

        canvas.restore()
    }
}
