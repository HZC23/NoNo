package com.hzc23.nonocontroller

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.atan2
import kotlin.math.min
import kotlin.math.sqrt

class JoystickView(context: Context, attrs: AttributeSet?) : View(context, attrs) {

    private val paint = Paint()
    private var centerX = 0f
    private var centerY = 0f
    private var radius = 0f
    private var handleX = 0f
    private var handleY = 0f
    private var handleRadius = 0f

    private var listener: JoystickListener? = null

    interface JoystickListener {
        fun onJoystickMoved(angle: Int, distance: Int)
        fun onJoystickReleased()
    }

    fun setOnJoystickMovedListener(listener: JoystickListener) {
        this.listener = listener
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        centerX = w / 2f
        centerY = h / 2f
        radius = min(w, h) / 2f - 40
        handleRadius = radius / 2f
        handleX = centerX
        handleY = centerY
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        paint.color = Color.GRAY
        canvas.drawCircle(centerX, centerY, radius, paint)
        paint.color = Color.RED
        canvas.drawCircle(handleX, handleY, handleRadius, paint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val x = event.x
        val y = event.y
        val dx = x - centerX
        val dy = y - centerY
        val distance = sqrt(dx * dx + dy * dy)

        when (event.action) {
            MotionEvent.ACTION_DOWN,
            MotionEvent.ACTION_MOVE -> {
                if (distance <= radius) {
                    handleX = x
                    handleY = y
                } else {
                    handleX = centerX + dx / distance * radius
                    handleY = centerY + dy / distance * radius
                }
                invalidate()

                val angle = (atan2(dy, dx) * 180 / Math.PI).toInt()
                val normalizedDistance = (min(distance, radius) / radius * 100).toInt()
                listener?.onJoystickMoved(angle, normalizedDistance)
            }
            MotionEvent.ACTION_UP -> {
                handleX = centerX
                handleY = centerY
                invalidate()
                listener?.onJoystickReleased()
            }
        }
        return true
    }
}
