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
import kotlin.math.pow
import kotlin.math.sqrt

class JoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    interface JoystickListener {
        fun onJoystickMoved(angle: Int, distance: Int)
        fun onJoystickReleased()
    }

    private var listener: JoystickListener? = null

    private val outerCirclePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#E0E0E0") // Light gray
        style = Paint.Style.FILL
    }

    private val innerCirclePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#9E9E9E") // Darker gray
        style = Paint.Style.FILL
    }

    private var centerX = 0f
    private var centerY = 0f
    private var outerRadius = 0f
    private var innerRadius = 0f

    private var innerCircleX = 0f
    private var innerCircleY = 0f

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        centerX = w / 2f
        centerY = h / 2f
        val radius = min(w, h) / 2f
        outerRadius = radius * 0.9f
        innerRadius = radius * 0.45f
        resetInnerCirclePosition()
    }

    private fun resetInnerCirclePosition() {
        innerCircleX = centerX
        innerCircleY = centerY
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawCircle(centerX, centerY, outerRadius, outerCirclePaint)
        canvas.drawCircle(innerCircleX, innerCircleY, innerRadius, innerCirclePaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val touchX = event.x
        val touchY = event.y
        val deltaX = touchX - centerX
        val deltaY = touchY - centerY
        val distance = sqrt(deltaX.pow(2) + deltaY.pow(2))

        when (event.action) {
            MotionEvent.ACTION_DOWN,
            MotionEvent.ACTION_MOVE -> {
                if (distance < outerRadius) {
                    innerCircleX = touchX
                    innerCircleY = touchY
                } else {
                    // Keep the knob inside the outer circle
                    val ratio = outerRadius / distance
                    innerCircleX = centerX + deltaX * ratio
                    innerCircleY = centerY + deltaY * ratio
                }
                notifyListener()
            }
            MotionEvent.ACTION_UP -> {
                resetInnerCirclePosition()
                listener?.onJoystickReleased()
            }
        }
        invalidate()
        return true
    }

    private fun notifyListener() {
        if (listener == null) return

        val deltaX = innerCircleX - centerX
        val deltaY = innerCircleY - centerY
        
        // Angle in degrees, 0-359, 0 is right
        var angle = Math.toDegrees(atan2(deltaY.toDouble(), deltaX.toDouble())).toInt()
        if (angle < 0) angle += 360

        val distance = sqrt(deltaX.pow(2) + deltaY.pow(2))
        val distancePercent = ((distance / outerRadius) * 100).toInt().coerceAtMost(100)

        listener?.onJoystickMoved(angle, distancePercent)
    }

    fun setOnJoystickMovedListener(listener: JoystickListener) {
        this.listener = listener
    }
}
