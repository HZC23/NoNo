package com.hzc23.nonocontroller

import com.google.gson.annotations.SerializedName

data class RobotTelemetry(
    val distance: Int,
    val cap: Int,
    val battery: Int,
    val pir: Boolean,
    val lcd: String
)