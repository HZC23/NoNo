package com.hzc23.nonocontroller

data class RobotTelemetry(
    val state: String,
    val heading: Int,
    val distanceHaut: Int,
    val distanceBas: Int,
    val battery: Int,
    val speedTarget: Int,
    val speedCurrent: Int
)