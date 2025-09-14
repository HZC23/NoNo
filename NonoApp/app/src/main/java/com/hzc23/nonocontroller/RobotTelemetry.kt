package com.hzc23.nonocontroller

import com.google.gson.annotations.SerializedName

data class RobotTelemetry(
        val state: String?,


    val heading: Int?,


    val distance: Int?,


    val battery: Int?,


    val speedTarget: Int?,


    val speedCurrent: Int?
)