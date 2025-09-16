package com.hzc23.nonocontroller.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import com.hzc23.nonocontroller.MainViewModel
import com.hzc23.nonocontroller.RobotTelemetry
import com.hzc23.nonocontroller.SettingsDataStore
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

import com.hzc23.nonocontroller.BlunoLibrary

class FakeMainViewModel(application: Application) : MainViewModel(application, FakeSettingsDataStore(application), BlunoLibrary(application.applicationContext)) {

    private val _fakeBatteryLevel = MutableStateFlow(75)
    val fakeBatteryLevel: StateFlow<Int> = _fakeBatteryLevel.asStateFlow()

    private val _fakeCurrentCap = MutableStateFlow(180)
    val fakeCurrentCap: StateFlow<Int> = _fakeCurrentCap.asStateFlow()

    private val _fakeDistanceLeft = MutableStateFlow(50)
    val fakeDistanceLeft: StateFlow<Int> = _fakeDistanceLeft.asStateFlow()

    private val _fakeDistanceRight = MutableStateFlow(60)
    val fakeDistanceRight: StateFlow<Int> = _fakeDistanceRight.asStateFlow()

    init {
        updateTelemetry(RobotTelemetry(
            distance = 0,
            cap = _fakeCurrentCap.value,
            battery = _fakeBatteryLevel.value,
            pir = false,
            lcd = "Hello Nono!"
        ))
    }

    fun setFakeBatteryLevel(level: Int) {
        _fakeBatteryLevel.value = level
        updateTelemetry(robotTelemetry.value!!.copy(battery = level))
    }

    fun setFakeCurrentCap(cap: Int) {
        _fakeCurrentCap.value = cap
        updateTelemetry(robotTelemetry.value!!.copy(cap = cap))
    }

    fun setFakeDistanceLeft(distance: Int) {
        _fakeDistanceLeft.value = distance
        updateTelemetry(robotTelemetry.value!!.copy(distance = distance))
    }

    fun setFakeDistanceRight(distance: Int) {
        _fakeDistanceRight.value = distance
        updateTelemetry(robotTelemetry.value!!.copy(distance = distance))
    }
}
