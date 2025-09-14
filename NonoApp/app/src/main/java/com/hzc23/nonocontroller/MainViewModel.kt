package com.hzc23.nonocontroller

import com.hzc23.nonocontroller.RobotTelemetry
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import java.text.SimpleDateFormat
import java.util.*

open class MainViewModel(private val settingsDataStore: SettingsDataStore) : ViewModel() {

    private val _batteryLevel = MutableStateFlow(0)
    val batteryLevel: StateFlow<Int> = _batteryLevel

    private val _currentCap = MutableStateFlow(0)
    val currentCap: StateFlow<Int> = _currentCap

    private val _targetCap = MutableStateFlow(0)
    val targetCap: StateFlow<Int> = _targetCap

    private val _distanceLeft = MutableStateFlow(0)
    val distanceLeft: StateFlow<Int> = _distanceLeft

    private val _distanceRight = MutableStateFlow(0)
    val distanceRight: StateFlow<Int> = _distanceRight

    private val _debugMessages = MutableStateFlow<List<Pair<String, String>>>(emptyList())
    val debugMessages: StateFlow<List<Pair<String, String>>> = _debugMessages

    private val _connectionState = MutableStateFlow("Disconnected")
    val connectionState: StateFlow<String> = _connectionState

    private val _robotTelemetry = MutableStateFlow<RobotTelemetry?>(null)
    val robotTelemetry: StateFlow<RobotTelemetry?> = _robotTelemetry

    val isLayoutInverted: StateFlow<Boolean> = settingsDataStore.isLayoutInverted
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    val isDarkModeEnabled: StateFlow<Boolean> = settingsDataStore.isDarkModeEnabled
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    private val dateFormat = SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault())

    fun toggleLayoutInversion() {
        viewModelScope.launch {
            settingsDataStore.toggleLayoutInversion()
        }
    }

    fun toggleDarkMode() {
        viewModelScope.launch {
            settingsDataStore.toggleDarkMode()
        }
    }

    fun updateTelemetry(telemetry: RobotTelemetry) {
        _robotTelemetry.value = telemetry

        val timestamp = dateFormat.format(Date())
        _debugMessages.update { messages ->
            (messages + (timestamp to telemetry.toString())).takeLast(100)
        }

        telemetry.battery?.let { _batteryLevel.value = it }
        telemetry.heading?.let { _currentCap.value = it }
        // Assuming targetCap is also part of telemetry, if not, it needs to be handled differently
        // telemetry.targetCap?.let { _targetCap.value = it }
        telemetry.distance?.let { distance ->
            // Simple alternating assignment for preview, or a more sophisticated logic
            if (_distanceLeft.value == 0) {
                _distanceLeft.value = distance
            } else {
                _distanceRight.value = distance
            }
        }
    }

    fun onConnectionStateChanged(state: String) {
        _connectionState.value = state
    }
}