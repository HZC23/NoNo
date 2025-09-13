package com.hzc23.nonocontroller

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

    open fun onSerialReceived(line: String) {
        val timestamp = dateFormat.format(Date())
        _debugMessages.update { messages ->
            (messages + (timestamp to line)).takeLast(100)
        }

        val capRegex = """Cap:\s*(\d+),\s*Cible:\s*(\d+)""".toRegex()
        val distRegex = """Distance US:\s*(\d+)\s*cm""".toRegex()
        val batRegex = """Batterie:\s*(\d+)%""".toRegex()

        capRegex.find(line)?.let {
            _currentCap.value = it.groupValues[1].toIntOrNull() ?: 0
            _targetCap.value = it.groupValues[2].toIntOrNull() ?: 0
        }

        distRegex.find(line)?.let {
            val distance = it.groupValues[1].toIntOrNull() ?: 0
            // Simple alternating assignment for preview
            if (_distanceLeft.value == 0) {
                _distanceLeft.value = distance
            } else {
                _distanceRight.value = distance
            }
        }

        batRegex.find(line)?.let {
            _batteryLevel.value = it.groupValues[1].toIntOrNull() ?: 0
        }
    }

    fun onConnectionStateChanged(state: String) {
        _connectionState.value = state
    }
}