package com.hzc23.nonocontroller

import android.app.Application
import android.bluetooth.BluetoothDevice
import android.content.Intent
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import java.text.SimpleDateFormat
import java.util.*

enum class RobotMode(val command: String) {
    MANUAL("CMD:MODE:MANUAL"),
    AUTO("CMD:MODE:AUTO"),
    OBSTACLE("CMD:MODE:OBSTACLE"),
    PIR("CMD:MODE:PIR"),
    CAP("CMD:MODE:CAP")
}

open class MainViewModel(
    application: Application,
    private val settingsDataStore: SettingsDataStore
) : AndroidViewModel(application), BlunoLibrary.BlunoListener {

    private val _connectionState = MutableStateFlow(BlunoLibrary.connectionStateEnum.isNull)
    val connectionState: StateFlow<BlunoLibrary.connectionStateEnum> = _connectionState

    private val _serialData = MutableStateFlow("")
    val serialData: StateFlow<String> = _serialData

    private val _robotTelemetry = MutableStateFlow<RobotTelemetry?>(null)
    val robotTelemetry: StateFlow<RobotTelemetry?> = _robotTelemetry

    private val _debugMessages = MutableStateFlow<List<Pair<String, String>>>(emptyList())
    val debugMessages: StateFlow<List<Pair<String, String>>> = _debugMessages

    private val _scannedDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val scannedDevices: StateFlow<List<BluetoothDevice>> = _scannedDevices

    val isLayoutInverted: StateFlow<Boolean> = settingsDataStore.isLayoutInverted
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    val isDarkModeEnabled: StateFlow<Boolean> = settingsDataStore.isDarkModeEnabled
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    private val dateFormat = SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault())

    private val blunoLibrary: BlunoLibrary = BlunoLibrary(application.applicationContext)

    init {
        blunoLibrary.setBlunoListener(this)
        blunoLibrary.initialize()
    }

    fun startScan() {
        android.util.Log.d("MainViewModel", "startScan called")
        blunoLibrary.scanLeDevice(true)
    }

    fun stopScan() {
        blunoLibrary.scanLeDevice(false)
    }

    fun connect(device: BluetoothDevice) {
        blunoLibrary.connect(device.address)
    }

    fun disconnect() {
        blunoLibrary.disconnect()
    }

    fun sendCommand(command: String) {
        blunoLibrary.serialSend(command)
    }

    fun onResume() {
        blunoLibrary.onResume()
    }

    fun onPause() {
        blunoLibrary.onPause()
    }

    fun onDestroy() {
        blunoLibrary.onDestroy()
    }

    fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        data?.let {
            blunoLibrary.onActivityResult(requestCode, resultCode, it)
        }
    }

    override fun onConnectionStateChange(theConnectionState: BlunoLibrary.connectionStateEnum) {
        _connectionState.value = theConnectionState
    }

    private var lastJoystickDirection: String? = null

    fun sendJoystickCommand(direction: String) {
        if (direction != lastJoystickDirection) {
            sendCommand("CMD:MOVE:$direction")
            lastJoystickDirection = direction
        }
    }

    override fun onSerialReceived(theString: String) {
        _serialData.value = theString
        try {
            val json = org.json.JSONObject(theString)
            val telemetry = RobotTelemetry(
                state = json.optString("state", "—"),
                heading = json.optInt("heading", 0),
                distanceHaut = json.optInt("distance_haut", 0),
                distanceBas = json.optInt("distance_bas", 0),
                battery = json.optInt("battery", 0),
                speedTarget = json.optInt("speedTarget", 0),
                speedCurrent = json.optInt("speedCurrent", 0)
            )
            updateTelemetry(telemetry)
        } catch (e: org.json.JSONException) {
            android.util.Log.e("MainViewModel", "JSON parse error: $e")
        }
    }

    override fun onDeviceScanned(devices: List<BluetoothDevice>) {
        _scannedDevices.value = devices
    }

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
    }
}
