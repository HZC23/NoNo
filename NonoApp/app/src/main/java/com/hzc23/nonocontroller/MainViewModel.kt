package com.hzc23.nonocontroller

import android.app.Application
import android.bluetooth.BluetoothDevice
import android.content.Intent
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.delay
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

enum class ScanState {
    STOPPED,
    SCANNING,
    DONE
}

open class MainViewModel(
    application: Application,
    private val settingsDataStore: SettingsDataStore,
    private val blunoLibrary: BlunoLibrary
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

    private val _scanState = MutableStateFlow(ScanState.STOPPED)
    val scanState: StateFlow<ScanState> = _scanState

    val isLayoutInverted: StateFlow<Boolean> = settingsDataStore.isLayoutInverted
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    val isDarkModeEnabled: StateFlow<Boolean> = settingsDataStore.isDarkModeEnabled
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    private val dateFormat = SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault())

    init {
        blunoLibrary.setBlunoListener(this)
    }

    fun startScan() {
        if (_scanState.value == ScanState.SCANNING) return

        _scannedDevices.value = emptyList() // Clear previous results
        blunoLibrary.scanLeDevice(true)
        _scanState.value = ScanState.SCANNING
        viewModelScope.launch {
            delay(5000) // Scan for 5 seconds
            if (_scanState.value == ScanState.SCANNING) {
                stopScan()
            }
        }
    }

    fun stopScan() {
        if (_scanState.value != ScanState.SCANNING) return
        blunoLibrary.scanLeDevice(false)
        _scanState.value = ScanState.DONE
    }

    fun resetScanState() {
        _scanState.value = ScanState.STOPPED
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

    override fun onSerialReceived(theString: String) {
        _serialData.value = theString
        // TODO: Parse theString to a RobotTelemetry object and call updateTelemetry
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
