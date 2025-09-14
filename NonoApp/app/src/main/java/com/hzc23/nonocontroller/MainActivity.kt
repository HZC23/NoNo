package com.hzc23.nonocontroller

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.core.content.ContextCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.dfrobot.angelo.blunobasicdemo.BlunoLibrary
import com.hzc23.nonocontroller.ui.NonoControllerScreen
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import com.hzc23.nonocontroller.MainViewModelFactory

import android.util.Log
import com.google.gson.Gson
import com.hzc23.nonocontroller.RobotTelemetry

class MainActivity : BlunoLibrary() {

    private val gson = Gson()

    enum class RobotMode(val command: String, val displayName: String) {
        MANUAL("manual", "Manuel"),
        AUTO("auto", "Auto"),
        OBSTACLE("obstacle", "Obstacle"),
        PIR("detect", "PIR"),
        CAP("cap", "Cap");
    }

    private val settingsDataStore by lazy { SettingsDataStore(applicationContext) }
    private val viewModel: MainViewModel by viewModels {
        MainViewModelFactory(settingsDataStore)
    }

    private val requestPermissionsLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.values.all { it }) {
            buttonScanOnClickProcess()
        } else {
            // Handle the case where the user denies the permissions
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        makeActivityImmersive()

        onCreateProcess()
        serialBegin(115200)

        setContent {
            NonoControllerScreen(
                viewModel = viewModel,
                onSendCommand = { command -> serialSend(command) },
                onSetMode = { mode -> serialSend(mode.command) },
                onConnect = { checkPermissions() }
            )
        }
    }

    private fun checkPermissions() {
        val permissionsToRequest = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissionsToRequest.add(Manifest.permission.BLUETOOTH_SCAN)
            permissionsToRequest.add(Manifest.permission.BLUETOOTH_CONNECT)
        }
        permissionsToRequest.add(Manifest.permission.ACCESS_FINE_LOCATION)

        val permissionsNotGranted = permissionsToRequest.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (permissionsNotGranted.isNotEmpty()) {
            requestPermissionsLauncher.launch(permissionsNotGranted.toTypedArray())
        } else {
            buttonScanOnClickProcess()
        }
    }

    private fun makeActivityImmersive() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        val controller = WindowInsetsControllerCompat(window, window.decorView)
        controller.hide(WindowInsetsCompat.Type.systemBars())
        controller.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
    }

    override fun onConectionStateChange(theConnectionState: connectionStateEnum?) {
        theConnectionState?.let {
            viewModel.onConnectionStateChanged(it.name)
        }
    }

    override fun onSerialReceived(theString: String?) {
        theString?.let { jsonString ->
            GlobalScope.launch(Dispatchers.IO) {
                try {
                    val telemetry = gson.fromJson(jsonString.trim(), RobotTelemetry::class.java)
                    viewModel.updateTelemetry(telemetry)
                } catch (e: Exception) {
                    Log.e("RobotTelemetry", "Erreur de parsing JSON: $jsonString", e)
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        onResumeProcess()
    }

    override fun onPause() {
        super.onPause()
        onPauseProcess()
    }

    override fun onStop() {
        super.onStop()
        onStopProcess()
    }

    override fun onDestroy() {
        super.onDestroy()
        onDestroyProcess()
    }
}