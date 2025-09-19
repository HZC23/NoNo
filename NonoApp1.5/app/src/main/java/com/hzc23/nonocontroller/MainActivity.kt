package com.hzc23.nonocontroller

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowDownward
import androidx.compose.material.icons.filled.ArrowUpward
import androidx.compose.material.icons.filled.Search
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.core.view.WindowCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import com.hzc23.nonocontroller.ui.theme.*

class MainActivity : ComponentActivity() {

    private val viewModel: MainViewModel by viewModels {
        MainViewModelFactory(application, SettingsDataStore(this))
    }

    private val requestMultiplePermissions =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            permissions.entries.forEach {
                // Handle permission result
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        WindowCompat.setDecorFitsSystemWindows(window, false)
        checkAndRequestPermissions()

        setContent {
            NonoControllerTheme {
                AppScreen()
            }
        }
    }

    private fun checkAndRequestPermissions() {
        val requiredPermissions =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                arrayOf(
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT,
                    Manifest.permission.ACCESS_FINE_LOCATION
                )
            } else {
                arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
            }

        val permissionsToRequest = requiredPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (permissionsToRequest.isNotEmpty()) {
            requestMultiplePermissions.launch(permissionsToRequest.toTypedArray())
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun NonoControllerApp(viewModel: MainViewModel = viewModel()) {
    val connectionState by viewModel.connectionState.collectAsState()
    val telemetry by viewModel.robotTelemetry.collectAsState()
    val scannedDevices by viewModel.scannedDevices.collectAsState()

    var showScanDialog by remember { mutableStateOf(false) }

    if (showScanDialog) {
        ScanDialog(
            devices = scannedDevices,
            onDismiss = { showScanDialog = false },
            onDeviceSelected = { device ->
                viewModel.connect(device)
                showScanDialog = false
            }
        )
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Nono Controller", fontWeight = FontWeight.Bold, fontSize = 24.sp) },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = Color.Transparent,
                    titleContentColor = TextPrimary
                ),
                actions = {
                    IconButton(onClick = { viewModel.startScan(); showScanDialog = true }) {
                        Icon(Icons.Default.Search, contentDescription = "Scan for devices", tint = AccentBlue)
                    }
                    ConnectionStatus(connectionState, telemetry?.battery ?: 0)
                }
            )
        },
        bottomBar = {
            StopButton(onClick = { viewModel.sendCommand("CMD:MOVE:STOP") })
        }
    ) { paddingValues ->
        Box(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
                .background(BackgroundDark)
                .padding(16.dp),
            contentAlignment = Alignment.Center
        ) {
            MainCard(viewModel, telemetry)
        }
    }
}

@Composable
fun MainCard(viewModel: MainViewModel, telemetry: RobotTelemetry?) {
    Card(
        modifier = Modifier.fillMaxSize(),
        shape = MaterialTheme.shapes.large,
        colors = CardDefaults.cardColors(containerColor = MainCardDark)
    ) {
        Row(
            modifier = Modifier
                .fillMaxSize()
                .padding(24.dp),
            horizontalArrangement = Arrangement.spacedBy(40.dp)
        ) {
            // Left Column
            Column(
                modifier = Modifier.weight(1f),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.SpaceBetween
            ) {
                Joystick(onMove = { angle, distance ->
                    val direction = when {
                        distance < 30 -> "STOP"
                        angle in 45..135 -> "FWD"
                        angle in 135..225 -> "LEFT"
                        angle in 225..315 -> "BWD"
                        else -> "RIGHT"
                    }
                    viewModel.sendJoystickCommand(direction)
                })
                SpeedSlider(telemetry?.speedTarget ?: 0) { speed ->
                    viewModel.sendCommand("CMD:SPEED:$speed")
                }
                val logMessages by viewModel.debugMessages.collectAsState()
                LogConsole(logMessages)
            }
            // Right Column
            Column(
                modifier = Modifier.weight(1f),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.SpaceAround
            ) {
                RobotCard(telemetry)
                Compass(heading = telemetry?.heading ?: 0)
            }
        }
    }
}

@Composable
fun ScanDialog(devices: List<BluetoothDevice>, onDismiss: () -> Unit, onDeviceSelected: (BluetoothDevice) -> Unit) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Scanned Devices") },
        text = {
            LazyColumn {
                items(devices) { device ->
                    Text(
                        text = device.name ?: "Unknown Device",
                        modifier = Modifier
                            .fillMaxWidth()
                            .clickable { onDeviceSelected(device) }
                            .padding(16.dp)
                    )
                }
            }
        },
        confirmButton = { }
    )
}

@Composable
fun ConnectionStatus(connectionState: BlunoLibrary.connectionStateEnum, battery: Int) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        val statusText: String
        val statusColor: Color
        when (connectionState) {
            BlunoLibrary.connectionStateEnum.isConnected -> {
                statusText = "Connected"
                statusColor = EmeraldGreen
            }
            BlunoLibrary.connectionStateEnum.isConnecting -> {
                statusText = "Connecting"
                statusColor = Orange
            }
            else -> {
                statusText = "Disconnected"
                statusColor = AlertRed
            }
        }
        Text(statusText, color = statusColor)
        Text("🔋 $battery%", color = TextPrimary)
    }
}

@Composable
fun RobotCard(telemetry: RobotTelemetry?) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = MaterialTheme.shapes.medium,
        colors = CardDefaults.cardColors(containerColor = RobotCardDark)
    ) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
            Text("Robot", fontWeight = FontWeight.Bold, fontSize = 20.sp)
            Text("Mode: ${telemetry?.state ?: "--"}")
            Text("Heading: ${telemetry?.heading ?: 0}°")
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.ArrowUpward, contentDescription = "Distance Haut", tint = TextPrimary, modifier = Modifier.size(16.dp))
                Spacer(modifier = Modifier.width(4.dp))
                Text("Haut: ${telemetry?.distanceHaut ?: 0} cm")
            }
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.ArrowDownward, contentDescription = "Distance Bas", tint = TextPrimary, modifier = Modifier.size(16.dp))
                Spacer(modifier = Modifier.width(4.dp))
                Text("Bas: ${telemetry?.distanceBas ?: 0} cm")
            }
        }
    }
}

@Composable
fun SpeedSlider(speed: Int, onSpeedChange: (Int) -> Unit) {
    var sliderPosition by remember { mutableFloatStateOf(speed.toFloat()) }
    Column(modifier = Modifier.fillMaxWidth(), horizontalAlignment = Alignment.CenterHorizontally) {
        Slider(
            value = sliderPosition,
            onValueChange = { sliderPosition = it },
            onValueChangeFinished = { onSpeedChange(sliderPosition.toInt()) },
            valueRange = 0f..255f,
            colors = SliderDefaults.colors(thumbColor = AccentBlue, activeTrackColor = AccentBlue)
        )
        Text("Speed: ${sliderPosition.toInt()}%", color = TextPrimary, fontSize = 16.sp)
    }
}

@Composable
fun Joystick(onMove: (angle: Int, distance: Int) -> Unit) {
    AndroidView(
        modifier = Modifier.size(260.dp),
        factory = { context ->
            JoystickView(context, null).apply {
                setOnJoystickMovedListener(object : JoystickView.JoystickListener {
                    override fun onJoystickMoved(angle: Int, distance: Int) {
                        onMove(angle, distance)
                    }
                    override fun onJoystickReleased() {
                        onMove(0, 0)
                    }
                })
            }
        }
    )
}

@Composable
fun Compass(heading: Int) {
    AndroidView(
        modifier = Modifier.size(200.dp),
        factory = { context ->
            CompassView(context, null)
        },
        update = { view ->
            view.setHeading(heading)
        }
    )
}

@Composable
fun StopButton(onClick: () -> Unit) {
    Button(
        onClick = onClick,
        modifier = Modifier
            .fillMaxWidth()
            .height(80.dp)
            .padding(horizontal = 64.dp, vertical = 16.dp),
        shape = MaterialTheme.shapes.medium,
        colors = ButtonDefaults.buttonColors(containerColor = AlertRed)
    ) {
        Text("STOP", fontWeight = FontWeight.Bold, fontSize = 28.sp, color = TextPrimary)
    }
}

@Composable
fun LogConsole(logMessages: List<Pair<String, String>>) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .height(100.dp),
        shape = MaterialTheme.shapes.medium,
        colors = CardDefaults.cardColors(containerColor = LogConsoleDark)
    ) {
        LazyColumn(modifier = Modifier.padding(16.dp)) {
            items(logMessages) { (timestamp, message) ->
                Text(
                    text = "$timestamp: $message",
                    color = TextPrimary,
                    fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace,
                    fontSize = 12.sp
                )
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun NumberInputDialog(
    title: String,
    onDismiss: () -> Unit,
    onResult: (Int) -> Unit
) {
    var text by remember { mutableStateOf("") }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text(title) },
        text = {
            OutlinedTextField(
                value = text,
                onValueChange = { text = it },
                label = { Text("Value") },
                keyboardOptions = KeyboardOptions(
                    keyboardType = KeyboardType.Number
                )
            )
        },
        confirmButton = {
            Button(
                onClick = {
                    val value = text.toIntOrNull()
                    if (value != null) {
                        onResult(value)
                    }
                }
            ) {
                Text("Send")
            }
        },
        dismissButton = {
            Button(onClick = onDismiss) {
                Text("Cancel")
            }
        }
    )
}

@Preview(showBackground = true)
@Composable
fun DefaultPreview() {
    NonoControllerTheme {
        NonoControllerApp()
    }
}