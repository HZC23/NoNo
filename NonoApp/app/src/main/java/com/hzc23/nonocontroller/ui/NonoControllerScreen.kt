package com.hzc23.nonocontroller.ui

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowLeft
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowRight
import androidx.compose.material3.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.hapticfeedback.HapticFeedbackType
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalHapticFeedback
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hzc23.nonocontroller.MainViewModel
import com.hzc23.nonocontroller.MainActivity
import com.hzc23.nonocontroller.SettingsDataStore
import com.hzc23.nonocontroller.ui.theme.NonoControllerTheme
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.FilterChip
import androidx.compose.material3.Slider

@OptIn(ExperimentalLayoutApi::class)
@Composable
fun NonoControllerScreen(
    viewModel: MainViewModel,
    onSendCommand: (String) -> Unit,
    onSetMode: (MainActivity.RobotMode) -> Unit,
    onConnect: () -> Unit
) {
    val batteryLevel by viewModel.batteryLevel.collectAsState()
    val currentCap by viewModel.currentCap.collectAsState()
    val distanceLeft by viewModel.distanceLeft.collectAsState()
    val distanceRight by viewModel.distanceRight.collectAsState()
    val debugMessages by viewModel.debugMessages.collectAsState()
    val isLayoutInverted by viewModel.isLayoutInverted.collectAsState()
    val isDarkModeEnabled by viewModel.isDarkModeEnabled.collectAsState()
    var showCapDialog by remember { mutableStateOf(false) }

    if (showCapDialog) {
        var cap by remember { mutableStateOf("") }
        AlertDialog(
            onDismissRequest = { showCapDialog = false },
            title = { Text("Set Target Cap") },
            text = {
                OutlinedTextField(
                    value = cap,
                    onValueChange = { cap = it },
                    label = { Text("0-359") }
                )
            },
            confirmButton = {
                Button(onClick = {
                    val angle = cap.toIntOrNull()?.coerceIn(0, 359)
                    if (angle != null) {
                        onSendCommand("${MainActivity.RobotMode.CAP.command}=$angle")
                    }
                    showCapDialog = false
                }) {
                    Text("Set")
                }
            }
        )
    }

    NonoControllerTheme(darkTheme = isDarkModeEnabled) {
        Surface(
            modifier = Modifier.fillMaxSize(),
            color = MaterialTheme.colorScheme.background
        ) {
            val leftPanel = @Composable {
                Column(
                    modifier = Modifier
                        .padding(16.dp)
                        .fillMaxSize()
                ) {
                    Text("Nono Controller", style = MaterialTheme.typography.headlineMedium)
                    Spacer(modifier = Modifier.height(16.dp))
                    TopBar(
                        batteryLevel = batteryLevel,
                        onConnect = onConnect,
                        onToggleLayoutInversion = { viewModel.toggleLayoutInversion() },
                        onToggleDarkMode = { viewModel.toggleDarkMode() },
                        isLayoutInverted = isLayoutInverted,
                        isDarkModeEnabled = isDarkModeEnabled
                    )
                    Spacer(modifier = Modifier.height(16.dp))
                    SensorReadings(currentCap, distanceLeft, distanceRight)
                    Spacer(modifier = Modifier.height(16.dp)) // Spacer before new controls
                    LightControls(onSendCommand) // New Light Controls
                    Spacer(modifier = Modifier.height(16.dp)) // Spacer between new controls
                    UtilityControls(onSendCommand) // New Utility Controls
                    Spacer(modifier = Modifier.height(16.dp)) // Spacer before Console
                    Console(debugMessages)
                }
            }

            val rightPanel = @Composable {
                Column(
                    modifier = Modifier
                        .padding(16.dp)
                        .fillMaxSize(),
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    DirectionalController(onSendCommand)
                    Spacer(modifier = Modifier.height(16.dp))
                    ModeButtons(onSetMode) { showCapDialog = true }
                    Spacer(modifier = Modifier.height(16.dp))
                    AdvancedControls(onSendCommand)
                }
            }

            Row(modifier = Modifier.fillMaxSize()) {
                if (isLayoutInverted) {
                    Box(modifier = Modifier.weight(1f)) { rightPanel() }
                    Box(modifier = Modifier.weight(1f)) { leftPanel() }
                } else {
                    Box(modifier = Modifier.weight(1f)) { leftPanel() }
                    Box(modifier = Modifier.weight(1f)) { rightPanel() }
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun TopBar(
    batteryLevel: Int,
    onConnect: () -> Unit,
    onToggleLayoutInversion: () -> Unit,
    onToggleDarkMode: () -> Unit,
    isLayoutInverted: Boolean,
    isDarkModeEnabled: Boolean
) {
    val batteryColor by animateColorAsState(
        when {
            batteryLevel > 50 -> Color.Green
            batteryLevel > 20 -> Color.Yellow
            else -> Color.Red
        },
        label = "batteryColor"
    )

    var showSettingsSheet by remember { mutableStateOf(false) }

    if (showSettingsSheet) {
        ModalBottomSheet(onDismissRequest = { showSettingsSheet = false }) {
            Column(modifier = Modifier.padding(16.dp)) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Inverser le layout", style = MaterialTheme.typography.titleMedium)
                    Switch(
                        checked = isLayoutInverted,
                        onCheckedChange = { onToggleLayoutInversion() }
                    )
                }
                Spacer(modifier = Modifier.height(16.dp))
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Mode sombre", style = MaterialTheme.typography.titleMedium)
                    Switch(
                        checked = isDarkModeEnabled,
                        onCheckedChange = { onToggleDarkMode() }
                    )
                }
            }
        }
    }

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically, modifier = Modifier.padding(8.dp)) {
                Icon(Icons.Default.BatteryChargingFull, contentDescription = "Battery", tint = batteryColor)
                Spacer(modifier = Modifier.width(8.dp))
                Text(text = "$batteryLevel%", fontSize = 20.sp, fontWeight = FontWeight.Bold)
            }
        }
        Button(onClick = onConnect) {
            Text("Connect")
        }
        IconButton(onClick = { showSettingsSheet = true }) {
            Icon(Icons.Default.Settings, contentDescription = "Settings")
        }
    }
}

@OptIn(ExperimentalLayoutApi::class, ExperimentalMaterial3Api::class)
@Composable
fun ModeButtons(onSetMode: (MainActivity.RobotMode) -> Unit, onCapClick: () -> Unit) {
    var selectedMode by remember { mutableStateOf(MainActivity.RobotMode.MANUAL) }

    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp), modifier = Modifier.fillMaxWidth(0.9f)) {
        FilterChip(
            selected = selectedMode == MainActivity.RobotMode.MANUAL,
            onClick = { selectedMode = MainActivity.RobotMode.MANUAL; onSetMode(MainActivity.RobotMode.MANUAL) },
            label = { Text("Manuel") }
        )
        FilterChip(
            selected = selectedMode == MainActivity.RobotMode.AUTO,
            onClick = { selectedMode = MainActivity.RobotMode.AUTO; onSetMode(MainActivity.RobotMode.AUTO) },
            label = { Text("Auto") }
        )
        FilterChip(
            selected = selectedMode == MainActivity.RobotMode.OBSTACLE,
            onClick = { selectedMode = MainActivity.RobotMode.OBSTACLE; onSetMode(MainActivity.RobotMode.OBSTACLE) },
            label = { Text("Obstacle") }
        )
        FilterChip(
            selected = selectedMode == MainActivity.RobotMode.PIR,
            onClick = { selectedMode = MainActivity.RobotMode.PIR; onSetMode(MainActivity.RobotMode.PIR) },
            label = { Text("PIR") }
        )
        FilterChip(
            selected = selectedMode == MainActivity.RobotMode.CAP,
            onClick = { onCapClick() },
            label = { Text("Cap") }
        )
    }
}


@Composable
fun LightControls(onSendCommand: (String) -> Unit) {
    Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp), modifier = Modifier.fillMaxWidth()) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier.padding(16.dp)
        ) {
            Text("Phare", style = MaterialTheme.typography.titleMedium)
            Spacer(modifier = Modifier.height(8.dp))
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { onSendCommand("on") }, modifier = Modifier.weight(1f)) { Text("ON") }
                Button(onClick = { onSendCommand("off") }, modifier = Modifier.weight(1f)) { Text("OFF") }
            }
        }
    }
}

@Composable
fun UtilityControls(onSendCommand: (String) -> Unit) {
    Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp), modifier = Modifier.fillMaxWidth()) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier.padding(16.dp)
        ) {
            Text("Utilitaires", style = MaterialTheme.typography.titleMedium)
            Spacer(modifier = Modifier.height(8.dp))
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { onSendCommand("scan") }, modifier = Modifier.weight(1f)) { Text("Scan") }
                Button(onClick = { onSendCommand("calibrer") }, modifier = Modifier.weight(1f)) { Text("Calibrer") }
            }
        }
    }
}

@Composable
fun AdvancedControls(onSendCommand: (String) -> Unit) {
    var sliderValue by remember { mutableStateOf(150f) }
    Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp), modifier = Modifier.fillMaxWidth(0.9f)) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier.padding(16.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "Vitesse: ${sliderValue.toInt()}",
                    modifier = Modifier.width(100.dp) // Explicitly set width for the text
                )
                Slider(
                    value = sliderValue,
                    onValueChange = { sliderValue = it },
                    valueRange = 0f..255f,
                    onValueChangeFinished = {
                        onSendCommand("vitesse${sliderValue.toInt()}")
                    },
                    modifier = Modifier.weight(1f) // Allow slider to take available space
                )
            }

        }
    }
}


@Composable
fun SensorReadings(currentCap: Int, distanceLeft: Int, distanceRight: Int) {
    Column(verticalArrangement = Arrangement.spacedBy(16.dp)) {
        Compass(currentCap)
        DistanceSensors(distanceLeft, distanceRight)
    }
}

@Composable
fun Compass(angle: Int) {
    val rotation by animateFloatAsState(targetValue = angle.toFloat(), label = "compassRotation")
    val surfaceVariant = MaterialTheme.colorScheme.surfaceVariant
    val onSurfaceVariant = MaterialTheme.colorScheme.onSurfaceVariant
    val primaryColor = MaterialTheme.colorScheme.primary

    Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)) {
        Column(
            modifier = Modifier.padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text("Orientation", style = MaterialTheme.typography.titleMedium)
            Spacer(modifier = Modifier.height(8.dp))
            Box(contentAlignment = Alignment.Center) {
                Canvas(modifier = Modifier.size(120.dp)) {
                    drawCircle(color = surfaceVariant)
                    for (i in 0..360 step 30) {
                        val angleRad = Math.toRadians(i.toDouble())
                        val lineLength = if (i % 90 == 0) 20f else 10f
                        drawLine(
                            color = onSurfaceVariant,
                            start = center.copy(
                                x = center.x + (size.minDimension / 2 - lineLength) * kotlin.math.cos(angleRad).toFloat(),
                                y = center.y + (size.minDimension / 2 - lineLength) * kotlin.math.sin(angleRad).toFloat()
                            ),
                            end = center.copy(
                                x = center.x + (size.minDimension / 2) * kotlin.math.cos(angleRad).toFloat(),
                                y = center.y + (size.minDimension / 2) * kotlin.math.sin(angleRad).toFloat()
                            ),
                            strokeWidth = if (i % 90 == 0) 4f else 2f
                        )
                    }
                }
                Icon(
                    Icons.Default.Navigation,
                    contentDescription = "Compass Needle",
                    modifier = Modifier
                        .size(100.dp)
                        .rotate(rotation),
                    tint = primaryColor
                )
            }
            Text("$angle°", fontSize = 24.sp, fontWeight = FontWeight.Bold)
        }
    }
}

@Composable
fun DistanceSensors(left: Int, right: Int) {
    Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)) {
        Row(
            modifier = Modifier.padding(16.dp),
            horizontalArrangement = Arrangement.SpaceEvenly,
            verticalAlignment = Alignment.CenterVertically
        ) {
            SensorCircle(value = left, unit = "cm")
            SensorCircle(value = right, unit = "cm")
        }
    }
}

@Composable
fun SensorCircle(value: Int, unit: String) {
    val color by animateColorAsState(
        when {
            value > 50 -> Color.Green
            value > 20 -> Color.Yellow
            else -> Color.Red
        },
        label = "sensorColor"
    )
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text("$value $unit", fontSize = 20.sp, fontWeight = FontWeight.Bold, color = color)
    }
}

@Composable
fun Console(messages: List<Pair<String, String>>) {
    Card(elevation = CardDefaults.cardElevation(defaultElevation = 4.dp), modifier = Modifier.fillMaxHeight()) {
        Column(modifier = Modifier.padding(16.dp)) {
            Text("Console", style = MaterialTheme.typography.titleMedium)
            Spacer(modifier = Modifier.height(8.dp))
            LazyColumn(
                modifier = Modifier.fillMaxSize(),
                reverseLayout = true
            ) {
                items(messages) { (timestamp, message) -> // Removed .reversed() here
                    Row {
                        Text("[$timestamp] ", style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.primary)
                        Text(message, style = MaterialTheme.typography.bodySmall)
                    }
                }
            }
        }
    }
}

@Composable
fun DirectionalController(onSendCommand: (String) -> Unit) {
    val haptic = LocalHapticFeedback.current
    Box(
        modifier = Modifier
            .size(240.dp)
            .clip(CircleShape)
            .background(MaterialTheme.colorScheme.surfaceVariant),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            ControllerButton(icon = Icons.Default.KeyboardArrowUp, onClick = { onSendCommand("U") }, onRelease = { onSendCommand("stop") }, haptic = haptic)
            Row(verticalAlignment = Alignment.CenterVertically) {
                ControllerButton(icon = Icons.AutoMirrored.Filled.KeyboardArrowLeft, onClick = { onSendCommand("L") }, onRelease = { onSendCommand("stop") }, haptic = haptic)
                Spacer(modifier = Modifier.width(80.dp))
                ControllerButton(icon = Icons.AutoMirrored.Filled.KeyboardArrowRight, onClick = { onSendCommand("R") }, onRelease = { onSendCommand("stop") }, haptic = haptic)
            }
            ControllerButton(icon = Icons.Default.KeyboardArrowDown, onClick = { onSendCommand("D") }, onRelease = { onSendCommand("stop") }, haptic = haptic)
        }
        ControllerButton(icon = Icons.Default.Stop, onClick = { onSendCommand("stop") }, onRelease = {}, haptic = haptic, isStop = true)
    }
}

@Composable
fun ControllerButton(icon: ImageVector, onClick: () -> Unit, onRelease: () -> Unit, haptic: androidx.compose.ui.hapticfeedback.HapticFeedback, isStop: Boolean = false) {
    var isPressed by remember { mutableStateOf(false) }
    val size by animateDpAsState(if (isPressed) 72.dp else 80.dp, label = "buttonSize") // Re-introduce animation for size
    val color by animateColorAsState(if (isPressed) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.secondary, label = "buttonColor") // Re-introduce animation for color

    Box(
        modifier = Modifier
            .size(if (isStop) 64.dp else size) // Use animated size
            .clip(CircleShape)
            .background(if (isStop) MaterialTheme.colorScheme.errorContainer else color) // Use animated color
            .pointerInput(Unit) {
                detectTapGestures(
                    onPress = {
                        isPressed = true
                        haptic.performHapticFeedback(HapticFeedbackType.LongPress)
                        onClick()
                        tryAwaitRelease()
                        isPressed = false
                        onRelease()
                    }
                )
            },
        contentAlignment = Alignment.Center
    ) {
        Icon(icon, contentDescription = null, modifier = Modifier.fillMaxSize(0.7f), tint = if (isStop) MaterialTheme.colorScheme.onErrorContainer else Color.White)
    }
}
@Preview(showBackground = true, widthDp = 800, heightDp = 480)
@Composable
fun DefaultPreview() {
    val context = LocalContext.current
    val settingsDataStore = remember { SettingsDataStore(context) }
    val viewModel = remember { MainViewModel(settingsDataStore) }

    // Populate with some fake data for preview
    LaunchedEffect(Unit) {
        viewModel.onSerialReceived("Batterie: 88%")
        viewModel.onSerialReceived("Cap: 123, Cible: 90")
        viewModel.onSerialReceived("Distance US: 34 cm")
        viewModel.onSerialReceived("Distance US: 56 cm")
    }

    NonoControllerScreen(
        viewModel = viewModel,
        onSendCommand = {},
        onSetMode = {},
        onConnect = {}
    )
}