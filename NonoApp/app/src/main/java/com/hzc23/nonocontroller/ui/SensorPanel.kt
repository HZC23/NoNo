package com.hzc23.nonocontroller.ui

import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.BatteryChargingFull
import androidx.compose.material.icons.filled.Sensors
import androidx.compose.material3.* 
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp

@Composable
fun SensorPanel(
    speed: Int,
    distance: Int,
    cap: Int,
    targetCap: Int,
    battery: Int,
    pirStatus: String,
    lcdText: String,
    modifier: Modifier = Modifier
) {
    Card(
        modifier = modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainerHigh)
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            Text(
                text = "Sensors",
                style = MaterialTheme.typography.headlineSmall,
                color = MaterialTheme.colorScheme.onSurface
            )

            Spacer(modifier = Modifier.height(16.dp))

            // Speed
            Text(
                text = "Vitesse: $speed",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface
            )
            LinearProgressIndicator(
                progress = { speed / 255f },
                modifier = Modifier.fillMaxWidth().padding(top = 4.dp),
                color = MaterialTheme.colorScheme.primary
            )

            Spacer(modifier = Modifier.height(8.dp))

            // Distance
            Text(
                text = "Distance US: $distance cm",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface
            )
            LinearProgressIndicator(
                progress = { distance / 100f },
                modifier = Modifier.fillMaxWidth().padding(top = 4.dp),
                color = MaterialTheme.colorScheme.primary
            )

            Spacer(modifier = Modifier.height(8.dp))

            // Cap
            Text(
                text = "Cap: $cap° / $targetCap°",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface
            )
            LinearProgressIndicator(
                progress = { cap / 359f },
                modifier = Modifier.fillMaxWidth().padding(top = 4.dp),
                color = MaterialTheme.colorScheme.primary
            )

            Spacer(modifier = Modifier.height(8.dp))

            // Battery
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    Icons.Filled.BatteryChargingFull,
                    contentDescription = "Battery",
                    tint = MaterialTheme.colorScheme.onSurface,
                    modifier = Modifier.size(24.dp)
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "Batterie: $battery%",
                    style = MaterialTheme.typography.bodyLarge,
                    color = MaterialTheme.colorScheme.onSurface
                )
            }

            Spacer(modifier = Modifier.height(8.dp))

            // PIR
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    Icons.Filled.Sensors,
                    contentDescription = "PIR Sensor",
                    tint = MaterialTheme.colorScheme.onSurface,
                    modifier = Modifier.size(24.dp)
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "PIR: $pirStatus",
                    style = MaterialTheme.typography.bodyLarge,
                    color = MaterialTheme.colorScheme.onSurface
                )
            }

            Spacer(modifier = Modifier.height(8.dp))

            // LCD
            Text(
                text = "LCD: $lcdText",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface
            )
        }
    }
}

@Preview(showBackground = true)
@Composable
fun SensorPanelPreview() {
    MaterialTheme {
        SensorPanel(
            speed = 120,
            distance = 50,
            cap = 180,
            targetCap = 200,
            battery = 75,
            pirStatus = "aucun mouvement",
            lcdText = "Hello Robot!"
        )
    }
}