package com.hzc23.nonocontroller.ui

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.* 
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.layout.ExperimentalLayoutApi

@OptIn(ExperimentalMaterial3Api::class, ExperimentalLayoutApi::class)
@Composable
fun ModeBand(
    currentMode: String,
    onModeSelected: (String) -> Unit,
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
                text = "Mode: $currentMode",
                style = MaterialTheme.typography.headlineSmall,
                color = MaterialTheme.colorScheme.onSurface
            )

            Spacer(modifier = Modifier.height(16.dp))

            var selectedMode by remember { mutableStateOf(currentMode) }

            FlowRow(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                ModeChip(
                    text = "Manuel",
                    isSelected = selectedMode == "Manuel",
                    onClick = {
                        selectedMode = "Manuel"
                        onModeSelected("Manuel")
                    }
                )
                ModeChip(
                    text = "Auto",
                    isSelected = selectedMode == "Auto",
                    onClick = {
                        selectedMode = "Auto"
                        onModeSelected("Auto")
                    }
                )
                ModeChip(
                    text = "Obstacle",
                    isSelected = selectedMode == "Obstacle",
                    onClick = {
                        selectedMode = "Obstacle"
                        onModeSelected("Obstacle")
                    }
                )
                ModeChip(
                    text = "Détection PIR",
                    isSelected = selectedMode == "Détection PIR",
                    onClick = {
                        selectedMode = "Détection PIR"
                        onModeSelected("Détection PIR")
                    }
                )
                ModeChip(
                    text = "Cap",
                    isSelected = selectedMode == "Cap",
                    onClick = {
                        selectedMode = "Cap"
                        onModeSelected("Cap")
                    }
                )
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ModeChip(
    text: String,
    isSelected: Boolean,
    onClick: () -> Unit,
    enabled: Boolean = true
) {
    FilterChip(
        selected = isSelected,
        onClick = onClick,
        label = { Text(text) },
        modifier = Modifier.heightIn(min = 48.dp), // Ensuring touch target is at least 48dp
        enabled = enabled,
        shape = MaterialTheme.shapes.small, // Using small shape for chips
        colors = FilterChipDefaults.filterChipColors(
            selectedContainerColor = MaterialTheme.colorScheme.primary,
            selectedLabelColor = MaterialTheme.colorScheme.onPrimary,
            containerColor = MaterialTheme.colorScheme.surfaceVariant,
            labelColor = MaterialTheme.colorScheme.onSurfaceVariant
        )
    )
}

@Preview(showBackground = true)
@Composable
fun ModeBandPreview() {
    MaterialTheme {
        ModeBand(
            currentMode = "Manuel",
            onModeSelected = {}
        )
    }
}