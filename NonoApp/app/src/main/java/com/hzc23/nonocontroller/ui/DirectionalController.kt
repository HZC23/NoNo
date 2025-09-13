package com.hzc23.nonocontroller.ui

import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp

@Composable
fun DirectionalController(
    onUp: () -> Unit,
    onDown: () -> Unit,
    onLeft: () -> Unit,
    onRight: () -> Unit,
    onStop: () -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier.padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center
    ) {
        Button(
            onClick = onUp,
            modifier = Modifier.size(64.dp) // Ensuring touch target is at least 48dp
        ) {
            Icon(Icons.Filled.KeyboardArrowUp, contentDescription = "Up")
        }
        Spacer(modifier = Modifier.height(8.dp))
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center
        ) {
            Button(
                onClick = onLeft,
                modifier = Modifier.size(64.dp)
            ) {
                Icon(Icons.Filled.KeyboardArrowLeft, contentDescription = "Left")
            }
            Spacer(modifier = Modifier.width(8.dp))
            Button(
                onClick = onStop,
                modifier = Modifier.size(64.dp)
            ) {
                Icon(Icons.Filled.Stop, contentDescription = "Stop")
            }
            Spacer(modifier = Modifier.width(8.dp))
            Button(
                onClick = onRight,
                modifier = Modifier.size(64.dp)
            ) {
                Icon(Icons.Filled.KeyboardArrowRight, contentDescription = "Right")
            }
        }
        Spacer(modifier = Modifier.height(8.dp))
        Button(
            onClick = onDown,
            modifier = Modifier.size(64.dp)
        ) {
            Icon(Icons.Filled.KeyboardArrowDown, contentDescription = "Down")
        }
    }
}

@Preview(showBackground = true)
@Composable
fun DirectionalControllerPreview() {
    MaterialTheme {
        DirectionalController(
            onUp = {},
            onDown = {},
            onLeft = {},
            onRight = {},
            onStop = {}
        )
    }
}