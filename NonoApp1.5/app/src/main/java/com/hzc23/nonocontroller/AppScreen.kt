package com.hzc23.nonocontroller

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hzc23.nonocontroller.ui.theme.NonoControllerTheme
import com.hzc23.nonocontroller.ui.theme.BackgroundDark
import com.hzc23.nonocontroller.ui.theme.MainCardDark

@Composable
fun AppScreen() {
    NonoControllerTheme {
        Surface(
            modifier = Modifier.fillMaxSize(),
            color = BackgroundDark
        ) {
            Column(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(16.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Top
            ) {
                // Header
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(bottom = 16.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(
                        text = "Nono Controller",
                        color = MaterialTheme.colorScheme.onBackground,
                        fontSize = 32.sp,
                        fontWeight = FontWeight.Bold
                    )
                    // Placeholder for Bluetooth icon and status
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        // Icon placeholder
                        Box(
                            modifier = Modifier
                                .size(24.dp)
                                .background(Color.Blue, RoundedCornerShape(4.dp))
                        )
                        Spacer(modifier = Modifier.width(8.dp))
                        Text(
                            text = "Connected",
                            color = MaterialTheme.colorScheme.onBackground,
                            fontSize = 16.sp
                        )
                    }
                }

                // Main Content Card
                Surface(
                    modifier = Modifier
                        .fillMaxWidth(0.9f) // Approx 78-85% width
                        .fillMaxHeight(0.9f), // Approx 68% height
                    color = MainCardDark,
                    shape = RoundedCornerShape(24.dp) // Approx 18-24px
                ) {
                    Row(
                        modifier = Modifier
                            .fillMaxSize()
                            .padding(24.dp), // Approx 24-32px padding
                        horizontalArrangement = Arrangement.SpaceAround
                    ) {
                        // Left Column (Joystick, Speed Slider, Console Log)
                        Column(
                            modifier = Modifier
                                .weight(1f)
                                .fillMaxHeight()
                                .padding(end = 24.dp), // Gutter approx 40-60px
                            horizontalAlignment = Alignment.CenterHorizontally,
                            verticalArrangement = Arrangement.SpaceBetween
                        ) {
                            // Placeholder for Joystick
                            Box(
                                modifier = Modifier
                                    .size(200.dp) // Approx 260px diameter
                                    .background(Color.DarkGray, RoundedCornerShape(100.dp))
                            )
                            // Placeholder for Speed Slider
                            Box(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(40.dp)
                                    .background(Color.Gray, RoundedCornerShape(20.dp))
                            )
                            Text(
                                text = "Speed: 80%",
                                color = MaterialTheme.colorScheme.onBackground,
                                fontSize = 18.sp
                            )
                            // Placeholder for Console Log
                            Box(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(150.dp)
                                    .background(Color.Black, RoundedCornerShape(12.dp))
                            )
                        }

                        // Right Column (Robot Card, Compass, STOP Button)
                        Column(
                            modifier = Modifier
                                .weight(1f)
                                .fillMaxHeight()
                                .padding(start = 24.dp), // Gutter approx 40-60px
                            horizontalAlignment = Alignment.CenterHorizontally,
                            verticalArrangement = Arrangement.SpaceBetween
                        ) {
                            // Placeholder for Robot Card
                            Surface(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(150.dp),
                                color = MaterialTheme.colorScheme.surfaceVariant,
                                shape = RoundedCornerShape(12.dp)
                            ) {
                                Column(modifier = Modifier.padding(16.dp)) {
                                    Text(text = "Robot", fontWeight = FontWeight.Bold, fontSize = 20.sp)
                                    Text(text = "Mode: Auto", fontSize = 16.sp)
                                    Text(text = "Heading: 0°", fontSize = 16.sp)
                                    Row(
                                        modifier = Modifier.fillMaxWidth(),
                                        horizontalArrangement = Arrangement.SpaceAround
                                    ) {
                                        Text(text = "Distance haut: 0 cm", fontSize = 16.sp)
                                        Text(text = "Distance bas: 0 cm", fontSize = 16.sp)
                                    }
                                }
                            }
                            // Placeholder for Compass
                            Box(
                                modifier = Modifier
                                    .size(150.dp) // Approx 170-200px diameter
                                    .background(Color.LightGray, RoundedCornerShape(75.dp))
                            )
                            // Placeholder for STOP Button
                            Box(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(80.dp) // Approx 80-100px height
                                    .background(Color.Red, RoundedCornerShape(28.dp)),
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    text = "STOP",
                                    color = Color.White,
                                    fontSize = 28.sp,
                                    fontWeight = FontWeight.Bold
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}

@Preview(showBackground = true, widthDp = 1024, heightDp = 768)
@Composable
fun AppScreenPreview() {
    AppScreen()
}
