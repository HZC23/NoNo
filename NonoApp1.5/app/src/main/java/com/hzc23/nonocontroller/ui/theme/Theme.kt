package com.hzc23.nonocontroller.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.SideEffect
import androidx.compose.ui.graphics.Color
import com.google.accompanist.systemuicontroller.rememberSystemUiController

private val DarkColorPalette = darkColorScheme(
    primary = AccentBlue,
    onPrimary = TextPrimary,
    secondary = EmeraldGreen,
    onSecondary = TextPrimary,
    error = AlertRed,
    onError = TextPrimary,
    background = BackgroundDark,
    onBackground = TextPrimary,
    surface = MainCardDark,
    onSurface = TextPrimary,
    surfaceVariant = RobotCardDark,
    outline = StrokeGrey
)

@Composable
fun NonoControllerTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    content: @Composable () -> Unit
) {
    val colors = DarkColorPalette
    val systemUiController = rememberSystemUiController()
    val useDarkIcons = !darkTheme

    SideEffect {
        systemUiController.setSystemBarsColor(
            color = Color.Transparent,
            darkIcons = useDarkIcons
        )
    }

    MaterialTheme(
        colorScheme = colors,
        typography = Typography,
        shapes = Shapes,
        content = content
    )
}