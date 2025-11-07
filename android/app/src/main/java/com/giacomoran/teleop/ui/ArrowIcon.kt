package com.giacomoran.teleop.ui

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.giacomoran.teleop.R

/**
 * Arrow direction enum
 */
enum class ArrowDirection {
    LEFT, RIGHT, UP, DOWN
}

/**
 * Beautiful arrow icon based on SVG design
 * Uses Android Vector Drawable for clean, maintainable code
 */
@Composable
fun ArrowIcon(
    direction: ArrowDirection,
    color: Color,
    size: Dp,
    modifier: Modifier = Modifier
) {
    // Calculate rotation angle based on direction
    val rotationDegrees = when (direction) {
        ArrowDirection.RIGHT -> 0f
        ArrowDirection.LEFT -> 180f
        ArrowDirection.UP -> -90f
        ArrowDirection.DOWN -> 90f
    }

    Image(
        painter = painterResource(id = R.drawable.ic_arrow_right),
        contentDescription = null,
        modifier = modifier
            .size(size)
            .graphicsLayer {
                rotationZ = rotationDegrees
            },
        colorFilter = ColorFilter.tint(color)
    )
}
