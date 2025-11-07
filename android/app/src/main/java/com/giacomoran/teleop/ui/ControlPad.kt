package com.giacomoran.teleop.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.hapticfeedback.HapticFeedbackType
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalHapticFeedback
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.abs

/**
 * Data class representing the control pad output
 */
data class ControlPadOutput(
    val x: Float,      // -1.0 to 1.0, left is negative, right is positive
    val y: Float,      // -1.0 to 1.0, down is negative, up is positive (for gripper)
    val isFineControl: Boolean,  // true if left side (fine), false if right side (normal)
    val isActive: Boolean        // true if touch is active and outside dead zone
)

/**
 * Control pad for teleoperating the robot
 * - Left side: half-speed fine movements
 * - Right side: normal movements
 * - Y axis: controls gripper
 * - Central vertical dead zone to prevent movements when releasing touch
 */
@Composable
fun ControlPad(
    onControlChange: (ControlPadOutput) -> Unit,
    modifier: Modifier = Modifier
) {
    val haptic = LocalHapticFeedback.current
    val colorScheme = MaterialTheme.colorScheme

    var touchPosition by remember { mutableStateOf<Offset?>(null) }
    var isActive by remember { mutableStateOf(false) }

    // Dead zone width as fraction of pad width (e.g., 0.15 = 15% from center)
    val deadZoneFraction = 0.15f

    BoxWithConstraints(modifier = modifier) {
        val padWidth = constraints.maxWidth.toFloat()
        val padHeight = constraints.maxHeight.toFloat()
        val deadZoneWidth = padWidth * deadZoneFraction
        val centerX = padWidth / 2f
        val centerY = padHeight / 2f

        Box(
            modifier = Modifier
                .fillMaxSize()
                .clip(RoundedCornerShape(16.dp))
                .background(
                    color = colorScheme.surfaceContainerHighest,
                    shape = RoundedCornerShape(16.dp)
                )
                .pointerInput(Unit) {
                    detectDragGestures(
                        onDragStart = { offset ->
                            touchPosition = offset
                            haptic.performHapticFeedback(HapticFeedbackType.LongPress)
                        },
                        onDrag = { change, _ ->
                            touchPosition = change.position
                            val relativeX = change.position.x - centerX
                            val relativeY = change.position.y - centerY

                            // Check if outside vertical dead zone
                            val isOutsideDeadZone = abs(relativeX) > deadZoneWidth / 2f

                            if (isOutsideDeadZone) {
                                isActive = true
                                // Normalize to -1.0 to 1.0 range
                                val maxDistanceX = padWidth / 2f - deadZoneWidth / 2f
                                val maxDistanceY = padHeight / 2f
                                val normalizedX = (relativeX / maxDistanceX).coerceIn(-1f, 1f)
                                val normalizedY = (-relativeY / maxDistanceY).coerceIn(-1f, 1f)

                                // Determine if fine control (left side) or normal (right side)
                                val isFineControl = normalizedX < 0

                                // Apply half speed for fine control
                                val adjustedX = if (isFineControl) normalizedX * 0.5f else normalizedX

                                onControlChange(
                                    ControlPadOutput(
                                        x = adjustedX,
                                        y = normalizedY,
                                        isFineControl = isFineControl,
                                        isActive = true
                                    )
                                )
                            } else {
                                isActive = false
                                onControlChange(
                                    ControlPadOutput(
                                        x = 0f,
                                        y = 0f,
                                        isFineControl = false,
                                        isActive = false
                                    )
                                )
                            }
                        },
                        onDragEnd = {
                            touchPosition = null
                            isActive = false
                            onControlChange(
                                ControlPadOutput(
                                    x = 0f,
                                    y = 0f,
                                    isFineControl = false,
                                    isActive = false
                                )
                            )
                        },
                        onDragCancel = {
                            touchPosition = null
                            isActive = false
                            onControlChange(
                                ControlPadOutput(
                                    x = 0f,
                                    y = 0f,
                                    isFineControl = false,
                                    isActive = false
                                )
                            )
                        }
                    )
                }
        ) {
            Canvas(modifier = Modifier.fillMaxSize()) {
                val width = size.width
                val height = size.height
                val centerX = width / 2f

                // Draw left side (fine control) background
                drawRect(
                    color = colorScheme.primaryContainer.copy(alpha = 0.2f),
                    topLeft = Offset(0f, 0f),
                    size = Size(centerX, height)
                )

                // Draw right side (normal control) background
                drawRect(
                    color = colorScheme.secondaryContainer.copy(alpha = 0.2f),
                    topLeft = Offset(centerX, 0f),
                    size = Size(centerX, height)
                )

                // Draw dashed vertical divider in the middle
                val dashPath = androidx.compose.ui.graphics.Path().apply {
                    moveTo(centerX, 0f)
                    lineTo(centerX, height)
                }
                drawPath(
                    path = dashPath,
                    color = colorScheme.onSurfaceVariant.copy(alpha = 0.4f),
                    style = Stroke(
                        width = 2.dp.toPx(),
                        pathEffect = PathEffect.dashPathEffect(floatArrayOf(8f, 8f), 0f)
                    )
                )

                // Draw vertical dead zone bar
                val deadZoneWidthPx = width * deadZoneFraction
                val deadZoneLeft = centerX - deadZoneWidthPx / 2f
                val deadZoneRight = centerX + deadZoneWidthPx / 2f
                drawRect(
                    color = colorScheme.surfaceVariant.copy(alpha = 0.6f),
                    topLeft = Offset(deadZoneLeft, 0f),
                    size = Size(deadZoneWidthPx, height)
                )

                // Draw touch feedback indicator
                touchPosition?.let { pos ->
                    val relativeX = pos.x - centerX
                    val isOutsideDeadZone = abs(relativeX) > deadZoneWidthPx / 2f

                    if (isOutsideDeadZone) {
                        val indicatorColor = if (relativeX < 0) {
                            colorScheme.primary.copy(alpha = 0.8f)
                        } else {
                            colorScheme.secondary.copy(alpha = 0.8f)
                        }

                        // Draw larger touch indicator circle
                        drawCircle(
                            color = indicatorColor,
                            radius = 32.dp.toPx(),
                            center = pos
                        )

                        // Draw smaller inner circle for better visibility
                        drawCircle(
                            color = colorScheme.surface.copy(alpha = 0.4f),
                            radius = 16.dp.toPx(),
                            center = pos
                        )
                    }
                }
            }

            // Labels with beautiful arrow icons
            Row(
                modifier = Modifier
                    .align(Alignment.TopStart)
                    .padding(12.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                ArrowIcon(
                    direction = ArrowDirection.LEFT,
                    color = colorScheme.onPrimaryContainer,
                    size = 20.dp
                )
                Text(
                    text = "FINE",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Medium,
                    color = colorScheme.onPrimaryContainer
                )
            }

            Row(
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .padding(12.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "NORMAL",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Medium,
                    color = colorScheme.onSecondaryContainer
                )
                ArrowIcon(
                    direction = ArrowDirection.RIGHT,
                    color = colorScheme.onSecondaryContainer,
                    size = 20.dp
                )
            }

            Column(
                modifier = Modifier
                    .align(Alignment.CenterEnd)
                    .padding(end = 12.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                ArrowIcon(
                    direction = ArrowDirection.UP,
                    color = colorScheme.onSurfaceVariant,
                    size = 20.dp
                )
                Text(
                    text = "GRIPPER",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Medium,
                    color = colorScheme.onSurfaceVariant
                )
                ArrowIcon(
                    direction = ArrowDirection.DOWN,
                    color = colorScheme.onSurfaceVariant,
                    size = 20.dp
                )
            }
        }
    }
}

