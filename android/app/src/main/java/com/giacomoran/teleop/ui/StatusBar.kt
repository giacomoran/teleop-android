package com.giacomoran.teleop.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.giacomoran.teleop.util.ARCoreSessionManager
import com.giacomoran.teleop.util.ARCoreTrackingTracker
import com.giacomoran.teleop.util.WebSocketClient

/**
 * Status bar component that displays:
 * - Camera tracking status and update rate
 * - Robot connection status
 * - Position and orientation in separate cards
 */
@Composable
fun StatusBar(
    poseState: ARCoreSessionManager.PoseState,
    trackingState: ARCoreTrackingTracker.TrackingMetrics,
    connectionState: WebSocketClient.ConnectionState,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .background(
                color = MaterialTheme.colorScheme.surfaceContainerHighest,
                shape = RoundedCornerShape(bottomStart = 16.dp, bottomEnd = 16.dp)
            )
            .padding(horizontal = 12.dp, vertical = 12.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        // Top row: Camera status and Laptop connection
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            FPSMeter(
                updatesPerSecond = trackingState.updatesPerSecond,
                cameraTrackingState = trackingState.cameraTrackingState,
                isStalled = trackingState.isStalled,
                modifier = Modifier
            )

            ConnectionStatusText(connectionState = connectionState)
        }

        // Position and Orientation cards
        when (val state = poseState) {
            is ARCoreSessionManager.PoseState.Initializing -> {
                Text(
                    text = "Initializing camera tracking...",
                    fontSize = 12.sp,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            is ARCoreSessionManager.PoseState.Tracking -> {
                Text(
                    text = "Waiting for pose data...",
                    fontSize = 12.sp,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            is ARCoreSessionManager.PoseState.Pose -> {
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(IntrinsicSize.Min),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    // Position card
                    DataCard(
                        title = "Position",
                        values = listOf(
                            "X" to formatFloat(state.position[0]),
                            "Y" to formatFloat(state.position[1]),
                            "Z" to formatFloat(state.position[2])
                        ),
                        modifier = Modifier
                            .weight(1f)
                            .fillMaxHeight()
                    )

                    // Orientation card
                    DataCard(
                        title = "Orientation",
                        values = listOf(
                            "X" to formatFloat(state.quaternion[0]),
                            "Y" to formatFloat(state.quaternion[1]),
                            "Z" to formatFloat(state.quaternion[2]),
                            "W" to formatFloat(state.quaternion[3])
                        ),
                        modifier = Modifier
                            .weight(1f)
                            .fillMaxHeight()
                    )
                }
            }
            is ARCoreSessionManager.PoseState.Error -> {
                Text(
                    text = "Error: ${state.message}",
                    fontSize = 12.sp,
                    color = MaterialTheme.colorScheme.error
                )
            }
        }
    }
}

/**
 * Reusable card component for displaying labeled data values
 * Cards are forced to have the same height
 */
@Composable
private fun DataCard(
    title: String,
    values: List<Pair<String, String>>,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .background(
                color = MaterialTheme.colorScheme.surface,
                shape = RoundedCornerShape(8.dp)
            )
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Text(
            text = title,
            fontSize = 11.sp,
            fontWeight = FontWeight.Bold,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )

        Column(
            verticalArrangement = Arrangement.spacedBy(3.dp)
        ) {
            values.forEach { (label, value) ->
                Row(
                    horizontalArrangement = Arrangement.SpaceBetween,
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Text(
                        text = "$label:",
                        fontSize = 11.sp,
                        fontWeight = FontWeight.Medium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.weight(1f)
                    )
                    Text(
                        text = value,
                        fontSize = 11.sp,
                        fontFamily = FontFamily.Monospace,
                        color = MaterialTheme.colorScheme.onSurface
                    )
                }
            }
        }
    }
}

/**
 * Format float to 3 decimal places for display
 */
private fun formatFloat(value: Float): String {
    return String.format("%.3f", value)
}

/**
 * Display connection status text based on WebSocket connection state.
 */
@Composable
private fun ConnectionStatusText(
    connectionState: WebSocketClient.ConnectionState
) {
    val (text, color) = when (connectionState) {
        is WebSocketClient.ConnectionState.Connected -> {
            "Laptop Connected" to Color(0xFF4CAF50) // Green
        }
        is WebSocketClient.ConnectionState.Connecting -> {
            "Connecting..." to Color(0xFFFF9800) // Orange
        }
        is WebSocketClient.ConnectionState.Disconnected -> {
            "Laptop Disconnected" to Color(0xFFF44336) // Red
        }
    }

    Text(
        text = text,
        fontSize = 12.sp,
        fontFamily = FontFamily.Default,
        fontWeight = FontWeight.Medium,
        color = color,
        lineHeight = 14.4.sp,
        maxLines = 1
    )
}

