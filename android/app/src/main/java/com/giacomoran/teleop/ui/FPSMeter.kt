package com.giacomoran.teleop.ui

import androidx.compose.foundation.layout.*
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
import com.google.ar.core.TrackingState

/**
 * ARCore Tracking Meter component that displays:
 * - ARCore updates per second (not rendering FPS)
 * - Real-time tracking status with clear labels
 * - Stall detection (when ARCore stops providing updates)
 */
@Composable
fun FPSMeter(
    updatesPerSecond: Float,
    cameraTrackingState: TrackingState,
    isStalled: Boolean,
    modifier: Modifier = Modifier
) {
    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(12.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        // Updates per second display - use neutral colors
        Text(
            text = "${updatesPerSecond.toInt()} Hz",
            fontSize = 12.sp,
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            lineHeight = 14.4.sp
        )

        // Tracking status with clear label
        val (statusLabel, statusColor) = when {
            isStalled -> "Stalled" to Color(0xFFF44336)
            cameraTrackingState == TrackingState.TRACKING -> "Camera Active" to Color(0xFF4CAF50)
            cameraTrackingState == TrackingState.PAUSED -> "Camera Paused" to Color(0xFFFFC107)
            else -> "Camera Stopped" to Color(0xFFF44336)
        }

        Text(
            text = statusLabel,
            fontSize = 12.sp,
            fontFamily = FontFamily.Default,
            fontWeight = FontWeight.Medium,
            color = statusColor,
            lineHeight = 14.4.sp
        )
    }
}

