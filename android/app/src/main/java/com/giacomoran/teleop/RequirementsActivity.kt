package com.giacomoran.teleop

import android.content.Intent
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Cancel
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.lifecycle.lifecycleScope
import com.giacomoran.teleop.ui.theme.TeleopTheme
import com.giacomoran.teleop.util.ARCoreHelper
import com.giacomoran.teleop.util.CameraPermissionHelper
import com.google.ar.core.ArCoreApk
import kotlinx.coroutines.launch

/**
 * State for a single requirement
 */
sealed class RequirementState {
    object Checking : RequirementState()
    object Met : RequirementState()
    object Unmet : RequirementState()
}

/**
 * Data class representing a requirement
 */
data class Requirement(
    val id: String,
    val name: String,
    val description: String,
    var state: RequirementState = RequirementState.Checking,
    val canGrant: Boolean = false
)

class RequirementsActivity : ComponentActivity() {
    private var userRequestedInstall = true
    private val requirementsState = mutableStateOf<List<Requirement>>(emptyList())

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        if (isGranted) {
            updateRequirementState("camera", RequirementState.Met)
        } else {
            updateRequirementState("camera", RequirementState.Unmet)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        initializeRequirements()
        checkRequirements()

        setContent {
            TeleopTheme {
                val requirements by requirementsState
                RequirementsScreen(
                    requirements = requirements,
                    onGrantClick = { requirementId ->
                        handleGrantClick(requirementId)
                    },
                    onStartClick = {
                        startARActivity()
                    }
                )
            }
        }
    }

    override fun onResume() {
        super.onResume()
        // Re-check requirements when activity resumes (e.g., after returning from settings)
        checkRequirements()
    }

    private fun initializeRequirements() {
        requirementsState.value = listOf(
            Requirement(
                id = "arcore_support",
                name = "ARCore Support",
                description = "Device must support ARCore",
                state = RequirementState.Checking,
                canGrant = false
            ),
            Requirement(
                id = "arcore_installation",
                name = "ARCore Installation",
                description = "Google Play Services for AR must be installed",
                state = RequirementState.Checking,
                canGrant = true
            ),
            Requirement(
                id = "camera",
                name = "Camera Permission",
                description = "Camera access is required for AR",
                state = RequirementState.Checking,
                canGrant = true
            )
        )
    }

    private fun checkRequirements() {
        lifecycleScope.launch {
            // Check ARCore availability
            checkARCoreSupport()

            // Check ARCore installation
            checkARCoreInstallation()

            // Check camera permission
            checkCameraPermission()
        }
    }

    private fun checkARCoreSupport() {
        val availability = ARCoreHelper.checkAvailability(this)
        val state = when (availability) {
            ArCoreApk.Availability.SUPPORTED_INSTALLED,
            ArCoreApk.Availability.SUPPORTED_APK_TOO_OLD,
            ArCoreApk.Availability.SUPPORTED_NOT_INSTALLED -> RequirementState.Met
            ArCoreApk.Availability.UNSUPPORTED_DEVICE_NOT_CAPABLE -> RequirementState.Unmet
            else -> RequirementState.Checking
        }
        updateRequirementState("arcore_support", state)
    }

    private fun checkARCoreInstallation() {
        val installStatus = ARCoreHelper.requestInstall(this, false)
        val state = when (installStatus) {
            ArCoreApk.InstallStatus.INSTALLED -> RequirementState.Met
            ArCoreApk.InstallStatus.INSTALL_REQUESTED -> RequirementState.Checking
            null -> RequirementState.Unmet
            else -> RequirementState.Unmet
        }
        updateRequirementState("arcore_installation", state)
    }

    private fun checkCameraPermission() {
        val state = if (CameraPermissionHelper.hasCameraPermission(this)) {
            RequirementState.Met
        } else {
            RequirementState.Unmet
        }
        updateRequirementState("camera", state)
    }

    private fun updateRequirementState(id: String, newState: RequirementState) {
        requirementsState.value = requirementsState.value.map { requirement ->
            if (requirement.id == id) {
                requirement.copy(state = newState)
            } else {
                requirement
            }
        }
    }

    private fun handleGrantClick(requirementId: String) {
        when (requirementId) {
            "arcore_installation" -> {
                userRequestedInstall = true
                val installStatus = ARCoreHelper.requestInstall(this, true)
                if (installStatus == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                    updateRequirementState("arcore_installation", RequirementState.Checking)
                }
            }
            "camera" -> {
                requestPermissionLauncher.launch(android.Manifest.permission.CAMERA)
            }
        }
    }

    private fun startARActivity() {
        val intent = Intent(this, ARActivity::class.java)
        startActivity(intent)
    }

}

@Composable
fun RequirementsScreen(
    requirements: List<Requirement>,
    onGrantClick: (String) -> Unit,
    onStartClick: () -> Unit
) {
    val allMet = requirements.all { it.state == RequirementState.Met }

    Scaffold(
        modifier = Modifier.fillMaxSize()
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
                .padding(24.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(24.dp)
        ) {
            Text(
                text = "AR Requirements",
                style = MaterialTheme.typography.headlineLarge,
                fontWeight = FontWeight.Bold
            )

            Text(
                text = "Please ensure all requirements are met before starting",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )

            LazyColumn(
                modifier = Modifier
                    .fillMaxWidth()
                    .weight(1f),
                verticalArrangement = Arrangement.spacedBy(16.dp)
            ) {
                items(requirements) { requirement ->
                    RequirementItem(
                        requirement = requirement,
                        onGrantClick = { onGrantClick(requirement.id) }
                    )
                }
            }

            Button(
                onClick = onStartClick,
                enabled = allMet,
                modifier = Modifier
                    .fillMaxWidth()
                    .height(56.dp)
            ) {
                Text(
                    text = "START",
                    style = MaterialTheme.typography.titleMedium
                )
            }
        }
    }
}

@Composable
fun RequirementItem(
    requirement: Requirement,
    onGrantClick: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        elevation = CardDefaults.cardElevation(defaultElevation = 2.dp)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalArrangement = Arrangement.spacedBy(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Status indicator on the left
            when (requirement.state) {
                is RequirementState.Checking -> {
                    CircularProgressIndicator(
                        modifier = Modifier.size(24.dp),
                        strokeWidth = 2.dp
                    )
                }
                is RequirementState.Met -> {
                    Icon(
                        imageVector = Icons.Default.CheckCircle,
                        contentDescription = "Met",
                        tint = MaterialTheme.colorScheme.primary,
                        modifier = Modifier.size(24.dp)
                    )
                }
                is RequirementState.Unmet -> {
                    Icon(
                        imageVector = Icons.Default.Cancel,
                        contentDescription = "Unmet",
                        tint = MaterialTheme.colorScheme.error,
                        modifier = Modifier.size(24.dp)
                    )
                }
            }

            Column(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                Text(
                    text = requirement.name,
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.SemiBold
                )
                Text(
                    text = requirement.description,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }

            // Grant button on the right (only show for unmet requirements that can be granted)
            if (requirement.state is RequirementState.Unmet && requirement.canGrant) {
                Button(
                    onClick = onGrantClick,
                    modifier = Modifier.height(36.dp)
                ) {
                    Text("GRANT", style = MaterialTheme.typography.labelSmall)
                }
            }
        }
    }
}

