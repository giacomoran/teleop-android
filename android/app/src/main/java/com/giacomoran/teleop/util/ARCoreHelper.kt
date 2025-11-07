package com.giacomoran.teleop.util

import android.app.Activity
import com.google.ar.core.ArCoreApk
import com.google.ar.core.ArCoreApk.InstallStatus
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException

object ARCoreHelper {
    /**
     * Check if ARCore is supported on this device
     */
    fun checkAvailability(activity: Activity): ArCoreApk.Availability {
        return ArCoreApk.getInstance().checkAvailability(activity)
    }

    /**
     * Request installation of Google Play Services for AR
     * Returns the installation status
     */
    fun requestInstall(
        activity: Activity,
        userRequestedInstall: Boolean
    ): InstallStatus? {
        return try {
            ArCoreApk.getInstance().requestInstall(activity, userRequestedInstall)
        } catch (e: UnavailableUserDeclinedInstallationException) {
            null
        } catch (e: Exception) {
            null
        }
    }
}

