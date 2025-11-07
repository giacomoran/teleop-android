package com.giacomoran.teleop

import android.content.Intent
import android.os.Bundle
import androidx.activity.ComponentActivity

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Launch RequirementsActivity immediately
        val intent = Intent(this, RequirementsActivity::class.java)
        startActivity(intent)
        finish()
    }
}
