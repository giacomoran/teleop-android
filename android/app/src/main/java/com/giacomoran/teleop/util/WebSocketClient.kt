package com.giacomoran.teleop.util

import android.util.Log
import com.giacomoran.teleop.ui.ControlPadOutput
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import okhttp3.*
import org.json.JSONObject
import java.security.cert.X509Certificate
import java.util.concurrent.TimeUnit
import javax.net.ssl.*

/**
 * WebSocket client for sending pose and control data to the teleop server.
 */
class WebSocketClient(private val serverIp: String, private val serverPort: Int) {
    private val TAG = "WebSocketClient"

    private val client = OkHttpClient.Builder()
        .pingInterval(30, TimeUnit.SECONDS)
        .hostnameVerifier { _, _ -> true } // Accept all hostnames (for self-signed certs)
        .sslSocketFactory(createTrustAllSSLSocketFactory(), createTrustAllTrustManager())
        .build()

    /**
     * Creates an SSL socket factory that trusts all certificates.
     * WARNING: This is insecure and should only be used for development with self-signed certificates.
     */
    private fun createTrustAllSSLSocketFactory(): SSLSocketFactory {
        val trustAllCerts = arrayOf<TrustManager>(
            object : X509TrustManager {
                override fun checkClientTrusted(chain: Array<out X509Certificate>?, authType: String?) {}
                override fun checkServerTrusted(chain: Array<out X509Certificate>?, authType: String?) {}
                override fun getAcceptedIssuers(): Array<X509Certificate> = arrayOf()
            }
        )
        val sslContext = SSLContext.getInstance("TLS")
        sslContext.init(null, trustAllCerts, java.security.SecureRandom())
        return sslContext.socketFactory
    }

    /**
     * Creates a trust manager that trusts all certificates.
     */
    private fun createTrustAllTrustManager(): X509TrustManager {
        return object : X509TrustManager {
            override fun checkClientTrusted(chain: Array<out X509Certificate>?, authType: String?) {}
            override fun checkServerTrusted(chain: Array<out X509Certificate>?, authType: String?) {}
            override fun getAcceptedIssuers(): Array<X509Certificate> = arrayOf()
        }
    }

    private var webSocket: WebSocket? = null
    private var request: Request? = null

    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    val connectionState: StateFlow<ConnectionState> = _connectionState

    private var reconnectJob: Job? = null
    private var shouldReconnect = false
    private var retryDelayMs = 1000L // Start with 1 second
    private val maxRetryDelayMs = 30000L // Max 30 seconds

    sealed class ConnectionState {
        object Disconnected : ConnectionState()
        object Connecting : ConnectionState()
        object Connected : ConnectionState()
    }

    private val listener = object : WebSocketListener() {
        override fun onOpen(webSocket: WebSocket, response: Response) {
            Log.d(TAG, "WebSocket connected")
            _connectionState.value = ConnectionState.Connected
            // Reset retry delay on successful connection
            retryDelayMs = 1000L
            reconnectJob?.cancel()
            reconnectJob = null
        }

        override fun onMessage(webSocket: WebSocket, text: String) {
            Log.d(TAG, "Received message: $text")
            // Server can send messages back if needed
        }

        override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
            Log.d(TAG, "WebSocket closing: $code - $reason")
            webSocket.close(1000, null)
        }

        override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
            Log.d(TAG, "WebSocket closed: $code - $reason")
            _connectionState.value = ConnectionState.Disconnected
            // Attempt reconnection if we should be connected
            if (shouldReconnect) {
                scheduleReconnect()
            }
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            val errorDetails = buildString {
                append("WebSocket failure: ${t.message}")
                if (response != null) {
                    append(" | Response code: ${response.code}")
                    append(" | Response message: ${response.message}")
                    // Log response headers for debugging HTTP 101 issues
                    if (response.headers.size > 0) {
                        append(" | Headers: ${response.headers}")
                    }
                } else {
                    append(" | No response received")
                }
            }
            Log.e(TAG, errorDetails, t)
            // Don't set Error state - just log it and treat as disconnected
            // This prevents error messages from showing in the UI
            _connectionState.value = ConnectionState.Disconnected
            // Attempt reconnection if we should be connected
            if (shouldReconnect) {
                scheduleReconnect()
            }
        }
    }

    /**
     * Connect to the WebSocket server.
     */
    fun connect() {
        if (_connectionState.value is ConnectionState.Connected) {
            Log.w(TAG, "Already connected")
            return
        }

        shouldReconnect = true
        attemptConnection()
    }

    /**
     * Internal method to attempt a connection.
     */
    private fun attemptConnection() {
        val url = "wss://$serverIp:$serverPort/ws"
        Log.d(TAG, "Connecting to $url")

        request = Request.Builder()
            .url(url)
            .build()

        _connectionState.value = ConnectionState.Connecting
        webSocket = client.newWebSocket(request!!, listener)
    }

    /**
     * Schedule a reconnection attempt with exponential backoff.
     */
    private fun scheduleReconnect() {
        if (reconnectJob?.isActive == true) {
            return // Already scheduled
        }

        reconnectJob = CoroutineScope(Dispatchers.IO).launch {
            Log.d(TAG, "Scheduling reconnect in ${retryDelayMs}ms")
            delay(retryDelayMs)

            if (shouldReconnect && _connectionState.value !is ConnectionState.Connected) {
                Log.d(TAG, "Attempting reconnection...")
                attemptConnection()
                // Exponential backoff: double the delay, capped at maxRetryDelayMs
                retryDelayMs = minOf(retryDelayMs * 2, maxRetryDelayMs)
            }
        }
    }

    /**
     * Disconnect from the WebSocket server.
     */
    fun disconnect() {
        shouldReconnect = false
        reconnectJob?.cancel()
        reconnectJob = null
        webSocket?.close(1000, "Client disconnecting")
        webSocket = null
        request = null
        _connectionState.value = ConnectionState.Disconnected
    }

    /**
     * Send pose data to the server.
     * @param position FloatArray of 3 elements [x, y, z]
     * @param quaternion FloatArray of 4 elements [x, y, z, w]
     */
    fun sendPose(position: FloatArray, quaternion: FloatArray) {
        if (_connectionState.value !is ConnectionState.Connected) {
            return
        }

        try {
            val message = JSONObject().apply {
                put("type", "pose")
                put("data", JSONObject().apply {
                    put("position", JSONObject().apply {
                        put("x", position[0].toDouble())
                        put("y", position[1].toDouble())
                        put("z", position[2].toDouble())
                    })
                    put("orientation", JSONObject().apply {
                        put("x", quaternion[0].toDouble())
                        put("y", quaternion[1].toDouble())
                        put("z", quaternion[2].toDouble())
                        put("w", quaternion[3].toDouble())
                    })
                })
            }

            webSocket?.send(message.toString())
        } catch (e: Exception) {
            Log.e(TAG, "Error sending pose data", e)
        }
    }

    /**
     * Send control pad data to the server.
     * @param control ControlPadOutput containing x, y, isFineControl, and isActive
     */
    fun sendControl(control: ControlPadOutput) {
        if (_connectionState.value !is ConnectionState.Connected) {
            return
        }

        try {
            val message = JSONObject().apply {
                put("type", "control")
                put("data", JSONObject().apply {
                    put("x", control.x.toDouble())
                    put("y", control.y.toDouble())
                    put("isFineControl", control.isFineControl)
                    put("isActive", control.isActive)
                })
            }

            webSocket?.send(message.toString())
        } catch (e: Exception) {
            Log.e(TAG, "Error sending control data", e)
        }
    }

}

